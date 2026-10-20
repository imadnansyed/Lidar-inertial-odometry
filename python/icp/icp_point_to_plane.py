import numpy as np
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt


class PointToPlane:

    """
         b. source point cloud : 3D numpy array of shape (N, 3)
         c. target point cloud : 3D numpy array of shape (M, 3)
         d. max_iterations : maximum number of iterations to run the ICP algorithm
         e. max_distance_toleration : maximum distance to consider a correspondence valid
         f. tolerance : convergence criteria based on RMSE error change
         """
    def __init__(self, source_cloud : np.ndarray, target_cloud : np.ndarray , R_gt : np.ndarray, t_gt : np.ndarray, max_iterations : int , max_distance_toleration, tolerance : float, k_normals : int = 20, R_init : np.ndarray = None, t_init : np.ndarray = None):
        
        self.source_cloud = source_cloud
        self.target_cloud = target_cloud
        self.max_iterations = max_iterations
        self.max_distance_toleration = max_distance_toleration
        self.tolerance = tolerance
        self.R_gt = R_gt
        self.t_gt = t_gt
        self.R_init = R_init
        self.t_init = t_init

        # The target never changes during ICP, so build these ONCE.
        self.target_tree = cKDTree(self.target_cloud)
        self.target_normals = self.compute_normals(self.target_cloud, k_normals)


    def find_correspondences(self, source, target):

        #  find correspondences between source and target point clouds using nearest neighbor search

        distances, indices = self.target_tree.query(source, k=1)

         # Reject distant correspondences.
        mask = (
            (distances < self.max_distance_toleration)
            & np.isfinite(distances)
        )

        if mask.sum() < 6:
            raise ValueError("Not enough matches after masked")

        matched_target_points = target[indices[mask]]
        
        matched_source = source[mask]

        matched_target_normals = self.target_normals[indices[mask]]


        return (matched_source, matched_target_points, matched_target_normals)

    def compute_normals(self, points:np.ndarray, k:int=20):

        tree = self.target_tree if points is self.target_cloud else cKDTree(points)
        _, indices = tree.query(points, k)
        normals = []

        for neighbour_idx in indices:
            neighbors = points[neighbour_idx]

            # center it
            centered = neighbors - neighbors.mean(axis=0)

            # compute covariance matrix
            covariance = centered.T @ centered / len(neighbors)

            # compute eigen vectors and values
            eigen_values, eigen_vectors = np.linalg.eigh(covariance)

            # choose the eigen vector which gives smallest variance so it will be normal
            normal = eigen_vectors[: , np.argmin(eigen_values)] # eigen vectors are stored as cols
            normal /= np.linalg.norm(normal)

            # update normals
            normals.append(normal)

        return np.asarray(normals)

    
    def skew(self, v: np.ndarray) -> np.ndarray:
        """3x3 skew-symmetric matrix such that skew(a) @ b == cross(a, b)."""
        return np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ])

    
    def exp_so3(self, omega: np.ndarray) -> np.ndarray:
        """Rotation vector -> rotation matrix (Rodrigues)."""
        angle = np.linalg.norm(omega)
        if angle < 1e-12:
            return np.eye(3) + self.skew(omega)  # first-order approximation
        K = self.skew(omega / angle)
        return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)
    

    def compute_transform(self, source_cloud, target_cloud, normals):

        # Point-to-plane residual:
        # r = n.T @ (p - q)
        residual = np.sum(
            normals * (source_cloud - target_cloud),
            axis=1
        )

        # Linearized rotational Jacobian:
        # n.T @ (omega x p) = (p x n).T @ omega
        cross = np.cross(source_cloud, normals)

        # Unknowns: [omega_x, omega_y, omega_z, tx, ty, tz]
        A = np.hstack((cross, normals))

        # Solve A @ delta = -residual
        delta, *_ = np.linalg.lstsq(
            A, -residual, rcond=None
        )

        omega = delta[:3]
        delta_t = delta[3:]

        # Convert rotation vector to a rotation matrix

        return self.exp_so3(omega), delta_t, np.linalg.norm(omega), np.linalg.norm(delta_t)



    def p_to_plane(self):

        R_total = np.eye(3) if self.R_init is None else np.asarray(self.R_init, dtype=float).copy()
        t_total = np.zeros(3) if self.t_init is None else np.asarray(self.t_init, dtype=float).ravel().copy()

        # Keep working copy 
        aligned = self.source_cloud @ R_total.T + t_total

        previous_rmse = np.inf
        rmse = np.inf

        for iteration in range(self.max_iterations):

            (matched_source, matched_target,matched_normals
            ) = self.find_correspondences(
                aligned,
                self.target_cloud)

            # Point-to-plane RMSE before this update.
            residual = np.sum(
                matched_normals * (matched_source - matched_target),
                axis=1
            )

            rmse = np.sqrt(np.mean(residual ** 2))

            print(
                f"Iteration {iteration + 1}: "
                f"matches={len(matched_source)}, "
                f"plane RMSE={rmse:.6f}"
            )

            # The current pose is already sufficiently converged.
            if (
                np.isfinite(previous_rmse)
                and abs(previous_rmse - rmse) < self.tolerance
            ):
                break

            previous_rmse = rmse

            # Estimate one incremental transformation.
            delta_R, delta_t, rot_step, trans_step = self.compute_transform(
                matched_source,
                matched_target,
                matched_normals
            )

            # Apply update to ALL currently aligned source points.
            aligned = (delta_R @ aligned.T).T + delta_t

            # Accumulate the total transformation.
            R_total = delta_R @ R_total
            t_total = delta_R @ t_total + delta_t

            # Stop if the incremental pose update is tiny.
            # Rotation (rad) and translation (m) checked separately
            if rot_step < self.tolerance and trans_step < self.tolerance:
                break

        # Recompute final RMSE on the final aligned cloud.
        (
            matched_source,
            matched_target,
            matched_normals
        ) = self.find_correspondences(
            aligned,
            self.target_cloud,
        )

        residual = np.sum(
            matched_normals * (matched_source - matched_target),
            axis=1
        )

        rmse = np.sqrt(np.mean(residual ** 2))

        return (
            R_total,
            t_total,
            aligned,
            rmse,
            iteration + 1
        )
       
    def rotation_error_degrees(self, R_gt, R_est):
        R_error = R_gt.T @ R_est

        cos_angle = (np.trace(R_error) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)

        return np.rad2deg(np.arccos(cos_angle))

    def show_results(self, R_est, t_est, iterations, aligned, rmse, plot):
        print("Ground-truth rotation:\n", self.R_gt)
        print("\nEstimated rotation:\n", R_est)

        print("\nGround-truth translation:", self.t_gt)
        print("Estimated translation:", t_est)

        print("\nTranslation error:",
            np.linalg.norm(self.t_gt - t_est))

        print("Rotation error (degrees):",
            self.rotation_error_degrees(self.R_gt, R_est))

        print("Final point-to-plane RMSE:", rmse)
        print("Iterations:", iterations)
        if plot:
            fig = plt.figure(figsize=(12, 5))

            ax1 = fig.add_subplot(121, projection="3d")
            ax1.scatter(*self.source_cloud.T, s=15, label="Source")
            ax1.scatter(*self.target_cloud.T, s=15, label="Target")
            ax1.set_title("Before ICP")
            ax1.legend()

            ax2 = fig.add_subplot(122, projection="3d")
            ax2.scatter(*self.target_cloud.T, s=15, label="Target")
            ax2.scatter(*aligned.T, s=15, label="Aligned source")
            ax2.set_title("After ICP")
            ax2.legend()

            plt.tight_layout()
            plt.show()               


    def run(self, plot : bool = False):
        print("Running point-to-plane ICP...")
        R_est, t_est, aligned, rmse, iterations = self.p_to_plane()

        self.show_results(R_est, t_est, iterations, aligned, rmse, plot)

        return R_est, t_est