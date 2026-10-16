import numpy as np
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt


class ICP:

    """
         a. type : 0 for point-to-point ICP, 1 for point-to-plane ICP 
         b. source point cloud : 3D numpy array of shape (N, 3)
         c. target point cloud : 3D numpy array of shape (M, 3)
         d. max_iterations : maximum number of iterations to run the ICP algorithm
         e. max_distance_toleration : maximum distance to consider a correspondence valid
         f. tolerance : convergence criteria based on RMSE error change
         """
    def __init__(self, type : int, source_cloud : np.ndarray, target_cloud : np.ndarray , R_gt : np.ndarray, t_gt : np.ndarray, max_iterations : int = 100, max_distance_toleration=np.inf, tolerance : float = 1e-6):
        
        self.type = type
        self.source_cloud = source_cloud
        self.target_cloud = target_cloud
        self.max_iterations = max_iterations
        self.max_distance_toleration = max_distance_toleration
        self.tolerance = tolerance
        self.R_gt = R_gt
        self.t_gt = t_gt

    def find_correspondences(self, source, target):
        #  find correspondences between source and target point clouds using nearest neighbor search
        tree = cKDTree(target)
        distances, indices = tree.query(source, k=1)
        print("indices",indices[:10])
        print("distances",distances[:10])

        matched_target_points = target[indices]

        return matched_target_points, distances

    def compute_transform(self, source_cloud, target_cloud):
        # compute centroids of source and matched target points
        centroid_source = np.mean(source_cloud, axis=0)
        centroid_target = np.mean(target_cloud, axis=0)

        A = source_cloud - centroid_source
        B = target_cloud - centroid_target

        # compute covariance matrix
        H = A.T@ B

        # SVD
        U, S, Vt = np.linalg.svd(H)

        # compute rotation matrix
        R = Vt.T @ U.T

        # reflection correction
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # check for 
        # compute translation vector
        t = centroid_target - R @ centroid_source

        return R, t

    def p_to_point(self):


        # Keep working copy 
        aligned = self.source_cloud.copy()
        R_total = np.eye(3)
        t_total = np.zeros(3)

        previous_error = np.inf

        for iteration in range(self.max_iterations):
            #  find correspondences 
            matched_target_points, distances = self.find_correspondences(aligned, self.target_cloud)

            # reject correspondences that are too far away
            mask = distances < self.max_distance_toleration

            if mask.sum() <= 3:
                raise RuntimeError("Too few valid correspondences.")

            # compute RMSE error 
            rmse = np.sqrt(np.mean(distances**2))

            print(f"Iteration {iteration+1}, RMSE error: {rmse}")

            if abs(previous_error - rmse) < self.tolerance:
                break
            previous_error = rmse
            
            # estimate incremental transformation
            delta_R, delta_t = self.compute_transform(aligned[mask], matched_target_points[mask])

            # apply to working cloud
            aligned = (delta_R @ aligned.T).T + delta_t

            # accumulate total transformation
            R_total = delta_R @ R_total
            t_total = delta_R @ t_total + delta_t


                
        return R_total, t_total, aligned, rmse, iteration+1

    def p_to_plane(self):

        pass 

    def rotation_error_degrees(self, R_gt, R_est):
        R_error = R_gt.T @ R_est

        cos_angle = (np.trace(R_error) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)

        return np.rad2deg(np.arccos(cos_angle))

    
    def show_results(self, R_est, t_est, iterations, aligned, rmse):
        print("Ground-truth rotation:\n", self.R_gt)
        print("\nEstimated rotation:\n", R_est)

        print("\nGround-truth translation:", self.t_gt.reshape(3,1))
        print("Estimated translation:", t_est)

        print("\nTranslation error:",
            np.linalg.norm(self.t_gt - t_est))

        print("Rotation error (degrees):",
            self.rotation_error_degrees(self.R_gt, R_est))

        print("Final nearest-neighbor RMSE:", rmse)
        print("Iterations:", iterations)

        fig = plt.figure(figsize=(12, 5))

        ax1 = fig.add_subplot(121, projection="3d")
        ax1.scatter(*self.source_cloud.T, s=5, label="Source")
        ax1.scatter(*self.target_cloud.T, s=5, label="Target")
        ax1.set_title("Before ICP")
        ax1.legend()

        ax2 = fig.add_subplot(122, projection="3d")
        ax2.scatter(*self.target_cloud.T, s=5, label="Target")
        ax2.scatter(*aligned.T, s=5, label="Aligned source")
        ax2.set_title("After ICP")
        ax2.legend()

        plt.tight_layout()
        plt.show()               


    def run(self):
        # run the ICP algorithm based on the type specified during initialization
        if self.type == 0:
            print("Running point-to-point ICP...")
            R_est, t_est, aligned, rmse, iterations = self.p_to_point()

            # show results 
            self.show_results(R_est, t_est, iterations, aligned, rmse)

            return R_est, t_est

        elif self.type == 1:
            print("Running point-to-plane ICP...")
            # Implement point-to-plane ICP algorithm here
            # For demonstration, we will just return the defined transformation
            return self.defined_R, self.defined_t
        else:
            raise ValueError("Invalid type specified. Use 0 for point-to-point or 1 for point-to-plane.")