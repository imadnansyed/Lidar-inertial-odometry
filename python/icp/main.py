from pathlib import Path
# from synthetic_data import GenerateSyntheticData
from icp import ICP
import numpy as np


if __name__ == "__main__":

    # generator = GenerateSyntheticData(num_points=10_000, seed=42)
    # source, target, defined_R, defined_t = generator.generate()

    

        # ICP Type 0
        icp_0 = ICP(
            type=1,
            max_iterations=20,
            max_distance_toleration=np.inf,
            tolerance=1e-6,
            plot=False
        )

        R_est, t_est, T_rel, poses = icp_0.run()

        R_est = np.array(R_est)       # (N-1, 3, 3)
        t_est = np.array(t_est)       # (N-1, 3)
        T_rel = np.array(T_rel)       # (N-1, 4, 4)
        poses = np.array(poses)       # (N, 4, 4)

        np.savez_compressed(
                                "results/estimated_odometry.npz",
                                R_est=R_est,
                                t_est=t_est,
                                T_rel=T_rel,
                                poses=poses
                            )

        print("Odometry results saved successfully!")





