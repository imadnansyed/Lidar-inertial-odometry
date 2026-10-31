import os
from pathlib import Path
from icp_point_to_point import PointToPoint
from icp_point_to_plane import PointToPlane
import numpy as np
from loader.load_bin import load_bin
from preprocess.voxel_downsampling import voxel_downsample

class ICP:
    """  
    a. type : 0 for point-to-point ICP, 1 for point-to-plane ICP , default = 0 
    """

    def __init__(self,  R_gt : np.ndarray = np.eye(3), t_gt : np.ndarray=np.zeros(3), plot:bool=False ,type:int = 0, max_iterations : int = 100, max_distance_toleration=np.inf, tolerance : float = 1e-6, dir_path:str = r"E:\Workspace\PC\AI Engineer\cv\projects\VO and SLAM\LIO Master 01\Lidar-inertial-odometry\data"):

        self.plot = plot
        self.dir_path = dir_path
        self.type = type
        self.max_iterations = max_iterations
        self.max_distance_toleration = max_distance_toleration
        self.tolerance = tolerance
        self.R_gt = R_gt
        self.t_gt = t_gt
        

    def run(self):

        bin_files = sorted(f for f in os.listdir(self.dir_path) if f.endswith(".bin"))
        velodyne_dir = Path(self.dir_path)

        R_est = []
        t_est = []
        T_rel = []

        poses = [np.eye(4)] # include 4x4 Trajectory moatrix


        # R_init and t_init
        R_init = np.eye(3)
        t_init = np.zeros(3)


        # R_global and t_global
        R_global = np.eye(3)
        t_global = np.zeros(3)

        for i in range(len(bin_files)-2):
    
            source_path = velodyne_dir / f"{i:06d}.bin"
            target_path = velodyne_dir / f"{i+1:06d}.bin"
    
            # load
            source_cloud, target_cloud = load_bin(source_path, target_path)
    
            # downsample
            source = voxel_downsample(source_cloud, 0.2)
            target = voxel_downsample(target_cloud, 0.2)

            if self.type == 0:
                        icp_to_run = PointToPoint(source_cloud=source, target_cloud=target, R_gt=self.R_gt, t_gt=self.t_gt, max_distance_toleration=self.max_distance_toleration, max_iterations=self.max_iterations, tolerance=self.tolerance, R_init=R_init, t_init=t_init)
            else:
                        icp_to_run = PointToPlane(source_cloud=source, target_cloud=target, R_gt=self.R_gt, t_gt=self.t_gt, max_distance_toleration=self.max_distance_toleration, max_iterations=self.max_iterations, tolerance=self.tolerance, R_init=R_init, t_init=t_init)

            R_new, t_new = icp_to_run.run(self.plot)

            # update the inits
            R_init = R_new.copy()
            t_init = t_new.copy()

            # update global
            R_global = R_new @ R_global
            t_global = R_new @ t_global + t_new

            T_global = np.eye(4)
            T_global[:3, :3] = R_global
            T_global[:3, 3] = t_global

            poses.append(T_global.copy())

            # Store relative rotation and translation
            R_est.append(R_new)
            t_new = np.asarray(t_new).reshape(3)
            t_est.append(t_new)

            # Build relative transformation
            T = np.eye(4)
            T[:3, :3] = R_new
            T[:3, 3] = t_new

            T_rel.append(T)


        return R_est, t_est, T_rel, poses