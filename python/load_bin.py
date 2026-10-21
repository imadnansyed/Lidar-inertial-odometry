import numpy as np
import open3d as o3d
from pathlib import Path


def load(path:str = r"E:\Workspace\PC\AI Engineer\cv\projects\VO and SLAM\data\000000.bin", plot:bool = False):
    
    scan = np.fromfile(path, dtype = np.float32)

    print(scan.shape)
    print(scan.reshape(-1, 4).shape) # we get dimension as num_points * 4 so we have to reshape, -1 for auto caculate no of points while 4 means 4 values for each point
    scan = scan.reshape(-1, 4)

    points = scan[:, :3]
    intensity = scan[:, 3]

    print("Number of points:", len(points))
    print("Point cloud shape:", points.shape)
    print("First five points:\n", scan[:5])
    print("XYZ minimum:", points.min(axis=0))
    print("XYZ maximum:", points.max(axis=0))

    if plot: 
    # # Visualize points

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        o3d.visualization.draw_geometries([pcd])
    return points