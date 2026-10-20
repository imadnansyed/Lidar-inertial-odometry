from icp_point_to_point import PointToPoint
from icp_point_to_plane import PointToPlane
import numpy as np
class ICP:
    """  
    a. type : 0 for point-to-point ICP, 1 for point-to-plane ICP , default = 0 
    """

    def __init__(self, source_cloud : np.ndarray, target_cloud : np.ndarray, R_gt : np.ndarray, t_gt : np.ndarray,plot:bool=False ,type:int = 0, max_iterations : int = 100, max_distance_toleration=np.inf, tolerance : float = 1e-6):

        self.type = type
        self.plot = plot
        self.p2Point = PointToPoint(source_cloud=source_cloud, target_cloud=target_cloud, R_gt=R_gt, t_gt=t_gt, max_distance_toleration=max_distance_toleration, max_iterations=max_iterations, tolerance=tolerance)
        self.p2Plane = PointToPlane(source_cloud=source_cloud, target_cloud=target_cloud, R_gt=R_gt, t_gt=t_gt, max_distance_toleration=max_distance_toleration, max_iterations=max_iterations, tolerance=tolerance)

    def run(self):
        if self.type==0:
            R, t = self.p2Point.run(self.plot)
        else:
            R , t = self.p2Plane.run(self.plot)

        return R, t