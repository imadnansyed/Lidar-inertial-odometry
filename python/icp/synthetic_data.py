import numpy as np

class GenerateSyntheticData:
    def __init__(self, num_points: int= 10_000, seed : int = 42, noise : float = 0.002):
        self.num_points = num_points
        self.seed = seed
        self.noise = noise

    def get_source(self):
        rng = np.random.default_rng(self.seed)

        # Floor: z = 0
        floor = rng.uniform(-2, 2, (self.num_points, 3))
        floor[:, 2] = 0

        # Wall: x = 2
        wall_x = rng.uniform(-2, 2, (self.num_points, 3))
        wall_x[:, 0] = 2

        # Wall: y = 2
        wall_y = rng.uniform(-2, 2, (self.num_points, 3))
        wall_y[:, 1] = 2

        points = np.vstack((floor, wall_x, wall_y))

        # Small measurement noise
        points += rng.normal(0, self.noise, points.shape)

        return points

    def make_transform(self, rotation_angle: float, translation_vector: np.ndarray = np.array([0.5, -0.3, 0.2])):

        angle = np.deg2rad(rotation_angle)

        # Create a rotation matrix
        R = np.array([[np.cos(angle), -np.sin(angle), 0], 
                      [np.sin(angle), np.cos(angle), 0], 
                      [0,0,1]])

        # Create a translation matrix
        t = np.array(translation_vector).reshape((3, 1))
        print(f"Transform generated for {rotation_angle} degree and \n t:{t.T}" )
        return R, t
 

    def generate(self):

        # gen source point cloud
        source = self.get_source()

        # get defined transformation
        rotation_angle=30
        R, t = self.make_transform(rotation_angle)

        # get target point cloud by applying the transformation to the source point cloud
        target = (R @ source.T).T + t.T
        print("target point cloud generated")
        return source, target, R, t