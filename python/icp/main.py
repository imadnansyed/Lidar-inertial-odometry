from synthetic_data import GenerateSyntheticData
from icp import ICP
import numpy as np

if __name__ == "__main__":
    generator = GenerateSyntheticData(num_points=10_000, seed=42)
    source, target, defined_R, defined_t = generator.generate()

    icp = ICP(type=0, source_cloud=source, target_cloud=target, max_iterations=100, R_gt=defined_R, t_gt=defined_t.reshape((3,)))

    total_R, total_t = icp.run()
