from synthetic_data import GenerateSyntheticData
from icp import ICP
import numpy as np
import time

if __name__ == "__main__":
    generator = GenerateSyntheticData(num_points=10_000, seed=42)
    source, target, defined_R, defined_t = generator.generate()

    # ICP Type 0
    icp_0 = ICP(
        type=0,
        source_cloud=source,
        target_cloud=target,
        max_iterations=50,
        R_gt=defined_R,
        t_gt=defined_t.ravel(),
        plot=False
    )

    start = time.perf_counter()
    R_0, t_0 = icp_0.run()
    time_0 = time.perf_counter() - start


    # ICP Type 1
    icp_1 = ICP(
        type=1,
        source_cloud=source,
        target_cloud=target,
        max_iterations=50,
        R_gt=defined_R,
        t_gt=defined_t.ravel(),
        plot=False
    )

    start = time.perf_counter()
    R_1, t_1 = icp_1.run()
    time_1 = time.perf_counter() - start


    # Compare execution times
    print(f"Point-to-point ICP (Type 0): {time_0:.3f} ms")
    print(f"Point-to-plane ICP (Type 1):  {time_1:.3f} ms")

    if time_0 < time_1:
        print(f"Type 0 is {time_1 / time_0:.2f}x faster")
    elif time_1 < time_0:
        print(f"Type 1 is {time_0 / time_1:.2f}x faster")
    else:
        print("Both took approximately the same time")
