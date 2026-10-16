from synthetic_data import GenerateSyntheticData
from icp import ICP

import numpy as np

if __name__ == "__main__":
    generator = GenerateSyntheticData(num_points=10_000, seed=42)
    source, target, defined_angle, defined_t = generator.generate()

    icp = ICP(type=0, source_cloud=source, target_cloud=target)
    total_R, total_t = icp.run()

    print(f"""
    R : {total_R}

    
    t = {total_t} 
""")
    # recover R in degrees
    cos_theta = (np.trace(total_R) - 1)/2
    
    # Protect against floating-point errors
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    total_rad = np.arccos(cos_theta)
    total_deg = np.degrees(total_rad)

    print(f"defined angle = {defined_angle} \n Recovered Angle = {total_deg}")
    print(f"\ndefined t = {defined_t} \n Recovered t = {total_t}")

    

    