import numpy as np
from load_bin import load

def voxel_downsample(points, voxel_size=0.2):
    if voxel_size <= 0:
        raise ValueError("voxel_size must be positive")

    # Assign each point to a voxel.
    voxel_indices = np.floor(points / voxel_size).astype(np.int64)

    # Group points by their unique voxel coordinates.
    unique_voxels, inverse = np.unique(
        voxel_indices,
        axis=0,
        return_inverse=True
    )

    # Sum points in each voxel.
    sums = np.zeros((len(unique_voxels), 3), dtype=np.float64)
    np.add.at(sums, inverse, points)

    # Count points in each voxel.
    counts = np.bincount(inverse, minlength=len(unique_voxels))

    # Compute one centroid per occupied voxel.
    return sums / counts[:, None]


points = load()
# downsampled, counts = down_sample(points, 1)

# # Report results.
# print("Original points:", len(points))
# print("Downsampled points:", len(downsampled))
# print("Points removed:", len(points) - len(downsampled))

# reduction = 100 * (1 - len(downsampled) / len(points))
# print(f"Reduction: {reduction:.2f}%")
# print("Original shape:", points.shape)
# print("Downsampled shape:", downsampled.shape)

for size in [0.1, 0.2, 0.5]:
    downsampled = voxel_downsample(points, voxel_size=size)
    reduction = 100 * (1 - len(downsampled) / len(points))

    print(
        f"Voxel: {size:.1f} m | "
        f"Points: {len(downsampled)} | "
        f"Reduction: {reduction:.2f}%"
    )
