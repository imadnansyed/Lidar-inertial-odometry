from pathlib import Path
import numpy as np


def load_bin(source_path, target_path):

    # Check that both files exist
    if not source_path.exists():
        raise FileNotFoundError(f"Source scan not found: {source_path}")

    if not target_path.exists():
        raise FileNotFoundError(f"Target scan not found: {target_path}")

    # Load scans
    scan_source = np.fromfile(source_path, dtype=np.float32)
    scan_target = np.fromfile(target_path, dtype=np.float32)

    # Validate scan formats
    if scan_source.size % 4 != 0:
        raise ValueError(f"Invalid source scan format: {source_path}")

    if scan_target.size % 4 != 0:
        raise ValueError(f"Invalid target scan format: {target_path}")

    # Reshape into (N, 4)
    scan_source = scan_source.reshape(-1, 4)
    scan_target = scan_target.reshape(-1, 4)

    # Extract XYZ coordinates
    points_source = scan_source[:, :3]
    points_target = scan_target[:, :3]

    # Return the current consecutive pair
    return points_source, points_target