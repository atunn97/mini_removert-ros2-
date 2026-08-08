import numpy as np
import os

def bin_to_pcd(bin_path, pcd_path):
    points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    xyz = points[:, :3]
    n = xyz.shape[0]
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA binary
"""
    with open(pcd_path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(xyz.astype(np.float32).tobytes())

def convert_sequence(bin_dir, pcd_dir, limit=None):
    os.makedirs(pcd_dir, exist_ok=True)
    files = sorted(f for f in os.listdir(bin_dir) if f.endswith('.bin'))
    if limit:
        files = files[:limit]
    for i, fname in enumerate(files):
        bin_path = os.path.join(bin_dir, fname)
        pcd_path = os.path.join(pcd_dir, fname.replace('.bin', '.pcd'))
        bin_to_pcd(bin_path, pcd_path)
        if i % 100 == 0:
            print(f"Converted {i}/{len(files)}")
    print("Done.")

if __name__ == "__main__":
    convert_sequence(
        "/đường/dẫn/sequences/00/velodyne",
        "/đường/dẫn/sequences/00/pcd",
        limit=20  # thử 20 file trước
    )
