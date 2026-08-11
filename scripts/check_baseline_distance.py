#!/usr/bin/env python3
"""
Kiểm tra khoảng cách thật (mét) giữa vị trí robot lúc quét scan_idx
và lúc quét từng map_idx, dùng ĐÚNG công thức C++ đang dùng:
    world_pose(idx) = poses[idx] (4x4, dạng 3x4 doc tu file) mo rong hang [0,0,0,1]
    world_pose_lidar(idx) = world_pose(idx) @ Tr        (Tr: LiDAR -> cam0)
    vi_tri_that(idx) = world_pose_lidar(idx)[:3, 3]

Usage:
    python3 check_baseline_distance.py <poses.txt> <calib.txt> <scan_idx> <map_idx_1> [map_idx_2] ...
"""
import sys
import numpy as np


def load_poses(path):
    poses = []
    with open(path) as f:
        for line in f:
            vals = list(map(float, line.split()))
            T = np.eye(4)
            T[:3, :4] = np.array(vals).reshape(3, 4)
            poses.append(T)
    return poses


def load_tr(calib_path):
    with open(calib_path) as f:
        for line in f:
            if line.startswith("Tr:"):
                vals = list(map(float, line[3:].split()))
                Tr = np.eye(4)
                Tr[:3, :4] = np.array(vals).reshape(3, 4)
                return Tr
    raise RuntimeError(f"Khong tim thay dong 'Tr:' trong {calib_path}")


def main():
    if len(sys.argv) < 5:
        print(f"Usage: {sys.argv[0]} <poses.txt> <calib.txt> <scan_idx> <map_idx_1> [map_idx_2] ...")
        sys.exit(1)

    poses_path = sys.argv[1]
    calib_path = sys.argv[2]
    scan_idx = int(sys.argv[3])
    map_indices = [int(x) for x in sys.argv[4:]]

    poses = load_poses(poses_path)
    Tr = load_tr(calib_path)

    def lidar_position(idx):
        world_lidar = poses[idx] @ Tr
        return world_lidar[:3, 3]

    scan_pos = lidar_position(scan_idx)

    print(f"Vi tri LiDAR tai scan_idx={scan_idx}: {scan_pos}")
    print("")
    print(f"{'map_idx':<10}{'khoang_cach_frame':<20}{'khoang_cach_that(m)':<20}")
    for idx in map_indices:
        map_pos = lidar_position(idx)
        dist = np.linalg.norm(map_pos - scan_pos)
        print(f"{idx:<10}{abs(idx - scan_idx):<20}{dist:<20.2f}")


if __name__ == "__main__":
    main()