#!/usr/bin/env python3
"""
Chon N map_idx cho mini_removert dua tren KHOANG CACH KHONG GIAN THAT (met),
khong dua tren so frame - de tranh loi parallax/occlusion khi baseline qua xa
(xem NOTES_viec4_N_sweep_debug.md).

Thuat toan:
  - Tinh vi tri LiDAR that (met) cho moi frame trong dataset (dung Tr tu calib.txt).
  - Voi N/2 map moi ben (truoc/sau scan_idx), chon frame co khoang cach THAT
    gan nhat voi cac muc tieu cach deu trong (0, max_distance_m].
  - Dam bao khong trung frame, khong vuot qua max_distance_m.

Usage:
    python3 select_maps_by_distance.py <poses.txt> <calib.txt> <scan_idx> <N> <max_distance_m>

Output: in ra danh sach map_idx (dung truc tiep lam tham so cho mini_removert)
        va bang khoang cach that de kiem chung.
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


def pick_side(scan_pos, positions, frame_indices, n_wanted, max_distance_m):
    """
    positions, frame_indices: danh sach vi tri va index frame ve 1 phia
    (frame_indices[0] la frame GAN scan_idx nhat, tang dan khoang cach).
    Chon n_wanted frame co khoang cach that gan nhat voi cac muc tieu
    cach deu trong (0, max_distance_m].
    """
    dists = np.array([np.linalg.norm(p - scan_pos) for p in positions])
    valid = dists <= max_distance_m
    if valid.sum() < n_wanted:
        raise RuntimeError(
            f"Chi co {valid.sum()} frame trong ban kinh {max_distance_m}m, "
            f"can {n_wanted}. Hay tang max_distance_m hoac giam N."
        )

    targets = np.linspace(max_distance_m / n_wanted, max_distance_m, n_wanted)
    chosen = []
    used = set()
    for target in targets:
        candidate_order = np.argsort(np.abs(dists - target))
        for ci in candidate_order:
            if ci in used or not valid[ci]:
                continue
            used.add(ci)
            chosen.append(ci)
            break
    chosen.sort(key=lambda ci: dists[ci])
    return [(frame_indices[ci], dists[ci]) for ci in chosen]


def main():
    if len(sys.argv) != 6:
        print(f"Usage: {sys.argv[0]} <poses.txt> <calib.txt> <scan_idx> <N> <max_distance_m>")
        sys.exit(1)

    poses_path = sys.argv[1]
    calib_path = sys.argv[2]
    scan_idx = int(sys.argv[3])
    N = int(sys.argv[4])
    max_distance_m = float(sys.argv[5])

    if N % 2 != 0:
        print("N phai la so chan (chia deu 2 phia truoc/sau).")
        sys.exit(1)

    poses = load_poses(poses_path)
    Tr = load_tr(calib_path)

    def lidar_position(idx):
        world_lidar = poses[idx] @ Tr
        return world_lidar[:3, 3]

    n_frames = len(poses)
    scan_pos = lidar_position(scan_idx)

    before_indices = list(range(scan_idx - 1, -1, -1))
    after_indices = list(range(scan_idx + 1, n_frames))

    before_positions = [lidar_position(i) for i in before_indices]
    after_positions = [lidar_position(i) for i in after_indices]

    n_each_side = N // 2
    chosen_before = pick_side(scan_pos, before_positions, before_indices, n_each_side, max_distance_m)
    chosen_after = pick_side(scan_pos, after_positions, after_indices, n_each_side, max_distance_m)

    all_chosen = sorted(chosen_before + chosen_after, key=lambda x: x[0])

    print(f"scan_idx={scan_idx}  N={N}  max_distance_m={max_distance_m}")
    print("")
    print(f"{'map_idx':<10}{'khoang_cach_that(m)':<20}")
    for idx, dist in all_chosen:
        print(f"{idx:<10}{dist:<20.2f}")

    print("")
    map_idx_str = " ".join(str(idx) for idx, _ in all_chosen)
    print("map_idx (dung truc tiep):")
    print(map_idx_str)


if __name__ == "__main__":
    main()