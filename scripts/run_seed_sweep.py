#!/usr/bin/env python3
"""
run_seed_sweep.py — chạy CÙNG một cấu hình trên NHIỀU ground mask (`--ground-seed`)
rồi báo cáo F1 trung bình ± độ lệch.

Đây là thứ đề bài B tồn tại để làm được (mục 11 + 12 phiên 14/8): RANSAC trong C++ tiền
định, nên trước khi có `--ground-seed` thì binary chỉ nhìn thấy ĐÚNG MỘT mask trong cả họ
mask hợp lệ, và quy tắc "đo trên nhiều ground mask" (mục 9 phiên 13/8) không thực thi được.

Dùng nó cho MỌI thay đổi đụng tới ground từ giờ trở đi: một chênh lệch F1 nhỏ hơn độ lệch
theo mask thì KHÔNG kết luận được.

Usage:
    python3 scripts/run_seed_sweep.py                          # 5 scan moc, seed 0-5, 4 thang
    python3 scripts/run_seed_sweep.py --seeds 0                # chi seed 0 (moc cu)
    python3 scripts/run_seed_sweep.py --levels 64x900,32x450,16x225
    python3 scripts/run_seed_sweep.py --seq 06 --scans 349,939
"""
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = os.path.expanduser('~/kitti_data/dataset')
BINARY = f'{REPO}/build/mini_removert/mini_removert'

THRESHOLD = '1.0'                       # mốc chốt 13/8
N_MAPS = '4'
MAX_DIST = '4.0'
DEFAULT_SCANS = [50, 100, 150, 200, 250]        # ĐÚNG bộ 5 scan của mốc 0.720/0.691
DEFAULT_SEEDS = [0, 1, 2, 3, 4, 5]


def select_maps(seq, scan):
    """Gọi select_maps_by_distance.py; map_idx nằm ở DÒNG CUỐI stdout."""
    sel = subprocess.run(
        ['python3', f'{REPO}/scripts/select_maps_by_distance.py',
         f'{D}/poses/{seq}.txt', f'{D}/sequences/{seq}/calib.txt',
         str(scan), N_MAPS, MAX_DIST],
        capture_output=True, text=True)
    if sel.returncode != 0:
        return None
    return sel.stdout.strip().split('\n')[-1].split()


def run_one(seq, scan, maps, seed, levels):
    """Một lần chạy binary + evaluate.py. Trả (P, R, F1) hoặc None."""
    args = [BINARY, f'{D}/sequences/{seq}/pcd', f'{D}/poses/{seq}.txt',
            f'{D}/sequences/{seq}/calib.txt', str(scan), THRESHOLD] + maps
    if seed != 0:
        args += ['--ground-seed', str(seed)]
    if levels:
        args += ['--levels', levels]

    work = tempfile.mkdtemp()           # binary ghi ra CWD -> mỗi lần một thư mục riêng
    r = subprocess.run(args, cwd=work, capture_output=True, text=True)
    if r.returncode != 0:
        return None
    ev = subprocess.run(
        ['python3', f'{REPO}/scripts/evaluate.py',
         f'{D}/sequences/{seq}/labels/{scan:06d}.label',
         f'{work}/dynamic_indices_scan{scan}.txt'],
        capture_output=True, text=True).stdout
    g = lambda k: float(re.search(rf'{k}:\s+([\d.]+)', ev).group(1))
    return g('Precision'), g('Recall'), g('F1 score')


def arg(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


def main():
    seq = arg('--seq', '04')
    scans = [int(x) for x in arg('--scans', ','.join(map(str, DEFAULT_SCANS))).split(',')]
    seeds = [int(x) for x in arg('--seeds', ','.join(map(str, DEFAULT_SEEDS))).split(',')]
    levels = arg('--levels', '')

    print(f'seq {seq} | scan {scans} | threshold={THRESHOLD} N={N_MAPS} max_dist={MAX_DIST}')
    print(f'levels: {levels if levels else "mac dinh (4 thang 64x900,32x450,16x225,8x112)"}')
    print(f'ground seed: {seeds}\n')

    maps = {s: select_maps(seq, s) for s in scans}
    for s, m in maps.items():
        if m is None:
            print(f'  scan {s}: khong chon duoc map, bo qua')

    print(f'  {"scan":>6}' + ''.join(f'{f"seed {sd}":>10}' for sd in seeds))
    f1 = np.full((len(scans), len(seeds)), np.nan)
    pr = np.full((len(scans), len(seeds), 2), np.nan)
    for i, s in enumerate(scans):
        if maps[s] is None:
            continue
        row = ''
        for j, sd in enumerate(seeds):
            res = run_one(seq, s, maps[s], sd, levels)
            if res is None:
                row += f'{"loi":>10}'
                continue
            pr[i, j], f1[i, j] = res[:2], res[2]
            row += f'{res[2]:>10.3f}'
        print(f'  {s:>6}' + row)

    print(f'  {"TB":>6}' + ''.join(f'{np.nanmean(f1[:, j]):>10.3f}' for j in range(len(seeds))))
    print()
    per_seed = np.nanmean(f1, axis=0)            # F1 TB của từng mask
    print(f'  F1 TB tren {len(scans)} scan, tinh cho tung ground mask:')
    print(f'    trung binh cua cac mask : {np.nanmean(per_seed):.3f}')
    print(f'    do lech chuan giua mask : {np.nanstd(per_seed):.3f}')
    print(f'    min / max              : {np.nanmin(per_seed):.3f} / {np.nanmax(per_seed):.3f}'
          f'   (bien do {np.nanmax(per_seed) - np.nanmin(per_seed):.3f})')
    print(f'    P / R trung binh       : {np.nanmean(pr[..., 0]):.3f} / {np.nanmean(pr[..., 1]):.3f}')
    print()
    print('  Doc bang: moi thay doi code cho dF1 NHO HON bien do tren thi KHONG ket luan duoc.')


if __name__ == '__main__':
    main()
