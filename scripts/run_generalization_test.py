#!/usr/bin/env python3
"""
Viec 4 muc 7 phien 12/8: test tren sequence KHAC seq04.

GIAO THUC CHON SCAN - chot TRUOC khi chay, de khong cherry-pick:
  1. Chi xet frame co > 0 diem GT dynamic. Day la loc TINH HOP LE, khong phai loc
     hieu nang: khong co vat dong that thi recall khong co mau so, F1 vo nghia.
     KHONG dung ket qua thuat toan de chon - chi dung nhan.
  2. Trong danh sach do, lay 5 vi tri CACH DEU (20/35/50/65/80 %).
  3. Bo frame qua gan dau/cuoi sequence (khong du map hai phia trong ban kinh).
  4. Bao cao TAT CA, ke ca sequence ra ket qua xau.

Usage:
    python3 scripts/run_generalization_test.py [seq ...]     (mac dinh: 03 04 06 07)
"""
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

REPO = '/home/atun/mini_removert'
D = os.path.expanduser('~/kitti_data/dataset')
BINARY = f'{REPO}/build/mini_removert/mini_removert'
THRESHOLD = '1.0'      # moc chot 2026-08-13 (HANDOFF muc 12)
N_MAPS = '4'
MAX_DIST = '4.0'
MOVING = list(range(252, 260))
PICK_AT = [0.20, 0.35, 0.50, 0.65, 0.80]
EDGE_SKIP = 10         # bo 10 frame dau/cuoi


def gt_dynamic_counts(seq):
    lab_dir = f'{D}/sequences/{seq}/labels'
    files = sorted(os.listdir(lab_dir))
    out = []
    for i, f in enumerate(files):
        lab = np.fromfile(f'{lab_dir}/{f}', dtype=np.uint32) & 0xFFFF
        out.append(int(np.isin(lab, MOVING).sum()))
    return np.array(out)


def pick_scans(seq):
    cnt = gt_dynamic_counts(seq)
    n = len(cnt)
    ok = np.flatnonzero((cnt > 0))
    ok = ok[(ok >= EDGE_SKIP) & (ok < n - EDGE_SKIP)]
    if len(ok) < len(PICK_AT):
        return [], cnt
    idx = [ok[int(p * (len(ok) - 1))] for p in PICK_AT]
    return idx, cnt


def run_one(seq, scan):
    poses = f'{D}/poses/{seq}.txt'
    calib = f'{D}/sequences/{seq}/calib.txt'
    sel = subprocess.run(['python3', f'{REPO}/scripts/select_maps_by_distance.py',
                          poses, calib, str(scan), N_MAPS, MAX_DIST],
                         capture_output=True, text=True)
    if sel.returncode != 0:
        return None, 'khong chon duoc map'
    maps = sel.stdout.strip().split('\n')[-1].split()

    work = tempfile.mkdtemp()
    r = subprocess.run([BINARY, f'{D}/sequences/{seq}/pcd', poses, calib,
                        str(scan), THRESHOLD] + maps,
                       cwd=work, capture_output=True, text=True)
    if r.returncode != 0:
        return None, 'binary loi'
    ev = subprocess.run(['python3', f'{REPO}/scripts/evaluate.py',
                         f'{D}/sequences/{seq}/labels/{scan:06d}.label',
                         f'{work}/dynamic_indices_scan{scan}.txt'],
                        capture_output=True, text=True).stdout
    g = lambda k: float(re.search(rf'{k}:\s+([\d.]+)', ev).group(1))
    return (g('Precision'), g('Recall'), g('F1 score'),
            int(g('GT dynamic \\(thật\\)')), int(g('Predicted dynamic'))), ' '.join(maps)


def main():
    seqs = sys.argv[1:] or ['03', '04', '06', '07']
    print(f'threshold={THRESHOLD}  N={N_MAPS}  max_distance_m={MAX_DIST}\n')
    summary = {}
    for seq in seqs:
        scans, cnt = pick_scans(seq)
        frac = 100.0 * (cnt > 0).mean()
        print(f'=== seq {seq} === ({len(cnt)} frame, {frac:.1f}% frame co vat dong)')
        if not scans:
            print('  khong du frame hop le, bo qua\n')
            continue
        print(f"  {'scan':>6}{'GT dyn':>9}{'du doan':>10}{'P':>8}{'R':>8}{'F1':>8}")
        fs = []
        for s in scans:
            res, info = run_one(seq, int(s))
            if res is None:
                print(f'  {s:>6}   {info}')
                continue
            p, r, f, gt, pred = res
            fs.append(f)
            print(f'  {s:>6}{gt:>9}{pred:>10}{p:>8.3f}{r:>8.3f}{f:>8.3f}')
        if fs:
            summary[seq] = float(np.mean(fs))
            print(f"  {'':>6}{'':>9}{'':>10}{'':>8}{'TB F1:':>8}{np.mean(fs):>8.3f}\n")
    print('=== TONG KET ===')
    for seq, f in summary.items():
        print(f'  seq {seq}: F1 TB = {f:.3f}')


if __name__ == '__main__':
    main()
