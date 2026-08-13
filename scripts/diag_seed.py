#!/usr/bin/env python3
"""
diag_seed.py — do do NHAY CAM cua ket luan "ANTI tot hon ABS" theo ground mask.

Van de (HANDOFF_2026-08-13 muc 7): mo phong Python chua khop C++ vi RANSAC hai ben
fit ra mat phang KHAC nhau (F1 0.746 vs 0.623 o scan 150). Cau hoi: chenh lech
giua 2 luat co phu thuoc ground mask khong?

Cach do: chay lai toan bo voi nhieu seed RANSAC khac nhau. Trong MOI lan chay,
ca 2 luat dung CHUNG mot mask -> so sanh la co cap (paired). Neu F1 tuyet doi
nhay theo seed nhung DELTA giua 2 luat dung yen => ket luan khong phu thuoc
ground mask, khong can sua C++ de tin no.

Usage:
    python3 scripts/diag_seed.py [so_seed]        (mac dinh 5 seed)
"""
import sys
import numpy as np
import diag_sign as d
import diag_anti as a

CASES = [(50, [48, 49, 51, 52]), (100, [98, 99, 101, 102]), (150, [148, 149, 151, 152]),
         (200, [198, 199, 201, 202]), (250, [248, 249, 251, 252])]
BASE = 'ABS (mo phong code hien tai)'
CAND = 'ABS @L0 + ANTI revert'


def main():
    n_seed = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    print(f"So sanh '{BASE}' vs '{CAND}' tren {len(CASES)} scan, {n_seed} seed RANSAC\n")
    print(f"{'seed':>5}{'ground%':>10}{'F1 ABS':>10}{'F1 ANTI':>10}{'delta':>9}")

    f1_base, f1_cand, deltas = [], [], []
    per_scan_delta = {s: [] for s, _ in CASES}
    for seed in range(n_seed):
        fb, fc, gpct = [], [], []
        for scan, maps in CASES:
            out, gt = a.run(scan, maps, seed=seed)
            b = d.prf(out[BASE], gt)[2]
            c = d.prf(out[CAND], gt)[2]
            fb.append(b); fc.append(c)
            per_scan_delta[scan].append(c - b)
        mb, mc = float(np.mean(fb)), float(np.mean(fc))
        f1_base.append(mb); f1_cand.append(mc); deltas.append(mc - mb)
        # ty le ground cua scan dau, chi de thay mask co doi theo seed that khong
        gpct = 100.0 * d.ransac_ground(d.load_xyz(CASES[0][0]), seed=seed).mean()
        print(f"{seed:>5}{gpct:>10.2f}{mb:>10.3f}{mc:>10.3f}{mc - mb:>+9.3f}")

    print(f"\n{'':5}{'':10}{'-'*29}")
    print(f"{'TB':>5}{'':10}{np.mean(f1_base):>10.3f}{np.mean(f1_cand):>10.3f}{np.mean(deltas):>+9.3f}")
    print(f"{'do lech chuan':>15}{np.std(f1_base):>15.3f}{np.std(f1_cand):>10.3f}{np.std(deltas):>9.3f}")
    print(f"{'min-max':>15}{np.ptp(f1_base):>15.3f}{np.ptp(f1_cand):>10.3f}{np.ptp(deltas):>9.3f}")

    print('\nDelta theo tung scan (de xem co scan nao ANTI thua khong):')
    for scan, v in per_scan_delta.items():
        v = np.array(v)
        flag = '' if (v > 0).all() else '   <-- KHONG on dinh'
        print(f'  scan {scan}: TB {v.mean():+.3f}  (min {v.min():+.3f}, max {v.max():+.3f}){flag}')

    # Do NHAY tren MOT scan (khong bi trung binh 5 scan lam nhoe) — muc 11 cua bao cao
    scan, maps = CASES[2]   # scan 150, co moc C++ de doi chieu: F1 = 0.623
    print(f'\nDo nhay tren rieng scan {scan} ({n_seed} ground mask khac nhau, cung pipeline):')
    print(f"{'seed':>5}{'ground':>9}{'F1 ABS':>10}{'F1 ANTI':>10}")
    A, B = [], []
    for seed in range(n_seed):
        out, gt = a.run(scan, maps, seed=seed)
        fa = d.prf(out[BASE], gt)[2]
        fc = d.prf(out[CAND], gt)[2]
        A.append(fa); B.append(fc)
        print(f'{seed:>5}{int(d.ransac_ground(d.load_xyz(scan), seed=seed).sum()):>9}{fa:>10.3f}{fc:>10.3f}')
    A, B = np.array(A), np.array(B)
    print(f"{'min-max':>14}{np.ptp(A):>10.3f}{np.ptp(B):>10.3f}")
    print(f"{'lech chuan':>14}{A.std():>10.3f}{B.std():>10.3f}")
    print(f'\n  (moc C++ that tren scan {scan}: F1 = 0.623 — nam LOT trong dai cua ABS)')


if __name__ == '__main__':
    main()
