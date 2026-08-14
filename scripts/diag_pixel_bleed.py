#!/usr/bin/env python3
"""
Đo "LÂY NHIỄM THEO PIXEL" — đề bài D, mục 12 HANDOFF_2026-08-14 (gốc: mục 6.1 phiên 13/8).

Cơ chế bị nghi: range image là phép rút gọn NHIỀU→1 (`min`), nhưng gán nhãn là phép tra
ngược 1→NHIỀU. `filter.cpp:22-41` chiếu lại MỌI điểm rồi phát phán quyết của pixel cho
tất cả điểm rơi vào đó. Pixel chứa xe 25m + tường 30.2m + tường 31.0m -> cả 3 bị gán
dynamic dù chỉ điểm 25m tạo ra discrepancy -> 1 TP + 2 FP.

Script chạy HAI phần độc lập:

  [D1] Đếm ăn ké, ở tầng PIXEL, thang REMOVE (64x900).
       Với mỗi pixel có disc > 0: bao nhiêu điểm rơi vào, bao nhiêu là "chủ của min",
       và trong số ăn ké thì bao nhiêu là GT dynamic thật. Con số quyết định: nếu ăn ké
       gần như toàn bộ là GT static -> vá được sẽ tăng precision mà mất ít recall.

  [D2] Mô phỏng luôn bản đã vá, sweep `eps`:
           flag điểm i  <=>  disc(pixel_i) > 0  VÀ  |dist_i - scan_img[pixel_i]| <= eps
       Hai biến thể: `eps@L0` (chỉ ở thang REMOVE) và `eps@all` (mọi thang).
       `eps = inf` = hành vi hiện tại, dùng làm đối chứng có cặp.

QUAN TRỌNG — mô phỏng KHÔNG khớp C++ về con số tuyệt đối (mục 7 + 11 phiên 13/8: ground
mask khác nhau làm F1 lệch tới 0.12, và F1 hỗn loạn theo mask). Vì vậy:
  - CHỈ đọc chênh lệch giữa các luật trong CÙNG một lần chạy (chung ground mask, chung
    mọi thứ khác) — đó là so sánh có cặp, tin được.
  - ĐỪNG trích con số F1 tuyệt đối ra ngoài.
Vì lý do đó script chạy trên nhiều ground mask (`--seeds`) và báo cáo cả độ lệch.

Usage:
    python3 scripts/diag_pixel_bleed.py                 # seq 04 06, 3 ground mask
    python3 scripts/diag_pixel_bleed.py 04 06 07 --seeds 5
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import diag_sign as ds                      # noqa: E402  (loader + ransac + projection)
from select_maps_by_distance import pick_side  # noqa: E402
from run_generalization_test import pick_scans  # noqa: E402

DATA = os.path.expanduser('~/kitti_data/dataset')

# Cấu hình ĐANG CHỐT trong main.cpp — phải khớp, không dùng mặc định cũ của diag_sign.py
THRESHOLD = 1.0
VOTE_THRESHOLD = 0.5
LEVELS = [(64, 900), (32, 450), (16, 225), (8, 112)]
N_MAPS = 4
MAX_DIST_M = 4.0

EPS_LIST = [0.0, 0.1, 0.3, 0.5, 1.0]        # inf = luật hiện tại, xử lý riêng


def select_maps(poses, Tr, scan_idx):
    """Gọi lại đúng logic của select_maps_by_distance.py (import, không subprocess)."""
    pos = lambda i: (poses[i] @ Tr)[:3, 3]
    scan_pos = pos(scan_idx)
    before = list(range(scan_idx - 1, -1, -1))
    after = list(range(scan_idx + 1, len(poses)))
    half = N_MAPS // 2
    chosen = (pick_side(scan_pos, [pos(i) for i in before], before, half, MAX_DIST_M)
              + pick_side(scan_pos, [pos(i) for i in after], after, half, MAX_DIST_M))
    return sorted(idx for idx, _ in chosen)


def decide(vote_by_level, obs, ground_mask):
    """Bản sao tầng quyết định main.cpp:164-210 với 4 thang.
    `vote_by_level[li]` cho phép mỗi thang dùng một luật vote khác nhau (để so eps@L0)."""
    def is_dyn(li):
        v, o = vote_by_level[li], obs[li]
        return (o > 0) & (v / np.maximum(o, 1) > VOTE_THRESHOLD)

    keep = ~ground_mask & is_dyn(0)
    for li in range(1, len(LEVELS)):
        keep &= is_dyn(li)
    return keep


def run_scan(seq, scan_idx, seed):
    """Một scan, một ground mask. Trả về dict thống kê D1 + tập dự đoán của mọi luật."""
    ds.SEQ_DIR = f'{DATA}/sequences/{seq}'      # diag_sign hard-code seq04, ghi đè
    ds.POSES_PATH = f'{DATA}/poses/{seq}.txt'
    poses = ds.load_poses(ds.POSES_PATH)
    Tr = ds.load_tr(f'{ds.SEQ_DIR}/calib.txt')
    map_indices = select_maps(poses, Tr, scan_idx)

    scan_xyz = ds.load_xyz(scan_idx)
    n = len(scan_xyz)
    scan_pose = poses[scan_idx] @ Tr
    scan_ground = ds.ransac_ground(scan_xyz, seed=seed)
    gt = np.isin(ds.load_labels(scan_idx), list(ds.MOVING_CLASSES))

    n_lv = len(LEVELS)
    obs = [np.zeros(n, dtype=np.int32) for _ in range(n_lv)]
    vote_base = [np.zeros(n, dtype=np.int32) for _ in range(n_lv)]
    # vote_eps[e][li] = như vote_base nhưng chỉ tính điểm là "chủ của min" trong eps
    vote_eps = [[np.zeros(n, dtype=np.int32) for _ in range(n_lv)] for _ in EPS_LIST]
    # vote_pp[li] = luật THEO TỪNG ĐIỂM: so range của chính điểm với map, không đi qua
    # scan_img. Xem mục 13 HANDOFF_2026-08-14 — luật này thay thế cả eps.
    vote_pp = [np.zeros(n, dtype=np.int32) for _ in range(n_lv)]
    vote_and = [np.zeros(n, dtype=np.int32) for _ in range(n_lv)]

    # --- thống kê D1, cộng dồn qua các map, chỉ ở thang REMOVE ---
    d1 = dict(pix=0, pts=0, owner=0, free=0, free_gt=0, owner_gt=0)

    for m in map_indices:
        map_xyz = ds.load_xyz(m)
        T_rel = np.linalg.inv(poses[m] @ Tr) @ scan_pose
        scan_in_map = ds.transform(scan_xyz, T_rel)
        map_ground = ds.ransac_ground(map_xyz, seed=seed)   # main.cpp:102 fit riêng mỗi map
        # range của từng điểm scan, trong hệ map — không phụ thuộc thang, tính 1 lần
        dist = np.linalg.norm(scan_in_map, axis=1)

        for li, (h, w) in enumerate(LEVELS):
            scan_img, row, col, valid = ds.build_range_image(scan_in_map, h, w, scan_ground)
            map_img, _, _, _ = ds.build_range_image(map_xyz, h, w, map_ground)

            px = row * w + col
            s = np.where(valid, scan_img[px], np.inf)
            mp = np.where(valid, map_img[px], np.inf)
            both = np.isfinite(s) & np.isfinite(mp)
            with np.errstate(invalid='ignore'):     # inf-inf ở pixel trống, đã lọc bằng `both`
                diff = np.where(both, mp - s, 0.0)

            dyn = both & (np.abs(diff) > THRESHOLD)     # giữ fabs() — mục 10.6
            obs[li] += valid & np.isfinite(mp)
            vote_base[li] += dyn

            # khoảng cách từ range của CHÍNH điểm tới range của pixel (= min).
            # 0 <=> điểm i là chủ của min. Điểm ground bị loại khỏi ảnh nên gap có thể
            # âm -> dùng abs, và dù sao main.cpp:192 đã ép ground = static.
            own_gap = np.where(both, np.abs(dist - s), np.inf)
            for ei, eps in enumerate(EPS_LIST):
                vote_eps[ei][li] += dyn & (own_gap <= eps)

            # LUẬT THEO TỪNG ĐIỂM: dùng range của CHÍNH điểm i, không dùng min của pixel.
            # Điểm sau xe (26m) vẫn vượt ngưỡng so với map (30.2m) -> giữ được recall;
            # điểm tường (30.2m) trùng map -> hết FP. Đây là chỗ luật eps thất bại.
            gate = valid & np.isfinite(mp)
            dyn_pp = gate & (np.abs(mp - dist) > THRESHOLD)
            vote_pp[li] += dyn_pp
            vote_and[li] += dyn & dyn_pp

            if li == 0:
                # đếm ở tầng PIXEL: pixel nào có disc>0, bao nhiêu điểm rơi vào
                in_flagged = dyn & ~scan_ground
                is_owner = in_flagged & (own_gap == 0.0)
                is_free = in_flagged & (own_gap > 0.0)
                d1['pix'] += len(np.unique(px[in_flagged]))
                d1['pts'] += int(in_flagged.sum())
                d1['owner'] += int(is_owner.sum())
                d1['free'] += int(is_free.sum())
                d1['free_gt'] += int((is_free & gt).sum())
                d1['owner_gt'] += int((is_owner & gt).sum())

    # --- các luật, chấm trên CÙNG ground mask / CÙNG map ---
    preds = {'inf (hiện tại)': decide(vote_base, obs, scan_ground)}
    for ei, eps in enumerate(EPS_LIST):
        preds[f'eps={eps} @L0'] = decide(
            [vote_eps[ei][0]] + vote_base[1:], obs, scan_ground)
        preds[f'eps={eps} @all'] = decide(vote_eps[ei], obs, scan_ground)
    preds['per-point @L0'] = decide([vote_pp[0]] + vote_base[1:], obs, scan_ground)
    preds['per-point @all'] = decide(vote_pp, obs, scan_ground)
    preds['pixel&pp @all'] = decide(vote_and, obs, scan_ground)

    return d1, {k: ds.prf(v, gt) for k, v in preds.items()}


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    n_seeds = 3
    if '--seeds' in sys.argv:
        n_seeds = int(sys.argv[sys.argv.index('--seeds') + 1])
    seqs = [a for a in args if a.isdigit() and len(a) == 2] or ['04', '06']

    print(f'threshold={THRESHOLD} vote={VOTE_THRESHOLD} N={N_MAPS} max_dist={MAX_DIST_M}m')
    print(f'{len(LEVELS)} thang: ' + ', '.join(f'{h}x{w}' for h, w in LEVELS))
    print(f'ground mask: {n_seeds} seed\n')
    print('CANH BAO: mo phong KHONG khop C++ ve con so tuyet doi (muc 7+11 phien 13/8).')
    print('          Chi doc CHENH LECH giua cac luat trong cung mot lan chay.\n')

    agg = {}      # rule -> list of F1 over (scan, seed)
    d1_tot = dict(pix=0, pts=0, owner=0, free=0, free_gt=0, owner_gt=0)

    for seq in seqs:
        scans, _ = pick_scans(seq)
        if not scans:
            print(f'=== seq {seq}: khong du frame hop le, bo qua\n')
            continue
        print(f'=== seq {seq} === scan {[int(s) for s in scans]}')
        for scan in scans:
            for seed in range(n_seeds):
                d1, res = run_scan(seq, int(scan), seed)
                for k in d1_tot:
                    d1_tot[k] += d1[k]
                for rule, (p, r, f, tp, fp, fn) in res.items():
                    agg.setdefault(rule, []).append((f, p, r, tp, fp))
        print('  xong\n')

    # ---------------- [D1] ----------------
    print('=' * 72)
    print('[D1] AN KE O TANG PIXEL (thang REMOVE 64x900, cong don moi scan x map x seed)')
    print('=' * 72)
    pts, own, free = d1_tot['pts'], d1_tot['owner'], d1_tot['free']
    print(f'  pixel bi flag disc>0            : {d1_tot["pix"]:>10,}')
    print(f'  diem (khong-ground) roi vao do  : {pts:>10,}   '
          f'= {pts / max(d1_tot["pix"], 1):.2f} diem/pixel')
    print(f'  trong do LA chu cua min         : {own:>10,}   ({100.0 * own / max(pts, 1):.1f}%)')
    print(f'  trong do AN KE (khong phai min) : {free:>10,}   ({100.0 * free / max(pts, 1):.1f}%)')
    print()
    print(f'  chu cua min  la GT dynamic      : {d1_tot["owner_gt"]:>10,}   '
          f'({100.0 * d1_tot["owner_gt"] / max(own, 1):.1f}% cua nhom)')
    print(f'  AN KE        la GT dynamic      : {d1_tot["free_gt"]:>10,}   '
          f'({100.0 * d1_tot["free_gt"] / max(free, 1):.1f}% cua nhom)')
    print()
    print('  Doc bang: neu nhom AN KE co ty le GT dynamic THAP hon han nhom chu-cua-min')
    print('  thi chan an ke se tang precision. Neu hai ty le xap xi nhau thi de bai D')
    print('  khong dang lam — an ke va tin hieu that nam lan trong nhau.')

    # ---------------- [D2] ----------------
    print()
    print('=' * 72)
    print(f'[D2] SWEEP eps — trung binh tren {len(next(iter(agg.values())))} (scan x seed)')
    print('=' * 72)
    print(f'  {"luat":<18}{"F1":>8}{"std":>7}{"P":>8}{"R":>8}{"TP":>8}{"FP":>8}   {"dF1":>7}')
    base_f1 = float(np.mean([x[0] for x in agg['inf (hiện tại)']]))
    order = (['inf (hiện tại)']
             + [f'eps={e} @{v}' for v in ('L0', 'all') for e in EPS_LIST]
             + ['per-point @L0', 'per-point @all', 'pixel&pp @all'])
    for rule in order:
        v = np.array([x[:3] for x in agg[rule]])
        tp = np.mean([x[3] for x in agg[rule]])
        fp = np.mean([x[4] for x in agg[rule]])
        f1 = v[:, 0].mean()
        print(f'  {rule:<18}{f1:>8.3f}{v[:, 0].std():>7.3f}{v[:, 1].mean():>8.3f}'
              f'{v[:, 2].mean():>8.3f}{tp:>8.1f}{fp:>8.1f}   {f1 - base_f1:>+7.3f}')
    print()
    print('  Cot std la do lech theo (scan x ground mask) — mot dF1 nho hon std cua chinh')
    print('  no thi KHONG ket luan duoc (quy tac muc 9 phien 13/8).')


if __name__ == '__main__':
    main()
