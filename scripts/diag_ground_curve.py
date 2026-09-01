#!/usr/bin/env python3
"""
diag_ground_curve.py — mặt đường có CONG ra khỏi mặt phẳng ground không?

Trả lời câu hỏi mở của phiên 01/09 (`START_HERE.md`, mục 12.3 HANDOFF_2026-09-01):
cùng một mặt phẳng nhưng scan ra 49.8% ground còn map tích luỹ chỉ 38.3%. Vì sao?

BA giả thuyết cạnh tranh, script này tách được cả ba:

  (A) MẶT ĐƯỜNG CONG. Mặt phẳng fit tại chỗ đứng của scan; map trải cả trăm mét nên
      mặt đường cong ra khỏi nó. Độ lệch của cung tròn bán kính R ở khoảng cách r là
      r²/(2R) — một PARABOL.
  (B) VOXEL ĐỔI THÀNH PHẦN. Mặt đường phẳng và dày, 9 lớp điểm gần trùng nhau nên
      voxel 0.2m nén rất mạnh; tường/cây thưa trong không gian 3D nên nén ít. Tỷ lệ
      ground tụt vì MẪU SỐ đổi, mặt đường chẳng cong đi đâu.
  (C) THÀNH PHẦN THEO KHOẢNG CÁCH. Vành đai xa có nhiều nhà/cây hơn và mặt đường bị
      che nhiều hơn. Tỷ lệ ground tụt theo r kể cả khi đường phẳng tuyệt đối.

VÌ SAO KHÔNG ĐO "TỶ LỆ GROUND": cả ba giả thuyết đều làm tỷ lệ tụt, nên tỷ lệ KHÔNG
phân biệt được chúng. Script này đo **ĐỈNH histogram khoảng cách CÓ DẤU tới mặt phẳng**.
Đổi thành phần làm số điểm đổi, nhưng KHÔNG thể đẩy đỉnh của cụm điểm-mặt-đường đi chỗ
khác. Đường phẳng ⇒ đỉnh đứng yên ở 0 ở mọi vành đai. Đường cong ⇒ đỉnh trôi theo r².

ĐỌC KẾT QUẢ:
  cột `dinh` đứng yên quanh 0 mọi vành đai      -> (A) SAI. Nhìn sang (B)/(C).
  cột `dinh` trôi dần, khớp r²/(2R)             -> (A) ĐÚNG, và R in ra là bán kính cong.
  `ground%` của accumulated ≈ của map_cloud     -> (B) SAI, voxel không phải thủ phạm.
  `ground%` tụt mạnh sau voxel                  -> (B) ĐÚNG.
  `ground%` tụt theo r nhưng `dinh` đứng yên    -> (C), tức chỉ là thành phần cảnh vật.

Usage:
    python3 scripts/diag_ground_curve.py 04 150
    python3 scripts/diag_ground_curve.py 06 939 --ring 5      # vành đai 5m thay vì 10m

KHÔNG đụng C++. Đúng luật "đo trước, sửa sau" của CLAUDE.md.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import src_quality as sq                    # noqa: E402  fit_plane_ransac, voxel_downsample

DATA = os.path.expanduser('~/kitti_data/dataset')
GROUND_THR = 0.2                            # ground_filter.cpp, phải khớp
RUNGS = [(5, 5), (10, 10), (20, 10)]        # mặc định của select_source_ladder.py
MIN_SRC = 8


def load_poses(seq):
    p = np.loadtxt(f'{DATA}/poses/{seq}.txt').reshape(-1, 3, 4)
    out = np.tile(np.eye(4), (len(p), 1, 1))
    out[:, :3, :] = p
    return out


def load_tr(seq):
    for line in open(f'{DATA}/sequences/{seq}/calib.txt'):
        if line.startswith('Tr:'):
            T = np.eye(4)
            T[:3, :] = np.array(line.split()[1:], dtype=float).reshape(3, 4)
            return T
    raise RuntimeError('khong thay dong Tr: trong calib.txt')


def pick_sources(poses, Tr, scan_idx):
    """Thang bậc y hệt select_source_ladder.py: bậc cuối chỉ đòi >= 1 (lưới an toàn)."""
    pos = np.array([(p @ Tr)[:3, 3] for p in poses])
    dist = np.linalg.norm(pos - pos[scan_idx], axis=1)
    idx = np.arange(len(poses))
    for k, (R, t_min) in enumerate(RUNGS):
        ok = (dist <= R) & (np.abs(idx - scan_idx) >= t_min) & (idx != scan_idx)
        need = 1 if k == len(RUNGS) - 1 else MIN_SRC
        if int(ok.sum()) >= need:
            return idx[ok].tolist(), k + 1, R, t_min
    raise RuntimeError('khong bac nao dat')


def peak_of(sd, lo=-2.0, hi=2.0, bin_w=0.02):
    """Đỉnh histogram khoảng cách có dấu — vị trí cụm điểm-mặt-đường.
    Hẹp cửa sổ về [-2,2]m để tường/mái nhà ở xa không kéo đỉnh đi."""
    sel = sd[(sd >= lo) & (sd <= hi)]
    if len(sel) < 50:
        return None
    hist, edges = np.histogram(sel, bins=int((hi - lo) / bin_w), range=(lo, hi))
    i = int(np.argmax(hist))
    return float((edges[i] + edges[i + 1]) / 2)


def report(name, xyz, nrm, d, ring_m, max_r):
    sd = xyz @ nrm + d                       # khoảng cách CÓ DẤU tới mặt phẳng
    r = np.linalg.norm(xyz, axis=1)          # scan ở gốc toạ độ nên đây là bán kính
    print(f'\n--- {name}  ({len(xyz):,} diem) ---')
    print(f'{"vanh dai":>12} | {"#diem":>10} | {"ground%":>8} | {"dinh (m)":>9} | {"trung vi":>9}')
    print('-' * 60)
    rows = []
    for lo in np.arange(0, max_r, ring_m):
        m = (r >= lo) & (r < lo + ring_m)
        n = int(m.sum())
        if n < 200:
            continue
        sdm = sd[m]
        g = float((np.abs(sdm) <= GROUND_THR).mean() * 100)
        pk = peak_of(sdm)
        near = sdm[np.abs(sdm) <= 1.0]
        med = float(np.median(near)) if len(near) else float('nan')
        pk_s = f'{pk:+.3f}' if pk is not None else '   --'
        print(f'{lo:5.0f}-{lo+ring_m:<6.0f} | {n:10,} | {g:7.1f}% | {pk_s:>9} | {med:+8.3f}')
        if pk is not None:
            rows.append((lo + ring_m / 2, pk))
    return rows


def fit_curvature(rows):
    """Khớp dinh ≈ a·r² rồi suy ra ban kinh cong R = 1/(2a). Bỏ vành đai đầu vì mặt
    phẳng được fit chủ yếu ở đó nên đỉnh bị ép về 0 theo cấu tạo."""
    if len(rows) < 3:
        return None
    r = np.array([x for x, _ in rows[1:]])
    p = np.array([y for _, y in rows[1:]])
    a = float((r ** 2 @ p) / (r ** 2 @ r ** 2))     # bình phương tối thiểu qua gốc
    resid = p - a * r ** 2
    ss = 1 - (resid @ resid) / max((p - p.mean()) @ (p - p.mean()), 1e-12)
    return a, (1 / (2 * a) if abs(a) > 1e-12 else float('inf')), ss


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    ring_m = 5.0 if '--ring' in sys.argv else 10.0
    if len(args) < 2:
        print(__doc__)
        sys.exit(1)
    seq, scan_idx = args[0], int(args[1])

    poses, Tr = load_poses(seq), load_tr(seq)
    src, rung, R, t_min = pick_sources(poses, Tr, scan_idx)
    print(f'seq{seq} / scan {scan_idx}: bac {rung} (R={R}, t_min={t_min}), {len(src)} nguon')

    scan = sq.load_xyz(seq, scan_idx)
    inv_scan = np.linalg.inv(poses[scan_idx] @ Tr)
    import diag_sign as ds
    acc = np.concatenate(
        [ds.transform(sq.load_xyz(seq, j), inv_scan @ (poses[j] @ Tr)) for j in src], axis=0)
    vox = sq.voxel_downsample(acc)
    print(f'gop: {len(acc):,} diem -> sau voxel {sq.VOXEL}m: {len(vox):,} '
          f'({100.0*len(vox)/len(acc):.2f}%)')

    # Mặt phẳng fit MỘT LẦN trên scan — y như main.cpp sau việc 3.
    nrm, d = sq.fit_plane_ransac(scan, seed=0)
    print(f'mat phang tu scan: n=({nrm[0]:+.4f}, {nrm[1]:+.4f}, {nrm[2]:+.4f}) d={d:+.4f}')

    max_r = float(np.percentile(np.linalg.norm(vox, axis=1), 99))
    report('SCAN', scan, nrm, d, ring_m, max_r)
    report('ACCUMULATED (truoc voxel)', acc, nrm, d, ring_m, max_r)
    rows = report('MAP_CLOUD (sau voxel)', vox, nrm, d, ring_m, max_r)

    print('\n=== KET LUAN ===')
    fc = fit_curvature(rows)
    if fc is None:
        print('Khong du vanh dai de khop.')
        return
    a, R_fit, ss = fc
    print(f'Khop dinh ~ a*r^2 tren MAP_CLOUD:  a = {a:+.6f},  R = {R_fit:,.0f} m,  R^2 = {ss:.3f}')
    drift = abs(rows[-1][1] - rows[0][1])
    print(f'Do troi cua dinh tu vanh dai dau den cuoi: {drift:.3f} m (nguong ground = {GROUND_THR} m)')
    if drift < GROUND_THR / 2 or ss < 0.5:
        print('=> Dinh gan nhu DUNG YEN. Gia thuyet (A) mat duong cong: KHONG duoc ung ho.')
        print('   Doc lai cot ground% cua ACCUMULATED vs MAP_CLOUD de xet (B) va (C).')
    else:
        print(f'=> Dinh TROI theo r^2. Gia thuyet (A) duoc ung ho, ban kinh cong ~ {R_fit:,.0f} m.')
        print(f'   Du bao vach do ground% roi o r* = sqrt(2*{GROUND_THR}*R) = '
              f'{np.sqrt(2*GROUND_THR*R_fit):.1f} m — doi chieu voi cot ground%.')


if __name__ == '__main__':
    main()
