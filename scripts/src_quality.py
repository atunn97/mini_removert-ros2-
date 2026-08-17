#!/usr/bin/env python3
"""
src_quality.py — CHẤM ĐIỂM BỘ FRAME NGUỒN cho map tích luỹ (đề bài E, mục 12
HANDOFF_2026-08-14; thiết kế và số liệu đối chiếu ở mục 10).

Đây là bản dựng lại của script dùng-một-lần đã mất (mục 10.7). Nhiệm vụ của nó là
trả lời câu hỏi **trước khi** bạn viết dòng C++ nào cho đề bài E:

    với (R, t_min) này, bộ frame nguồn có ĐỦ TỐT để tách động/tĩnh không?

Nếu AUC ở đây đã thấp thì bản C++ dù viết đúng cũng không thể ăn — và ngược lại, nếu
AUC ở đây cao mà C++ ra F1 = 0 thì lỗi nằm ở CÀI ĐẶT, không phải ở ý tưởng. Đó chính
là tiêu chí chấm số 1 của đề bài E (mục 10.7).

Chuỗi phép đo (đúng mục 10.7: gộp nguồn -> voxel 0.2 -> hệ scan -> 1 range image ->
thống kê |Δr| tách theo nhãn GT -> AUC hạng):

  1. Chọn nguồn:  { j : ||pos(j)-pos(scan)|| <= R  và  |j-scan| >= t_min  và  j != scan }
  2. Đưa MỌI frame nguồn về **hệ toạ độ của scan** bằng T = inv(scan_pose) @ pose_j,
     nối thành một cloud duy nhất  (code C++ hiện tại làm NGƯỢC chiều — mục 10 §E2).
  3. VoxelGrid 0.2 m (lấy TRỌNG TÂM mỗi voxel, giống pcl::VoxelGrid).
  4. Fit ground **MỘT LẦN** trên scan, dùng CHUNG hệ số mặt phẳng đó cho map. Vì map
     đã ở hệ scan nên không cần biến đổi mặt phẳng (đề bài C tự tan biến — mục 10 §E2.3).
  5. Dựng MỘT range image cho map, MỘT cho scan, ở thang REMOVE 64x900.
  6. Với mỗi điểm scan không-ground, quan sát được:  Δr = map_img[px] - scan_img[px].
     Tách theo nhãn GT -> nền nhiễu (trung vị trên GT tĩnh), tín hiệu (trên GT động).
  7. AUC hạng (Mann-Whitney) cho hai luật chấm điểm:
        AUC abs      : score = |Δr|      <- luật đang dùng, GIỮ (mục 10.6)
        AUC 1 chiều  : score = Δr        <- map xa hơn scan; ĐÃ BÁC BỎ làm mặc định

Đọc kết quả (ngưỡng ở mục 10.1):
    AUC >= 0.9  : bộ nguồn TỐT, viết C++ được
    AUC ~ 0.5   : vô dụng, KHÔNG ngưỡng nào cứu được — đổi (R, t_min) trước đã
    tỷ lệ tín hiệu/nền nhiễu >= 10 : đạt

Bảng thứ hai ("ho so nguong") là tiêu chí chấm số 2 của đề bài E: recall tại thr = 0.5 /
1 / 2 / 5 m. Recall gần như KHÔNG đổi trên cả dải = **cao nguyên**, tức `threshold` đã hết
là biến tới hạn (mục 10.7). Recall tụt dần = còn **vách đá** như mục 12.1 phiên 13/8.

    KHÔNG in F1 ở đây, và đó là chủ ý: một luật MỘT thang, MỘT ảnh, không voting, không
    REVERT thì F1 bé tí ở mọi ngưỡng (mất cân bằng lớp 160/34.000) — con số đó nói về
    luật rút gọn chứ không nói về bộ nguồn, mà bộ nguồn mới là thứ script này chấm.
    Vì lý do đó % điểm tĩnh bị flag in kèm cũng cao; nó chỉ để so TƯƠNG ĐỐI giữa các
    (R, t_min), đừng đọc như precision của pipeline.

ĐỐI CHIẾU với bảng gốc mục 10.4 (bản dựng lại này tái lập được, sai số nhỏ vì RANSAC
khác seed và voxel lấy trọng tâm): seq06/939 R=5,t=5 → nhiễu 0.18 (gốc 0.19), tín hiệu
22.30 (22.22), AUC 0.980 (0.978); seq04/150 R=20,t=10 → AUC 0.975 (0.974), AUC 1 chiều
0.267 (0.322). Xem `HANDOFF_2026-08-17.md`.

Usage:
    python3 scripts/src_quality.py 06 939                 # lưới (R,t_min) mặc định
    python3 scripts/src_quality.py 06 939 349
    python3 scripts/src_quality.py 04 150 --grid 5:1,10:5,20:10
    python3 scripts/src_quality.py 06 939 --seeds 3       # 3 ground mask, báo cáo ±
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import diag_sign as ds                      # noqa: E402  (loader + projection dùng chung)

DATA = os.path.expanduser('~/kitti_data/dataset')

VOXEL = 0.2                                 # m, mục 10.4
LEVEL = (64, 900)                           # thang REMOVE
GROUND_DIST_THR = 0.2                       # ground_filter.cpp
GROUND_ITERS = 200
# lưới (R, t_min) mặc định — đúng các dòng đã đo ở mục 10.4 để đối chiếu được
DEFAULT_GRID = [(5, 1), (5, 5), (10, 1), (10, 10), (20, 1), (20, 10), (20, 20)]
THR_PROFILE = [0.5, 1.0, 2.0, 5.0]          # m, để xem ngưỡng còn tới hạn hay không

_xyz_cache = {}


def load_xyz(seq, idx):
    """Cache theo (seq, idx): một lưới (R,t_min) dùng lại rất nhiều frame."""
    key = (seq, idx)
    if key not in _xyz_cache:
        ds.SEQ_DIR = f'{DATA}/sequences/{seq}'
        _xyz_cache[key] = ds.load_xyz(idx)
    return _xyz_cache[key]


def fit_plane_ransac(xyz, seed=0):
    """Như ground_filter.cpp nhưng TRẢ VỀ HỆ SỐ (n, d), không trả mask.
    Cần hệ số vì mục 10 §E2.3 dùng CHUNG một mặt phẳng cho cả scan lẫn map."""
    rng = np.random.default_rng(seed)
    n_pts = len(xyz)
    best_inl, best_cnt = None, -1
    for _ in range(GROUND_ITERS):
        i = rng.choice(n_pts, 3, replace=False)
        p0, p1, p2 = xyz[i]
        nrm = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(nrm)
        if nn < 1e-9:
            continue
        nrm = nrm / nn
        d = -nrm @ p0
        inl = np.abs(xyz @ nrm + d) <= GROUND_DIST_THR
        c = int(inl.sum())
        if c > best_cnt:
            best_cnt, best_inl = c, inl
    # setOptimizeCoefficients(true): refit least-squares trên inlier
    pts = xyz[best_inl]
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    nrm = vt[-1]
    return nrm, float(-nrm @ centroid)


def ground_mask_from_plane(xyz, nrm, d):
    return np.abs(xyz @ nrm + d) <= GROUND_DIST_THR


def voxel_downsample(xyz, size=VOXEL):
    """Trọng tâm mỗi voxel, giống pcl::VoxelGrid (KHÔNG phải lấy điểm đại diện).
    Mã hoá key 3 chiều thành một int64 để np.unique chạy nhanh trên vài triệu điểm."""
    k = np.floor(xyz / size).astype(np.int64) + (1 << 20)     # dời về dương
    if k.min() < 0 or k.max() >= (1 << 21):
        raise RuntimeError('toa do vuot khoi dai voxel key (|x| > ~200km?)')
    key = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    out = np.zeros((len(cnt), 3))
    for c in range(3):
        out[:, c] = np.bincount(inv, weights=xyz[:, c]) / cnt
    return out


def select_sources(poses, Tr, scan_idx, R, t_min):
    """source(scan) = { j : ||pos(j)-pos(scan)|| <= R và |j-scan| >= t_min và j != scan }
    (mục 10.2). Tập RỖNG là chuyện có thật — seq04 t_min=50 cho 0 nguồn với mọi R."""
    pos = np.array([(p @ Tr)[:3, 3] for p in poses])
    dist = np.linalg.norm(pos - pos[scan_idx], axis=1)
    idx = np.arange(len(poses))
    ok = (dist <= R) & (np.abs(idx - scan_idx) >= t_min) & (idx != scan_idx)
    return idx[ok].tolist()


def build_source_cloud(seq, poses, Tr, scan_idx, src_indices):
    """Gộp nguồn về HỆ TOẠ ĐỘ SCAN rồi voxel. Chiều biến đổi là quan trọng: mục 10 §E2
    ghi rõ code C++ hiện tại đang đưa scan vào hệ MAP (ngược), làm ảnh scan bị dựng lại
    từ điểm nhìn của map."""
    scan_pose = poses[scan_idx] @ Tr
    inv_scan = np.linalg.inv(scan_pose)
    chunks = []
    for j in src_indices:
        T = inv_scan @ (poses[j] @ Tr)          # điểm map -> hệ scan
        chunks.append(ds.transform(load_xyz(seq, j), T))
    return voxel_downsample(np.concatenate(chunks, axis=0))


def rank_auc(score, pos):
    """AUC hạng (Mann-Whitney U), có xử lý đồng hạng. pos = mask lớp dương (GT động)."""
    n_pos, n_neg = int(pos.sum()), int((~pos).sum())
    if n_pos == 0 or n_neg == 0:
        return float('nan')
    _, inv, cnt = np.unique(score, return_inverse=True, return_counts=True)
    avg_rank = np.cumsum(cnt) - (cnt - 1) / 2.0     # hạng trung bình của mỗi nhóm đồng hạng
    ranks = avg_rank[inv]
    return float((ranks[pos].sum() - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg))


def threshold_profile(score, gt):
    """Tỷ lệ GT động và GT tĩnh vượt ngưỡng, tại từng giá trị trong THR_PROFILE —
    tiêu chí chấm số 2 của đề bài E ("threshold hết vai trò tới hạn", mục 10.7).

    Cách đọc: recall gần như KHÔNG đổi khi thr đi 1 -> 5 m  =  CAO NGUYÊN, ngưỡng
    hết là biến tới hạn. Recall tụt dần  =  còn VÁCH ĐÁ như mục 12.1 phiên 13/8.

    Không rút gọn thành một con số "khoảng ngưỡng dùng được": đuôi nhiễu quá nặng
    (đo được trên seq06/939: phân vị 90% của GT tĩnh đã là 6.2 m) nên mọi định nghĩa
    kiểu "phân vị 99% tĩnh < phân vị 10% động" đều báo CHỒNG LẤN kể cả ở bộ nguồn
    AUC 0.98 — verdict đó đúng về hình thức mà vô dụng khi đọc."""
    tpr = [float((score[gt] > t).mean()) if gt.any() else float('nan') for t in THR_PROFILE]
    fpr = [float((score[~gt] > t).mean()) if (~gt).any() else float('nan') for t in THR_PROFILE]
    return tpr, fpr


def object_speed(seq, poses, Tr, scan_idx, gt_mask, lab_raw):
    """Tốc độ vật động trung bình (m/frame), bám (class, instance) sang frame kề.
    CLAUDE.md: mọi báo cáo chất lượng phải kèm con số này, không có nó thì hai
    sequence không so được với nhau (mục 4 phiên 14/8)."""
    sem, inst = lab_raw & 0xFFFF, lab_raw >> 16
    scan_pose = poses[scan_idx] @ Tr
    scan_xyz = load_xyz(seq, scan_idx)
    speeds = []
    for cls, iid in np.unique(np.stack([sem[gt_mask], inst[gt_mask]], axis=1), axis=0):
        m = gt_mask & (sem == cls) & (inst == iid)
        if m.sum() < 5:
            continue
        c0 = ds.transform(scan_xyz[m], scan_pose).mean(axis=0)
        for nb in (scan_idx + 1, scan_idx - 1):
            if not (0 <= nb < len(poses)):
                continue
            ds.SEQ_DIR = f'{DATA}/sequences/{seq}'
            lr = np.fromfile(f'{DATA}/sequences/{seq}/labels/{nb:06d}.label', dtype=np.uint32)
            mx = load_xyz(seq, nb)
            if len(lr) != len(mx):
                continue
            mm = ((lr & 0xFFFF) == cls) & ((lr >> 16) == iid)
            if mm.sum() < 5:
                continue
            c1 = ds.transform(mx[mm], poses[nb] @ Tr).mean(axis=0)
            # poses KITTI ở hệ cam0 (y = xuống, z = tới) -> dùng norm 3D, KHÔNG lấy [:2]
            speeds.append(float(np.linalg.norm(c1 - c0)))
            break
    return float(np.mean(speeds)) if speeds else float('nan')


def measure(seq, poses, Tr, scan_idx, src_indices, gt, seed):
    """Một bộ nguồn + một ground mask -> mọi con số của một dòng bảng."""
    h, w = LEVEL
    scan_xyz = load_xyz(seq, scan_idx)
    map_xyz = build_source_cloud(seq, poses, Tr, scan_idx, src_indices)

    # ground: fit MỘT LẦN trên scan, dùng chung hệ số cho map (map đã ở hệ scan)
    nrm, d = fit_plane_ransac(scan_xyz, seed=seed)
    scan_ground = ground_mask_from_plane(scan_xyz, nrm, d)
    map_ground = ground_mask_from_plane(map_xyz, nrm, d)

    scan_img, row, col, valid = ds.build_range_image(scan_xyz, h, w, scan_ground)
    map_img, _, _, _ = ds.build_range_image(map_xyz, h, w, map_ground)

    px = row * w + col
    s = np.where(valid, scan_img[px], np.inf)
    mp = np.where(valid, map_img[px], np.inf)
    keep = valid & ~scan_ground & np.isfinite(s) & np.isfinite(mp)
    with np.errstate(invalid='ignore'):          # inf-inf ở pixel trống, đã lọc bằng `keep`
        delta = np.where(keep, mp - s, 0.0)      # >0: map XA hơn scan (scan che mất nền)

    g = gt[keep]
    a = np.abs(delta[keep])
    noise = float(np.median(a[~g])) if (~g).any() else float('nan')
    signal = float(np.median(a[g])) if g.any() else float('nan')
    tpr, fpr = threshold_profile(a, g)
    return dict(
        n_src=len(src_indices), n_map=len(map_xyz), n_eval=int(keep.sum()),
        n_gt=int(g.sum()), gnd=float(scan_ground.mean()),
        noise=noise, signal=signal,
        ratio=signal / noise if noise and noise > 0 else float('nan'),
        auc_abs=rank_auc(a, g), auc_one=rank_auc(delta[keep], g),
        **{f'tpr{i}': v for i, v in enumerate(tpr)},
        **{f'fpr{i}': v for i, v in enumerate(fpr)},
    )


def run_scan(seq, scan_idx, grid, seeds):
    ds.SEQ_DIR = f'{DATA}/sequences/{seq}'
    ds.POSES_PATH = f'{DATA}/poses/{seq}.txt'
    poses = ds.load_poses(ds.POSES_PATH)
    Tr = ds.load_tr(f'{ds.SEQ_DIR}/calib.txt')

    lab_raw = np.fromfile(f'{ds.SEQ_DIR}/labels/{scan_idx:06d}.label', dtype=np.uint32)
    gt = np.isin(lab_raw & 0xFFFF, list(ds.MOVING_CLASSES))
    spd = object_speed(seq, poses, Tr, scan_idx, gt, lab_raw)

    print(f'=== seq {seq} / scan {scan_idx} ===  {int(gt.sum())} diem GT dong, '
          f'toc do vat TB {spd:.2f} m/frame, {len(seeds)} ground mask')
    print(f'  {"R":>4}{"t_min":>7}{"#src":>6}{"#map_pt":>10}{"#eval":>9}{"#gt":>7}'
          f'{"nhieu":>8}{"tin hieu":>10}{"ty le":>8}{"AUC abs":>9}{"AUC 1ch":>9}')

    rows = []
    for R, t_min in grid:
        src = select_sources(poses, Tr, scan_idx, R, t_min)
        if not src:
            # mục 10.5 quy tắc 4: tập rỗng KHÔNG có cảnh báo tự nhiên, phải in ra
            print(f'  {R:>4}{t_min:>7}{0:>6}      --  KHONG CO FRAME NGUON '
                  f'(noi R hoac giam t_min)')
            rows.append((R, t_min, None))
            continue
        res = [measure(seq, poses, Tr, scan_idx, src, gt, seed) for seed in seeds]
        m = {k: float(np.mean([r[k] for r in res])) for k in res[0]}
        sd = float(np.std([r['auc_abs'] for r in res]))
        sd_txt = f'±{sd:.3f}' if len(seeds) > 1 else ''
        print(f'  {R:>4}{t_min:>7}{int(m["n_src"]):>6}{int(m["n_map"]):>10,}'
              f'{int(m["n_eval"]):>9,}{int(m["n_gt"]):>7}'
              f'{m["noise"]:>8.2f}{m["signal"]:>10.2f}{m["ratio"]:>8.1f}'
              f'{m["auc_abs"]:>9.3f}{sd_txt}{m["auc_one"]:>9.3f}')
        rows.append((R, t_min, m))

    # --- bảng 2: hồ sơ ngưỡng (tiêu chí chấm số 2 của đề bài E) ---
    print(f'\n  ho so nguong — recall (va % diem TINH bi flag) tai tung thr, luat |map-scan|:')
    print('  ' + f'{"R":>4}{"t_min":>7}' + ''.join(f'{f"thr={t}m":>18}' for t in THR_PROFILE))
    for R, t_min, m in rows:
        if m is None:
            continue
        cells = ''
        for i in range(len(THR_PROFILE)):
            cells += f'{m[f"tpr{i}"]:>12.2f} ({100 * m[f"fpr{i}"]:>3.0f}%)'
        print('  ' + f'{R:>4}{t_min:>7}' + cells)
    print()
    return rows


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(args) < 2:
        print(__doc__)
        sys.exit(1)
    seq = args[0]
    scans = [int(a) for a in args[1:]]

    grid = DEFAULT_GRID
    if '--grid' in sys.argv:
        grid = [tuple(int(x) for x in tok.split(':'))
                for tok in sys.argv[sys.argv.index('--grid') + 1].split(',')]
    n_seeds = 1
    if '--seeds' in sys.argv:
        n_seeds = int(sys.argv[sys.argv.index('--seeds') + 1])
    seeds = list(range(n_seeds))

    print(f'voxel {VOXEL}m | thang {LEVEL[0]}x{LEVEL[1]} | ground: 1 mat phang fit tren SCAN,')
    print('dung chung cho map (map da o he scan) | luat cham diem: |map-scan| va (map-scan)\n')
    print('Nguong dat (muc 10.1): AUC >= 0.9 tot | AUC ~ 0.5 vo dung | ty le tin hieu/nhieu >= 10')
    print('Bang 2 = ho so nguong: recall phang tren dai 1-5m => cao nguyen (tieu chi 2, de bai E).\n')

    for scan in scans:
        run_scan(seq, scan, grid, seeds)


if __name__ == '__main__':
    main()
