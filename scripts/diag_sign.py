#!/usr/bin/env python3
"""
diag_sign.py — kiem chung gia thuyet A:
    `fabs()` trong discrepancy.cpp:58 lam mat DAU cua hieu (scan_range - map_range),
    khien diem NEN phia sau mot vat da bien mat khoi map bi gan nham la dynamic.

Phuong phap (giong muc 3 HANDOFF_2026-08-12): KHONG sua code core.
Mo phong lai TOAN BO pipeline C++ bang numpy, roi so 2 luat:
    (a) ABS   : dynamic <=> |scan - map| > threshold          (code hien tai)
    (b) SIGNED: dynamic <=> (map - scan)  > threshold          (scan GAN hon map)

In ra: P/R/F1 cua ca 2 luat + bang cheo TP/FP theo dau + phan loai FP theo class.

Usage:
    python3 diag_sign.py <scan_idx> <map_idx_1> [map_idx_2] ...
"""
import sys
import numpy as np

SEQ_DIR = "/home/atun/kitti_data/dataset/sequences/04"
POSES_PATH = "/home/atun/kitti_data/dataset/poses/04.txt"
THRESHOLD = 0.5
VOTE_THRESHOLD = 0.5
V_MIN_DEG, V_MAX_DEG = -24.8, 2.0
LEVELS = [(64, 900), (32, 450), (16, 225)]   # [0] = remove, [1:] = revert
GROUND_DIST_THR = 0.2
GROUND_ITERS = 200
MOVING_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259}

# SemanticKITTI: ten class de phan tich FP
CLASS_NAMES = {
    0: "unlabeled", 1: "outlier", 10: "car", 11: "bicycle", 13: "bus", 15: "motorcycle",
    18: "truck", 20: "other-vehicle", 30: "person", 31: "bicyclist", 32: "motorcyclist",
    40: "road", 44: "parking", 48: "sidewalk", 49: "other-ground", 50: "building",
    51: "fence", 52: "other-structure", 60: "lane-marking", 70: "vegetation",
    71: "trunk", 72: "terrain", 80: "pole", 81: "traffic-sign", 99: "other-object",
}


def load_poses(path):
    out = []
    with open(path) as f:
        for line in f:
            v = list(map(float, line.split()))
            T = np.eye(4)
            T[:3, :4] = np.array(v).reshape(3, 4)
            out.append(T)
    return out


def load_tr(path):
    with open(path) as f:
        for line in f:
            if line.startswith("Tr:"):
                v = list(map(float, line[3:].split()))
                T = np.eye(4)
                T[:3, :4] = np.array(v).reshape(3, 4)
                return T
    raise RuntimeError("khong tim thay Tr:")


def load_xyz(idx):
    """Doc .bin — dung thu tu diem giong het .pcd ma C++ doc (bin_to_pcd chi lay [:, :3])."""
    p = f"{SEQ_DIR}/velodyne/{idx:06d}.bin"
    return np.fromfile(p, dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


def load_labels(idx):
    p = f"{SEQ_DIR}/labels/{idx:06d}.label"
    return np.fromfile(p, dtype=np.uint32) & 0xFFFF


def ransac_ground(xyz, seed=0):
    """Ban sao cua ground_filter.cpp: SACMODEL_PLANE + SAC_RANSAC, 200 iter,
    distance_threshold=0.2, setOptimizeCoefficients(true) -> refit least-squares."""
    rng = np.random.default_rng(seed)
    n = len(xyz)
    best_inl, best_cnt = None, -1
    for _ in range(GROUND_ITERS):
        i = rng.choice(n, 3, replace=False)
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
    # refit least-squares tren inliers (setOptimizeCoefficients)
    pts = xyz[best_inl]
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    nrm = vt[-1]
    d = -nrm @ centroid
    return np.abs(xyz @ nrm + d) <= GROUND_DIST_THR


def ransac_ground_pcl(xyz, seed=0, prob=0.99):
    """Giong ransac_ground nhung co DUNG SOM theo xac suat, dung nhu
    pcl::RandomSampleConsensus::computeModel:
        k = log(1-prob) / log(1 - w^3),  w = ty le inlier tot nhat hien tai
    Voi w ~ 0.5 thi k ~ 35 -> PCL THUC TE chi chay ~35 vong, khong phai 200.
    Giu lai de doi chieu; ket luan o HANDOFF_2026-08-13 muc 11: dung ham nao cung
    khong lam mo phong khop C++ duoc, vi F1 qua nhay voi ground mask."""
    rng = np.random.default_rng(seed)
    n = len(xyz)
    best, best_cnt, k, it = None, -1, np.inf, 0
    while it < k and it < GROUND_ITERS:
        i = rng.choice(n, 3, replace=False)
        p0, p1, p2 = xyz[i]
        nrm = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(nrm)
        if nn > 1e-9:
            nrm = nrm / nn
            d_ = -nrm @ p0
            inl = np.abs(xyz @ nrm + d_) <= GROUND_DIST_THR
            c = int(inl.sum())
            if c > best_cnt:
                best_cnt, best = c, inl
                w = best_cnt / n
                p_no = 1.0 - w ** 3
                if 0.0 < p_no < 1.0:
                    k = np.log(1.0 - prob) / np.log(p_no)
        it += 1
    pts = xyz[best]
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    nrm = vt[-1]
    d_ = -nrm @ centroid
    return np.abs(xyz @ nrm + d_) <= GROUND_DIST_THR


def project(xyz, h, w):
    """Ban sao cua range_image::projectToPixel (vectorised)."""
    v_min, v_max = np.deg2rad(V_MIN_DEG), np.deg2rad(V_MAX_DEG)
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rho = np.sqrt(x * x + y * y)
    dist = np.sqrt(x * x + y * y + z * z)
    vang = np.arctan2(z, rho)
    hang = np.arctan2(y, x)
    valid = (dist >= 0.1) & (vang >= v_min) & (vang <= v_max)
    row = ((vang - v_min) / (v_max - v_min) * h).astype(np.int64)
    col = ((hang + np.pi) / (2.0 * np.pi) * w).astype(np.int64)
    np.clip(row, 0, h - 1, out=row)
    np.clip(col, 0, w - 1, out=col)
    return row, col, dist, valid


def build_range_image(xyz, h, w, exclude):
    """Ban sao cua range_image::buildRangeImage: pixel = min(range), o trong = +inf."""
    row, col, dist, valid = project(xyz, h, w)
    use = valid & ~exclude
    img = np.full(h * w, np.inf)
    np.minimum.at(img, row[use] * w + col[use], dist[use])
    return img, row, col, valid


def transform(xyz, T):
    return xyz @ T[:3, :3].T + T[:3, 3]


def decide(vote, obs, ground_mask, n_points):
    """Ban sao tang quyet dinh main.cpp:160-203 (ground=static, remove@L0, revert@all)."""
    def is_dyn(li):
        with np.errstate(invalid="ignore", divide="ignore"):
            r = np.where(obs[li] > 0, vote[li] / np.maximum(obs[li], 1), 0.0)
        return (obs[li] > 0) & (r > VOTE_THRESHOLD)

    keep = ~ground_mask & is_dyn(0)
    for li in range(1, len(LEVELS)):
        keep &= is_dyn(li)
    return keep


def prf(pred, gt):
    tp = int((pred & gt).sum())
    fp = int((pred & ~gt).sum())
    fn = int((~pred & gt).sum())
    p = tp / (tp + fp + 1e-9)
    r = tp / (tp + fn + 1e-9)
    return p, r, 2 * p * r / (p + r + 1e-9), tp, fp, fn


def run_scan(scan_idx, map_indices, poses, Tr, verbose=True):
    scan_xyz = load_xyz(scan_idx)
    n = len(scan_xyz)
    scan_pose = poses[scan_idx] @ Tr
    scan_ground = ransac_ground(scan_xyz, seed=0)

    n_lv = len(LEVELS)
    vote_abs = [np.zeros(n, dtype=np.int32) for _ in range(n_lv)]
    vote_sgn = [np.zeros(n, dtype=np.int32) for _ in range(n_lv)]
    obs = [np.zeros(n, dtype=np.int32) for _ in range(n_lv)]
    # dem theo dau, chi o thang REMOVE (level 0), de lam bang cheo
    flag_closer = np.zeros(n, dtype=np.int32)   # scan gan hon map  -> bang chung dung
    flag_farther = np.zeros(n, dtype=np.int32)  # scan xa hon map   -> nghi la FP

    for m in map_indices:
        map_xyz = load_xyz(m)
        map_pose = poses[m] @ Tr
        T_rel = np.linalg.inv(map_pose) @ scan_pose
        scan_in_map = transform(scan_xyz, T_rel)
        map_ground = ransac_ground(map_xyz, seed=0)

        for li, (h, w) in enumerate(LEVELS):
            scan_img, row, col, valid = build_range_image(scan_in_map, h, w, scan_ground)
            map_img, _, _, _ = build_range_image(map_xyz, h, w, map_ground)

            px = row * w + col
            s = np.where(valid, scan_img[px], np.inf)
            mp = np.where(valid, map_img[px], np.inf)
            both = np.isfinite(s) & np.isfinite(mp)
            with np.errstate(invalid="ignore"):  # inf - inf o pixel trong -> NaN, da loc bang `both`
                diff = np.where(both, mp - s, 0.0)   # >0: scan GAN hon map

            dyn_abs = both & (np.abs(diff) > THRESHOLD)
            dyn_sgn = both & (diff > THRESHOLD)

            vote_abs[li] += dyn_abs
            vote_sgn[li] += dyn_sgn
            # getObservedIndices: chi can map_img huu han tai pixel cua diem
            obs[li] += valid & np.isfinite(mp)

            if li == 0:
                flag_closer += dyn_abs & (diff > 0)
                flag_farther += dyn_abs & (diff < 0)

    pred_abs = decide(vote_abs, obs, scan_ground, n)
    pred_sgn = decide(vote_sgn, obs, scan_ground, n)

    labels = load_labels(scan_idx)
    gt = np.isin(labels, list(MOVING_CLASSES))

    return dict(n=n, labels=labels, gt=gt, pred_abs=pred_abs, pred_sgn=pred_sgn,
                flag_closer=flag_closer, flag_farther=flag_farther)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    scan_idx = int(sys.argv[1])
    map_indices = [int(x) for x in sys.argv[2:]]

    poses = load_poses(POSES_PATH)
    Tr = load_tr(f"{SEQ_DIR}/calib.txt")

    print(f"scan_idx={scan_idx}  map_idx={map_indices}  threshold={THRESHOLD}")
    res = run_scan(scan_idx, map_indices, poses, Tr)

    for name, pred in (("ABS (code hien tai)", res["pred_abs"]), ("SIGNED (map-scan>thr)", res["pred_sgn"])):
        p, r, f1, tp, fp, fn = prf(pred, res["gt"])
        print(f"  {name:24s} P={p:.3f} R={r:.3f} F1={f1:.3f}   TP={tp:6d} FP={fp:6d} FN={fn:6d}")

    # bang cheo: trong so diem ABS du doan dynamic, bao nhieu den tu nhanh "scan xa hon"
    pred = res["pred_abs"]
    gt = res["gt"]
    farther = pred & (res["flag_farther"] > res["flag_closer"])
    closer = pred & ~farther
    print("\n  Bang cheo (chi tren diem ABS du doan dynamic), theo dau o thang REMOVE:")
    print(f"    {'nhanh':<16}{'tong':>8}{'TP':>8}{'FP':>8}")
    for nm, msk in (("scan GAN hon", closer), ("scan XA hon", farther)):
        print(f"    {nm:<16}{int(msk.sum()):>8}{int((msk & gt).sum()):>8}{int((msk & ~gt).sum()):>8}")

    fp_far = farther & ~gt
    if fp_far.sum():
        lab, cnt = np.unique(res["labels"][fp_far], return_counts=True)
        order = np.argsort(-cnt)[:6]
        print("\n  Class cua FP thuoc nhanh 'scan XA hon':")
        for i in order:
            print(f"    {CLASS_NAMES.get(int(lab[i]), str(lab[i])):<16}{int(cnt[i]):>8}"
                  f"  ({100.0*cnt[i]/fp_far.sum():.1f}%)")


if __name__ == "__main__":
    main()
