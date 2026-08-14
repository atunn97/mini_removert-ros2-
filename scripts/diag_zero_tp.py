#!/usr/bin/env python3
"""
diag_zero_tp.py — dieu tra 3 scan co P = R = F1 = 0.000 tuyet doi
(HANDOFF_2026-08-13 muc 16.6: seq06/349, seq06/939, seq07/248).

Cau hoi: du doan hang tram diem ma KHONG trung MOT diem nao voi ground truth
         -> day la mot che do hong rieng, hay chi la "kem chinh xac"?

Script KHONG do F1. No do DIEM DONG THAT CHET O DAU, bang 2 phep doc lap:

  A) PHEU 6 TANG (phu thuoc pipeline) - mo phong lai main.cpp bang numpy,
     dem xem trong so diem GT dynamic con bao nhieu sau moi tang:
        GT -> valid(FOV) -> khong-ground -> duoc quan sat -> REMOVE@L0
           -> xac nhan @L1 -> @L2 -> @L3
     Tang nao lam so diem sut manh nhat chinh la che do hong.

  B) VAT DO CO THAT SU DI CHUYEN KHONG (doc lap hoan toan voi pipeline)
     Nhan SemanticKITTI: 16 bit thap = class, 16 bit CAO = instance id.
     Bam theo cung mot instance qua scan va cac map, doi ca hai ve HE TOA DO
     THE GIOI roi do do dich chuyen cua trong tam.
        - dich chuyen ~ 0     -> vat mang nhan `moving-*` nhung dang DUNG YEN
                                 => khong the bat duoc, day la gioi han NGUYEN LY
        - dich chuyen ~ 1.4 m -> vat that su chay => loi nam trong pipeline

Chay tren NHIEU ground mask (seed) vi F1 cuc nhay voi mask (muc 11).

Usage:
    python3 scripts/diag_zero_tp.py <seq> <scan_idx> [seq scan ...]
    python3 scripts/diag_zero_tp.py 06 349 06 939 07 248 04 150
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from select_maps_by_distance import load_poses, load_tr, pick_side

D = os.path.expanduser("~/kitti_data/dataset")
THRESHOLD = 1.0                 # moc chot 2026-08-13 muc 12
VOTE_THRESHOLD = 0.5
V_MIN_DEG, V_MAX_DEG = -24.8, 2.0
LEVELS = [(64, 900), (32, 450), (16, 225), (8, 112)]   # 4 thang, muc 14.2
GROUND_DIST_THR = 0.2
GROUND_ITERS = 200
N_MAPS, MAX_DIST = 4, 4.0
MOVING = list(range(252, 260))
SEEDS = [0, 1, 2]

CLASS_NAMES = {
    0: "unlabeled", 1: "outlier", 10: "car", 11: "bicycle", 13: "bus", 15: "motorcycle",
    18: "truck", 20: "other-vehicle", 30: "person", 31: "bicyclist", 32: "motorcyclist",
    40: "road", 44: "parking", 48: "sidewalk", 49: "other-ground", 50: "building",
    51: "fence", 52: "other-structure", 60: "lane-marking", 70: "vegetation",
    71: "trunk", 72: "terrain", 80: "pole", 81: "traffic-sign", 99: "other-object",
    252: "moving-car", 253: "moving-bicyclist", 254: "moving-person",
    255: "moving-motorcyclist", 256: "moving-on-rails", 257: "moving-bus",
    258: "moving-truck", 259: "moving-other-veh",
}


# ---------------------------------------------------------------- I/O
def load_xyz(seq, idx):
    p = f"{D}/sequences/{seq}/velodyne/{idx:06d}.bin"
    return np.fromfile(p, dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


def load_label_raw(seq, idx):
    return np.fromfile(f"{D}/sequences/{seq}/labels/{idx:06d}.label", dtype=np.uint32)


def pick_maps(poses, Tr, scan_idx):
    """Ban sao logic select_maps_by_distance.main()."""
    def pos(i):
        return (poses[i] @ Tr)[:3, 3]
    n = len(poses)
    sp = pos(scan_idx)
    bi = list(range(scan_idx - 1, -1, -1))
    ai = list(range(scan_idx + 1, n))
    b = pick_side(sp, [pos(i) for i in bi], bi, N_MAPS // 2, MAX_DIST)
    a = pick_side(sp, [pos(i) for i in ai], ai, N_MAPS // 2, MAX_DIST)
    return [i for i, _ in sorted(b + a, key=lambda x: x[0])]


# ------------------------------------------------- ban sao pipeline C++
def ransac_ground(xyz, seed):
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
        inl = np.abs(xyz @ nrm - nrm @ p0) <= GROUND_DIST_THR
        c = int(inl.sum())
        if c > best_cnt:
            best_cnt, best_inl = c, inl
    pts = xyz[best_inl]
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    nrm = vt[-1]
    return np.abs(xyz @ nrm - nrm @ centroid) <= GROUND_DIST_THR


def project(xyz, h, w):
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
    row, col, dist, valid = project(xyz, h, w)
    use = valid & ~exclude
    img = np.full(h * w, np.inf)
    np.minimum.at(img, row[use] * w + col[use], dist[use])
    return img, row, col, valid


def transform(xyz, T):
    return xyz @ T[:3, :3].T + T[:3, 3]


# ------------------------------------------------------- phep do B
def instance_motion(seq, scan_idx, map_indices, poses, Tr, gt_mask, lab_raw):
    """Vat mang nhan `moving-*` co THAT SU di chuyen khong?
    Bam theo (class, instance_id) qua cac frame, doi ve he the gioi, do trong tam.
    Doc lap hoan toan voi range image / ground filter / threshold."""
    scan_xyz = load_xyz(seq, scan_idx)
    scan_pose = poses[scan_idx] @ Tr
    sem = lab_raw & 0xFFFF
    inst = lab_raw >> 16

    keys = np.unique(np.stack([sem[gt_mask], inst[gt_mask]], axis=1), axis=0)
    rows = []
    for cls, iid in keys:
        m = gt_mask & (sem == cls) & (inst == iid)
        npt = int(m.sum())
        c_scan = transform(scan_xyz[m], scan_pose).mean(axis=0)
        disp, seen = [], 0
        for mi in map_indices:
            lr = load_label_raw(seq, mi)
            mx = load_xyz(seq, mi)
            if len(lr) != len(mx):
                continue
            mm = ((lr & 0xFFFF) == cls) & ((lr >> 16) == iid)
            if mm.sum() < 5:
                disp.append(np.nan)
                continue
            seen += 1
            c_map = transform(mx[mm], poses[mi] @ Tr).mean(axis=0)
            # LUU Y: poses KITTI odometry o he CAM0 (x=phai, y=XUONG, z=TOI).
            # Dung norm 3D. KHONG duoc lay [:2] tuong la "mat phang ngang" — (x,y)
            # la mat phang DUNG, lay [:2] se bo mat huong tien z va moi vat dang
            # chay thang deu hien ra nhu dang dung yen.
            disp.append(float(np.linalg.norm(c_map - c_scan)))
        rows.append((int(cls), int(iid), npt, seen, disp,
                     float(np.linalg.norm(c_scan - scan_pose[:3, 3]))))
    return rows


# ------------------------------------------------------- phep do A
def funnel(seq, scan_idx, map_indices, poses, Tr, gt_mask, seed):
    scan_xyz = load_xyz(seq, scan_idx)
    n = len(scan_xyz)
    scan_pose = poses[scan_idx] @ Tr
    scan_ground = ransac_ground(scan_xyz, seed)

    nl = len(LEVELS)
    vote = [np.zeros(n, dtype=np.int32) for _ in range(nl)]
    obs = [np.zeros(n, dtype=np.int32) for _ in range(nl)]
    valid0 = None
    diff_gt = {}     # map_idx -> mang (map - scan) tren diem GT, thang L0

    for mi in map_indices:
        map_xyz = load_xyz(seq, mi)
        T_rel = np.linalg.inv(poses[mi] @ Tr) @ scan_pose
        scan_in_map = transform(scan_xyz, T_rel)
        map_ground = ransac_ground(map_xyz, seed)

        for li, (h, w) in enumerate(LEVELS):
            s_img, row, col, valid = build_range_image(scan_in_map, h, w, scan_ground)
            m_img, _, _, _ = build_range_image(map_xyz, h, w, map_ground)
            px = row * w + col
            s = np.where(valid, s_img[px], np.inf)
            mp = np.where(valid, m_img[px], np.inf)
            both = np.isfinite(s) & np.isfinite(mp)
            with np.errstate(invalid="ignore"):
                diff = np.where(both, mp - s, 0.0)
            vote[li] += both & (np.abs(diff) > THRESHOLD)
            obs[li] += valid & np.isfinite(mp)
            if li == 0:
                valid0 = valid
                d = np.where(both & gt_mask, diff, np.nan)
                diff_gt[mi] = d[gt_mask]

    def is_dyn(li):
        r = np.where(obs[li] > 0, vote[li] / np.maximum(obs[li], 1), 0.0)
        return (obs[li] > 0) & (r > VOTE_THRESHOLD)

    stages = [("GT dynamic", gt_mask.copy())]
    cur = gt_mask & valid0
    stages.append(("trong FOV", cur.copy()))
    cur = cur & ~scan_ground
    stages.append(("khong-ground", cur.copy()))
    cur = cur & (obs[0] > 0)
    stages.append(("duoc quan sat", cur.copy()))
    cur = cur & is_dyn(0)
    stages.append(("REMOVE @64x900", cur.copy()))
    for li in range(1, len(LEVELS)):
        cur = cur & is_dyn(li)
        stages.append((f"xac nhan @{LEVELS[li][0]}x{LEVELS[li][1]}", cur.copy()))

    pred = ~scan_ground & is_dyn(0)
    for li in range(1, len(LEVELS)):
        pred &= is_dyn(li)
    return stages, pred, diff_gt, scan_ground


# ------------------------------------------------------------- bao cao
def report(seq, scan_idx):
    poses = load_poses(f"{D}/poses/{seq}.txt")
    Tr = load_tr(f"{D}/sequences/{seq}/calib.txt")
    map_indices = pick_maps(poses, Tr, scan_idx)

    lab_raw = load_label_raw(seq, scan_idx)
    sem = lab_raw & 0xFFFF
    gt = np.isin(sem, MOVING)

    print(f"\n{'='*72}")
    print(f"seq {seq}  scan {scan_idx}   map = {map_indices}")
    print(f"tong diem = {len(sem)}   GT dynamic = {int(gt.sum())}")
    cls, cnt = np.unique(sem[gt], return_counts=True)
    print("  class cua GT dynamic: " +
          ", ".join(f"{CLASS_NAMES.get(int(c), c)}={int(k)}" for c, k in zip(cls, cnt)))
    print(f"{'='*72}")

    # ---- B: vat co di chuyen khong ----
    print("\n[B] VAT CO THAT SU DI CHUYEN KHONG (doc lap voi pipeline)")
    print(f"    dich chuyen trong tam giua scan va tung map, he toa do THE GIOI, norm 3D"
          f"  (threshold={THRESHOLD} m)")
    rows = instance_motion(seq, scan_idx, map_indices, poses, Tr, gt, lab_raw)
    hdr = f"    {'class':<18}{'inst':>6}{'#pt':>6}{'range':>8}  " + \
          "".join(f"{('m'+str(m)):>9}" for m in map_indices) + f"{'ket luan':>18}"
    print(hdr)
    for c, iid, npt, seen, disp, rng_m in rows:
        ds = "".join((f"{d:>9.2f}" if np.isfinite(d) else f"{'-':>9}") for d in disp)
        fin = [d for d in disp if np.isfinite(d)]
        # nguong so voi THRESHOLD: dich chuyen scan<->map phai VUOT threshold thi
        # discrepancy moi vuot nguong duoc (dieu kien can, chua du - con phu thuoc
        # huong dich chuyen so voi tia nhin, xem HANDOFF muc 18).
        if not fin:
            verdict = "khong thay o map"
        elif max(fin) < THRESHOLD:
            verdict = "DUOI NGUONG"
        elif min(fin) < THRESHOLD:
            verdict = "mot phan duoi nguong"
        else:
            verdict = "tren nguong"
        print(f"    {CLASS_NAMES.get(c, c):<18}{iid:>6}{npt:>6}{rng_m:>8.1f}{ds}{verdict:>18}")

    # ---- A: pheu ----
    print("\n[A] PHEU: trong so diem GT dynamic, con lai bao nhieu sau moi tang")
    all_stages, all_pred, all_diff = [], [], []
    for seed in SEEDS:
        st, pred, dgt, gmask = funnel(seq, scan_idx, map_indices, poses, Tr, gt, seed)
        all_stages.append(st)
        all_pred.append(pred)
        all_diff.append(dgt)
    names = [n for n, _ in all_stages[0]]
    print(f"    {'tang':<24}" + "".join(f"{('seed'+str(s)):>10}" for s in SEEDS) + f"{'% con lai':>12}")
    n_gt = int(gt.sum())
    for k, nm in enumerate(names):
        vals = [int(st[k][1].sum()) for st in all_stages]
        print(f"    {nm:<24}" + "".join(f"{v:>10}" for v in vals) +
              f"{100.0*np.mean(vals)/max(n_gt,1):>11.1f}%")
    print(f"    {'-> TP cuoi cung':<24}" +
          "".join(f"{int((p & gt).sum()):>10}" for p in all_pred))
    print(f"    {'   (tong du doan)':<24}" +
          "".join(f"{int(p.sum()):>10}" for p in all_pred))

    # ---- dau discrepancy tren diem GT (nhu muc 3) ----
    print("\n[C] DISCREPANCY (map - scan) TAI PIXEL CUA DIEM GT, thang 64x900, seed 0")
    print(f"    {'map':>8}{'co du lieu':>12}{'trung vi':>12}{'|trung vi|':>12}{'>thr(1.0)':>12}")
    for mi in map_indices:
        d = all_diff[0][mi]
        fin = d[np.isfinite(d)]
        if len(fin) == 0:
            print(f"    {mi:>8}{0:>12}")
            continue
        print(f"    {mi:>8}{len(fin):>12}{np.median(fin):>12.2f}"
              f"{np.median(np.abs(fin)):>12.2f}"
              f"{100.0*np.mean(np.abs(fin) > THRESHOLD):>11.1f}%")

    # ---- FP roi vao class nao ----
    pred0 = all_pred[0]
    fp = pred0 & ~gt
    if fp.sum():
        l, c = np.unique(sem[fp], return_counts=True)
        o = np.argsort(-c)[:6]
        print(f"\n[D] FP (seed 0, tong {int(fp.sum())}) roi vao class:")
        for i in o:
            print(f"    {CLASS_NAMES.get(int(l[i]), int(l[i])):<18}{int(c[i]):>7}"
                  f"  ({100.0*c[i]/fp.sum():>5.1f}%)")


def main():
    a = sys.argv[1:]
    if len(a) < 2 or len(a) % 2:
        print(__doc__)
        sys.exit(1)
    for i in range(0, len(a), 2):
        report(a[i], int(a[i + 1]))


if __name__ == "__main__":
    main()
