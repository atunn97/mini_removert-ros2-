#!/usr/bin/env python3
"""
select_source_ladder.py — chọn bộ frame nguồn cho MAP TÍCH LUỸ (đề bài E1).

Thay cho `select_maps_by_distance.py` (chọn N frame cách đều theo khoảng cách): ở đây
lấy MỌI frame thoả điều kiện, theo một THANG BẬC nới dần.

    nguồn(scan) = { j : ||pos(j)-pos(scan)|| <= R  và  |j-scan| >= t_min  và  j != scan }

Vì sao phải có thang bậc: hai điều kiện trên loại trừ nhau khi xe chạy thẳng — đòi
`t_min` lớn (cách xa về thời gian, để vật động kịp đi khỏi chỗ cũ) mà vẫn đòi `R` nhỏ
(gần về không gian, để ít parallax) thì rất nhiều scan ra TẬP RỖNG: `R=5,t_min=5` cho
0 nguồn ở 5/5 scan mốc của seq04. Nên: thử bậc tốt nhất trước, không đủ nguồn thì nới dần.

Cấu hình mặc định đã CHỐT bằng số trên 20 scan / 4 sequence (`HANDOFF_2026-08-20.md`
mục 5 + 5b) — AUC trung bình 0.881, so với 0.723 của cách chọn nguồn cũ:

    bậc 1:  R=5,  t_min=5     nếu #nguồn >= 8  -> dùng
    bậc 2:  R=10, t_min=10    nếu #nguồn >= 8  -> dùng
    bậc 3:  R=20, t_min=10    nếu #nguồn >= 1  -> dùng   (bậc cuối = lưới an toàn)
    không bậc nào đạt         -> BÁO RÕ ra stderr và thoát mã 1

Ba bậc và biên `--min-src` PHẢI là tham số dòng lệnh, không được hard-code: chúng đã
được chỉnh trên toàn bộ 20 scan có trên đĩa nên không còn tập dữ liệu sạch nào để kiểm
lại — đổi chúng về sau không được tốn dòng code nào (mục 5b).

Usage:
    python3 scripts/select_source_ladder.py <poses.txt> <calib.txt> <scan_idx>
                                            [--rungs 5:5,10:10,20:10] [--min-src 8]

Output: bảng chẩn đoán, rồi danh sách map_idx ở **DÒNG CUỐI stdout** — đúng contract của
`select_maps_by_distance.py`, để các script sweep dùng lại được mà không phải sửa.
"""
import sys

import numpy as np

DEFAULT_RUNGS = [(5, 5), (10, 10), (20, 10)]
DEFAULT_MIN_SRC = 8


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


def select_sources(poses, Tr, scan_idx, R, t_min):
    """source(scan) = { j : ||pos(j)-pos(scan)|| <= R và |j-scan| >= t_min và j != scan }
    (mục 10.2). Tập RỖNG là chuyện có thật — seq04 t_min=50 cho 0 nguồn với mọi R."""
    pos = np.array([(p @ Tr)[:3, 3] for p in poses])
    dist = np.linalg.norm(pos - pos[scan_idx], axis=1)
    idx = np.arange(len(poses))
    ok = (dist <= R) & (np.abs(idx - scan_idx) >= t_min) & (idx != scan_idx)
    return idx[ok].tolist()


def pick_rung(poses, Tr, scan_idx, rungs, min_src):
    """Duyệt thang bậc, trả (số thứ tự bậc, R, t_min, danh sách nguồn, bảng chẩn đoán).

    Biên `min_src` áp cho MỌI bậc TRỪ bậc cuối: bậc cuối là lưới an toàn, chỉ đòi >= 1.
    Áp biên cho cả bậc cuối thì scan nghèo nguồn rơi thẳng xuống "không bậc nào đạt",
    mất trắng những scan mà bậc cuối vẫn cứu được.
    """
    table = []
    for i, (R, t_min) in enumerate(rungs):
        need = min_src if i < len(rungs) - 1 else 1
        src = select_sources(poses, Tr, scan_idx, R, t_min)
        # >= chứ KHÔNG phải >: seq06/939 có ĐÚNG 8 nguồn ở bậc 1, tức nằm chính xác
        # trên biên. Lấy bậc 1 -> AUC 0.980; rơi xuống bậc cuối -> 0.892 (mục 3 + 5).
        ok = len(src) >= need
        table.append((i + 1, R, t_min, len(src), need, ok))
        if ok:
            return i + 1, R, t_min, src, table
    return None, None, None, None, table


def parse_args(argv):
    """Parse tay, theo đúng luật của main.cpp: cờ lạ hoặc thiếu giá trị thì BÁO LỖI VÀ
    THOÁT, không im lặng bỏ qua."""
    positional = []
    rungs = DEFAULT_RUNGS
    min_src = DEFAULT_MIN_SRC

    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == "--rungs":
            if i + 1 >= len(argv):
                sys.exit(f"thieu gia tri sau {arg}")
            try:
                rungs = [tuple(int(x) for x in tok.split(":"))
                         for tok in argv[i + 1].split(",")]
            except ValueError:
                sys.exit(f"--rungs sai dinh dang (can R:t_min,...): {argv[i + 1]}")
            if not rungs or any(len(r) != 2 for r in rungs):
                sys.exit(f"--rungs sai dinh dang (can R:t_min,...): {argv[i + 1]}")
            i += 2
        elif arg == "--min-src":
            if i + 1 >= len(argv):
                sys.exit(f"thieu gia tri sau {arg}")
            min_src = int(argv[i + 1])
            i += 2
        elif arg.startswith("--"):
            sys.exit(f"co la: {arg}")
        else:
            positional.append(arg)
            i += 1

    if len(positional) != 3:
        print(__doc__)
        sys.exit(1)
    return positional[0], positional[1], int(positional[2]), rungs, min_src


def main():
    poses_path, calib_path, scan_idx, rungs, min_src = parse_args(sys.argv[1:])

    poses = load_poses(poses_path)
    Tr = load_tr(calib_path)
    if not 0 <= scan_idx < len(poses):
        sys.exit(f"scan_idx {scan_idx} nam ngoai [0, {len(poses)})")

    rung_no, R, t_min, src, table = pick_rung(poses, Tr, scan_idx, rungs, min_src)

    print(f"scan_idx={scan_idx}  thang bac={','.join(f'{r}:{t}' for r, t in rungs)}"
          f"  min_src={min_src}")
    print("")
    print(f"{'bac':<6}{'R':<6}{'t_min':<8}{'#nguon':<9}{'can':<6}{'ket qua':<16}")
    for i, r, t, n, need, ok in table:
        verdict = "DUNG BAC NAY" if ok else "khong du -> noi"
        print(f"{i:<6}{r:<6}{t:<8}{n:<9}{need:<6}{verdict:<16}")
    print("")

    if src is None:
        # Mục 5: KHÔNG fallback im lặng về t_min=1. Fallback im lặng là cách chắc chắn
        # nhất để về sau ngồi nhìn một con số F1 mà không biết nó đến từ cấu hình nào.
        print(f"KHONG BAC NAO DAT cho scan {scan_idx} — bo qua scan nay "
              f"(noi R, giam t_min, hoac ha --min-src)", file=sys.stderr)
        sys.exit(1)

    # Tiêu chí chấm số 4 (mục 9 phiên 20/8): phải in R và #nguồn THỰC DÙNG — đó là
    # thứ đầu tiên phải xem khi kết quả lạ.
    print(f"CHON bac {rung_no}: R={R}, t_min={t_min}, {len(src)} nguon")
    print("")
    print("map_idx (dung truc tiep):")
    print(" ".join(str(j) for j in src))


if __name__ == "__main__":
    main()
