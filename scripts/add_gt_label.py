#!/usr/bin/env python3
"""
add_gt_label.py — ghép cột `gt_label` (nhãn THẬT) vào CSV do mini_removert xuất ra.

VÌ SAO PHẢI TÁCH RA MỘT BƯỚC RIÊNG: nhãn thật nằm trong file `.label` của SemanticKITTI,
mà C++ không đọc định dạng đó (chỉ `evaluate.py` đọc). Dạy C++ đọc thêm một định dạng nữa
là làm nó phình ra vô ích — ghép bằng Python theo `point_id` rẻ hơn nhiều.

QUY ƯỚC NHÃN — chép nguyên từ `evaluate.py` để hai bên KHÔNG THỂ lệch nhau:

    labels   = np.fromfile(path, dtype=np.uint32)
    semantic = labels & 0xFFFF                      # 16 bit thấp = lớp ngữ nghĩa
    dynamic  = semantic in {252..259}               # các lớp "moving-*" của SemanticKITTI

    252 moving-car        253 moving-bicyclist   254 moving-person   255 moving-motorcyclist
    256 moving-on-rails   257 moving-bus         258 moving-truck    259 moving-other-vehicle

Lưu ý: chỉ lớp "moving-*" mới tính là động. Một chiếc xe ĐỖ mang lớp 10 (car), nhãn 0 —
đúng như vậy, vì nó đứng yên nên removert cũng không nên xoá nó.

BA PHÉP KIỂM tự động, sai là DỪNG chứ không ghi file hỏng:
  1. số dòng của mỗi scan phải BẰNG số điểm trong file .label tương ứng
  2. `point_id` phải chạy đúng 0..n-1 theo thứ tự — đây là khoá ghép, lệch một dòng là
     toàn bộ nhãn lệch theo mà không có dấu hiệu gì
  3. Tính lại P/R/F1 giữa `baseline_label` và `gt_label` vừa ghép. Con số này PHẢI trùng
     với `evaluate.py`. Trùng = ghép đúng; lệch = ghép sai ở đâu đó.

Phép kiểm 3 là quan trọng nhất: nó tái lập một con số đã được tính ĐỘC LẬP bằng đường
khác, nên ghép sai gần như không thể lọt qua.

Usage:
    python3 scripts/add_gt_label.py scan150.csv 04
    python3 scripts/add_gt_label.py gop.csv 06 --out gop_co_nhan.csv

Mặc định ghi ra `<ten>_gt.csv`, KHÔNG ghi đè file vào — chạy lại C++ tốn hàng phút.
"""
import csv
import os
import sys
from collections import Counter

import numpy as np

DATA = os.path.expanduser('~/kitti_data/dataset')
MOVING = [252, 253, 254, 255, 256, 257, 258, 259]     # y hệt evaluate.py


def load_gt(seq, scan_id):
    path = f'{DATA}/sequences/{seq}/labels/{scan_id:06d}.label'
    if not os.path.exists(path):
        sys.exit(f'LOI: khong thay file nhan {path}')
    lab = np.fromfile(path, dtype=np.uint32) & 0xFFFF
    return np.isin(lab, MOVING).astype(np.uint8)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(args) < 2:
        print(__doc__)
        sys.exit(1)
    csv_in, seq = args[0], args[1]
    out = None
    if '--out' in sys.argv:
        out = sys.argv[sys.argv.index('--out') + 1]
    if out is None:
        stem = csv_in[:-4] if csv_in.endswith('.csv') else csv_in
        out = stem + '_gt.csv'
    if os.path.abspath(out) == os.path.abspath(csv_in):
        sys.exit('LOI: file ra trung file vao, se mat du lieu goc')

    # --- Lượt 1: đếm số dòng mỗi scan, chỉ đọc cột đầu ---
    print('Luot 1: dem dong va kiem khoa ghep...')
    counts = Counter()
    with open(csv_in, newline='') as f:
        r = csv.reader(f)
        header = next(r)
        for need in ('scan_id', 'point_id', 'baseline_label'):
            if need not in header:
                sys.exit(f'LOI: CSV thieu cot bat buoc `{need}`')
        i_scan, i_pid = header.index('scan_id'), header.index('point_id')
        expect = {}
        for row in r:
            s = int(row[i_scan])
            # KIỂM 2: point_id phải là 0,1,2,... liên tục trong từng scan
            if int(row[i_pid]) != expect.get(s, 0):
                sys.exit(f'LOI: point_id khong lien tuc o scan {s}, dong {counts[s]} '
                         f'(cho {expect.get(s,0)}, gap {row[i_pid]}). '
                         f'Khoa ghep hong -> moi nhan se lech.')
            expect[s] = expect.get(s, 0) + 1
            counts[s] += 1

    # --- KIỂM 1: số dòng khớp số điểm trong .label ---
    gt = {}
    for s in sorted(counts):
        g = load_gt(seq, s)
        if len(g) != counts[s]:
            sys.exit(f'LOI: scan {s} co {counts[s]} dong trong CSV nhung file .label '
                     f'co {len(g)} diem. Sai sequence, hay CSV cua scan khac?')
        gt[s] = g
        print(f'  scan {s}: {counts[s]:,} dong, khop .label, GT dynamic = {int(g.sum()):,}')

    # --- Lượt 2: ghi ra, thêm một cột ---
    print(f'Luot 2: ghi {out} ...')
    i_base = header.index('baseline_label')
    stat = {s: dict(tp=0, fp=0, fn=0) for s in counts}
    with open(csv_in, newline='') as fi, open(out, 'w', newline='') as fo:
        r, w = csv.reader(fi), csv.writer(fo)
        next(r)
        w.writerow(header + ['gt_label'])
        pos = Counter()
        for row in r:
            s = int(row[i_scan])
            g = int(gt[s][pos[s]])
            pos[s] += 1
            w.writerow(row + [g])
            b = int(row[i_base])
            if   b and g:         stat[s]['tp'] += 1
            elif b and not g:     stat[s]['fp'] += 1
            elif not b and g:     stat[s]['fn'] += 1

    # --- KIỂM 3: tái lập P/R/F1, phải trùng evaluate.py ---
    print('\n=== DOI CHIEU: cac so nay PHAI trung evaluate.py ===')
    print(f'{"scan":>6} | {"TP":>6} | {"FP":>7} | {"FN":>6} | {"P":>6} | {"R":>6} | {"F1":>6}')
    print('-' * 60)
    for s in sorted(stat):
        d = stat[s]
        p = d['tp'] / (d['tp'] + d['fp'] + 1e-9)
        rc = d['tp'] / (d['tp'] + d['fn'] + 1e-9)
        f1 = 2 * p * rc / (p + rc + 1e-9)
        print(f'{s:>6} | {d["tp"]:>6,} | {d["fp"]:>7,} | {d["fn"]:>6,} | '
              f'{p:>6.3f} | {rc:>6.3f} | {f1:>6.3f}')
    print(f'\nDa ghi: {out}')
    print('Neu bang tren trung evaluate.py thi ghep DUNG. Lech thi dung dung file nay.')


if __name__ == '__main__':
    main()
