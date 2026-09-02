# ✅ CHECKLIST — Khám nghiệm FP bằng CSV (mục 15.9)

> Mở file này là vào việc được ngay, không cần đọc lại handoff 824 dòng.
> Đọc `START_HERE.md` nếu quên bối cảnh. Chi tiết đầy đủ: `HANDOFF_2026-09-01.md` mục 13–15.

**Câu hỏi duy nhất phải đóng hôm nay:**

> ### FP đến từ **LUẬT QUYẾT ĐỊNH** hay từ **ĐƯỜNG ỐNG**?

**Vì sao câu này quyết định bài đứng hay đổ:** MLP ăn *cùng* bộ đặc trưng do *cùng* đường ống
sinh ra. Nếu FP dồn ở vùng xa, MLP sẽ học đúng một luật `range > 30 ⇒ tĩnh`, precision nhảy vọt,
bảng rất đẹp — nhưng cái nó học **không phải phân biệt động/tĩnh, mà là một cái cổng khoảng cách**.
Hội đồng hỏi *"vậy chỉ cần cắt bán kính thì baseline cũng lên, phải không?"* là sập trong một câu.

⛔ **GIỚI HẠN CỨNG: MỘT PHIÊN.** Hết phiên là chốt nhánh và đi tiếp, không mở ngỏ.
⛔ **KHÔNG đụng C++ trong phiên này.** Chỉ đọc CSV bằng pandas.
⛔ **KHÔNG tune ngưỡng.** Ngưỡng cứng CHÍNH LÀ baseline của đề tài.

---

## ☐ Bước 0 — Tiền kiểm (5 phút)

- [ ] Có file `scan150_gt.csv` chưa? Chưa thì dựng lại:

```bash
./build/mini_removert/mini_removert ~/kitti_data/dataset/sequences/04/pcd \
    ~/kitti_data/dataset/poses/04.txt ~/kitti_data/dataset/sequences/04/calib.txt \
    150 0.5 <map_idx...> --csv scan150.csv
python3 scripts/add_gt_label.py scan150.csv 04      # -> scan150_gt.csv
```

- [ ] **Ba mốc phải khớp, sai là DỪNG** (số của phiên 01/09):

| Kiểm | Phải ra |
|---|---|
| số dòng | **126.941** |
| `is_ground.sum()` | **63.156** |
| `baseline_label.sum()` | **1.445** |
| TP / FP / FN | **301 / 1.144 / 69** |

Lệch bất kỳ ô nào ⇒ CSV không phải bản đã kiểm chéo, **đừng chạy ba bảng dưới**.

---

## ☐ Khối nạp — chạy một lần, dùng cho cả ba bảng

```python
import pandas as pd
df = pd.read_csv('scan150_gt.csv')

# Tach rieng diem ground: luat ground da nuot oan 4 diem dong that (muc 14.4),
# giu lai de doi chieu nhung khong tron vao ba bang duoi.
d = df[df['is_ground'] == 0].copy()

d['ket_qua'] = 'TN'
d.loc[(d['baseline_label'] == 1) & (d['gt_label'] == 1), 'ket_qua'] = 'TP'
d.loc[(d['baseline_label'] == 1) & (d['gt_label'] == 0), 'ket_qua'] = 'FP'
d.loc[(d['baseline_label'] == 0) & (d['gt_label'] == 1), 'ket_qua'] = 'FN'

print(d['ket_qua'].value_counts())   # phai ra TP=301  FP=1144  FN=65  (FN 4 con lai nam o ground)
```

---

## ☐ BẢNG 1 — FP theo vành đai 10 m

```python
d['vanh_dai'] = (d['range'] // 10 * 10).astype(int)
b1 = d.pivot_table(index='vanh_dai', columns='ket_qua',
                   values='point_id', aggfunc='count', fill_value=0)
for c in ('TP', 'FP'):
    if c in b1: b1[c + '_luy_tich_%'] = (100 * b1[c].cumsum() / b1[c].sum()).round(1)
print(b1)
```

**Đọc ra gì:**

| Thấy | Kết luận |
|---|---|
| **>70% FP nằm ngoài 30 m** mà TP dồn trong 30 m | nghi phạm mục 13.4 ĐÚNG ⇒ **ĐƯỜNG ỐNG** |
| FP rải đều theo cự ly, tỉ lệ giống TP | không phải chuyện cự ly ⇒ nghiêng về **LUẬT QUYẾT ĐỊNH** |

> Bối cảnh mục 13.4: ngoài 30 m, scan có **5.783** điểm còn map có **38.869** — gấp **6,7 lần**.

---

## ☐ BẢNG 2 — ⭐ BẢNG QUYẾT ĐỊNH: ứng viên theo số thang bỏ phiếu

Chạy trên **13.809 ứng viên**, không phải 1.445 điểm cuối — đó mới là chỗ tỉ lệ REVERT 89,5%
sáng nghĩa.

```python
ung_vien = d[d['vote_L0'] == 1].copy()
print('so ung vien =', len(ung_vien), '(mong doi ~13.809)')   # <-- LECH NHIEU thi DUNG, xem canh bao duoi

ung_vien['so_thang'] = ung_vien[['vote_L0','vote_L1','vote_L2','vote_L3']].sum(axis=1)
b2 = ung_vien.pivot_table(index='so_thang', columns='gt_label',
                          values='point_id', aggfunc='count', fill_value=0)
b2.columns = ['tinh_gt0', 'dong_gt1'][:len(b2.columns)]
b2['ty_le_dong_%'] = (100 * b2['dong_gt1'] / (b2['tinh_gt0'] + b2['dong_gt1'])).round(2)
print(b2)
```

⚠️ **Nếu `so_thang` của mọi ứng viên đều = 4, hoặc số ứng viên lệch xa 13.809** thì giả định
"`vote_L0`=1 ⇔ ứng viên" sai so với code. **Mở code xem `vote_*` được ghi ở đâu trước khi tin
bảng này** — đừng diễn giải một bảng mà mình chưa chắc nghĩa của cột.

**Đọc ra gì:**

| Thấy | Kết luận |
|---|---|
| `ty_le_dong_%` **thấp đều ở cả 4 mức** số thang | lệch **nhất quán** qua mọi phân giải ⇒ hình học ⇒ **ĐƯỜNG ỐNG** |
| `ty_le_dong_%` **tăng rõ theo số thang** | cơ chế revert đang làm đúng việc; FP còn lại là nhiễu quanh ngưỡng ⇒ **LUẬT QUYẾT ĐỊNH** |

---

## ☐ BẢNG 3 — Dấu của `range_diff` ở FP

```python
for L in range(4):
    c = f'range_diff_L{L}'
    fp = d.loc[d['ket_qua'] == 'FP', c].dropna()
    tp = d.loc[d['ket_qua'] == 'TP', c].dropna()
    print(f'L{L}  FP: n={len(fp):5d} duong={100*(fp>0).mean():5.1f}% trung_vi={fp.median():+.3f}'
          f'  |  TP: n={len(tp):4d} duong={100*(tp>0).mean():5.1f}% trung_vi={tp.median():+.3f}')
```

> Quy ước dấu (mục 14.1): **dương** = vật gần hơn bản đồ (nghi động) · **âm** = xa hơn
> (che khuất / bản đồ thiếu dữ liệu).

| Thấy | Kết luận |
|---|---|
| FP lệch hẳn về **dương**, trung vị xa 0, giống TP | che khuất / parallax ⇒ **ĐƯỜNG ỐNG** |
| FP **đối xứng quanh 0**, trung vị sát ngưỡng | nhiễu quanh ngưỡng ⇒ **LUẬT QUYẾT ĐỊNH** |

---

## ☐ CHỐT NHÁNH — làm ngay cuối phiên, đừng để sang hôm sau

### → Ra **ĐƯỜNG ỐNG**

- [ ] Vá đúng chỗ đó **VÀO BASELINE** (giới hạn bán kính, hoặc đòi mật độ map tại pixel).
- [ ] Đo lại ba mốc P/R/F1.
- [ ] Đi bước 3 (chạy nhiều scan rồi nối).

> Mất cái "gain" giả, nhưng phần còn lại là thật. **Baseline mạnh lên là bài KHOẺ hơn, không
> phải yếu đi** — vì MLP học được mấy cái vá đó miễn phí, không vá thì "thắng" của MLP là giả.

### → Ra **LUẬT QUYẾT ĐỊNH**

- [ ] Hết việc đào. Đi thẳng bước 3 → bước 4 (MLP numpy).
- [ ] Câu chuyện đã có sẵn: *"ngưỡng cố định không sống nổi khi mật độ lấy mẫu đổi theo cự ly."*

### Ba điều kiện tuyên bố "baseline đủ tử tế, dừng đào"

- [ ] Mỗi cụm FP lớn **gọi tên được** thuộc đường ống hay luật quyết định — cần *biết*, không cần sửa hết.
- [ ] Mọi cái vá hiển nhiên mà người cài Removert tử tế đương nhiên làm (bán kính, ground, voxel) đã nằm trong baseline.
- [ ] **Ổn định và giải thích được, không cần đẹp.** P 0,2 kèm câu chuyện có bằng chứng thì đăng được; P 0,2 kèm *"chưa rõ vì sao"* thì không.

---

## ☐ Trước khi tắt máy — ghi lại (10 phút)

- [ ] Dán **cả ba bảng số** vào `HANDOFF_2026-09-01.md` thành mục 16.
- [ ] Ghi **một câu** kết luận: đường ống hay luật quyết định.
- [ ] Cập nhật khối HÔM NAY trong `START_HERE.md`.
- [ ] Xoá dòng "câu hỏi mở duy nhất" nếu đã đóng được.

---

## 🃏 Nửa buổi phụ — bag `standstill_*` (mục 15.2) · KHÔNG làm chung phiên này

Cảm biến đứng yên ⇒ pose = ma trận đơn vị (khỏi SLAM) và nhãn lấy được bằng **trung vị range
theo thời gian** (khỏi gán tay). Câu hỏi kiểm duy nhất: **trong bag có vật động đi ngang qua không?**

Có ⇒ Go2 từ 2 tháng xuống vài ngày. Đây là quân bài **rẻ nhất chưa lật**, nhưng nhét chung một
phiên với ba bảng trên là hỏng cả hai.

---

## ❌ Đã bác bỏ bằng số — đừng đi lại

- "hai mặt phẳng RANSAC riêng gây FP" — đã hợp nhất ở việc 3, **số không đổi**
- "mặt đường cong ra khỏi mặt phẳng" — `R` ra **ÂM** ở seq06, `R²` = 0,26, vách dự báo 76 m mà vách thật ở 10–20 m
- ⭐ **bỏ `fabs()` ở `discrepancy.cpp:57` (luật một chiều)** — **BÁC BỎ 13/08**, đo lại 14/08.
  Luật SIGNED bắt **0/370** điểm động ở thang thô, recall sập về 0, AUC 0,322 (tệ hơn đoán bừa).
  Lý do: map tích luỹ chứa **vệt của chính vật động** — nguồn quá khứ cho dấu âm, nguồn tương lai
  cho dấu dương, luật một chiều tự cắt bỏ một nửa bằng chứng. Xem `HANDOFF_2026-08-13.md` mục 2–3.
  *Điều kiện mở lại (do chính mục đó đặt ra): chỉ khi nguồn được đảm bảo KHÔNG chứa vật.*
- chênh lệch ground 49,8% (scan) vs 38,3% (map) — **lành tính**, chỉ do phân bố bán kính khác nhau (mục 13.2)
