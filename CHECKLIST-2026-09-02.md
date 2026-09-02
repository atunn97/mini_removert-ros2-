# ✅ CHECKLIST HÔM NAY — 02/09/2026 (soát máy lúc 13:11)

> Kiểm bằng lệnh trên máy Linux, không phải nhớ lại. Việc của hôm nay **không đổi**:
> đóng câu hỏi mục 15.9 — *FP đến từ LUẬT QUYẾT ĐỊNH hay từ ĐƯỜNG ỐNG?*
> Quy trình chi tiết: `CHECKLIST-15.9-FP.md`. Bối cảnh: `START_HERE.md`.

---

## 0. Ta đang có gì trong tay

| Thứ | Trạng thái | Ghi chú |
|---|---|---|
| Repo `~/mini_removert` | ✅ **sạch**, không file sửa dở | HEAD = `3dcab38` *"Ham xuat CSV cho tang hoc may…"* |
| Binary `build/mini_removert/mini_removert` | ✅ có, build 01/09 17:55 | **khỏi `colcon build`**, chạy thẳng được |
| KITTI seq04 | ✅ 271 file `.pcd` + `labels/` + `poses/04.txt` + `calib.txt` | |
| `map_idx` cho scan 150 | ✅ **đã chạy sẵn sáng nay** | `136 137 138 139 140 160 161 162 163` — bậc 3 (R=20, t_min=10, **9 nguồn**) |
| `dynamic_indices_scan{150,349,939}.txt` | ✅ còn nguyên từ 01/09 | trong `~/mini_removert/` |
| **`scan150.csv` / `scan150_gt.csv`** | ✅ **đã dựng lại 02/09 13:13** | ba mốc + TP/FP/FN khớp nguyên bản 01/09 (xem cuối file) |
| `pandas` | ⚠️ **không có** ở `python3` hệ thống | có ở **`~/pointcloud_env/bin/python`** (pandas 3.0.3). `add_gt_label.py` chỉ cần `numpy` nên `python3` thường vẫn chạy được |

> 📌 **Sai lệch nhỏ trong `START_HERE.md`:** khối HÔM NAY ghi commit mới nhất là `cec4f2c`,
> `8f0043c`. Thực tế còn `3dcab38` (hàm xuất CSV) nằm trên. Không mất gì — chỉ là ghi thiếu,
> sửa luôn khi cập nhật cuối phiên.

---

## 1. Đã kiểm hộ: mối nghi về cột `vote_*` ở BẢNG 2 — **giải toả**

`CHECKLIST-15.9-FP.md` dặn *"mở code xem `vote_*` được ghi ở đâu trước khi tin bảng này"*.
Đã mở (`src/main.cpp`):

- **`vote_L{i}` và `observed_L{i}` là con ĐẾM, không phải cờ 0/1** (khai báo dòng 136–140,
  tăng ở 293–295). Nhưng sau việc 4 map đã gộp còn **một** ⇒ `observed ∈ {0,1}` ⇒ `vote ∈ {0,1}`.
- **`L0` = thang mịn nhất = bước REMOVE** (dòng 366). Ứng viên = điểm **không ground** và
  `is_dynamic_at(0,·)`; với đếm 0/1 thì điều đó **đúng bằng `vote_L0 == 1`** trên `d` (`d` đã bỏ ground).
  ⇒ **Khối lọc ứng viên trong bảng 2 dùng được như đã viết.**
- **REVERT đòi MỌI thang `L1..L3` xác nhận** (dòng 374) ⇒ cột `so_thang` có nghĩa thật, không phải trang trí.

- [x] **Đã chạy 02/09 — `vote > observed` ra `0`, `vote max = observed max = 1`. Bảng 2 dùng được.**
  ```python
  print('vote > observed:', (d[['vote_L0','vote_L1','vote_L2','vote_L3']].values >
                             d[['observed_L0','observed_L1','observed_L2','observed_L3']].values).sum())  # phai = 0
  ```
  `dynamic_indices` và `observed_indices` do **hai hàm riêng** tính (dòng 286–291); chưa có gì
  trong code *bắt buộc* dynamic ⊆ observed. Ra khác 0 ⇒ dừng, đọc lại `getObservedIndices`.

---

## 2. Việc hôm nay, đúng thứ tự

- [x] **B0 — dựng lại CSV** ✅ *(xong 02/09 13:13)* (~vài phút, `map_idx` đã có sẵn ở trên)
  ```bash
  cd ~/mini_removert && ./build/mini_removert/mini_removert \
      ~/kitti_data/dataset/sequences/04/pcd ~/kitti_data/dataset/poses/04.txt \
      ~/kitti_data/dataset/sequences/04/calib.txt 150 0.5 \
      136 137 138 139 140 160 161 162 163 --csv scan150.csv
  ```
  ```bash
  cd ~/mini_removert && python3 scripts/add_gt_label.py scan150.csv 04
  ```

- [x] **B0.5 — ba mốc ĐÃ KHỚP HẾT:** số dòng **126.941** · `is_ground.sum()` **63.156** ·
  `baseline_label.sum()` **1.445** · TP/FP/FN **301 / 1.144 / 69**.
  Log lúc chạy cũng phải cho `observed_count min=0 max=1` và gộp **1.141.089 → 118.574**.

- [ ] **B1–B3 — ba bảng.** Code có sẵn trong `CHECKLIST-15.9-FP.md`, chạy nguyên si, **đổi interpreter**:
  ```bash
  ~/pointcloud_env/bin/python
  ```

- [ ] **Chốt nhánh ngay cuối phiên** (đường ống ⇒ vá vào baseline rồi đo lại; luật quyết định ⇒ đi thẳng bước 3–4).

- [ ] **Ghi lại (10 phút):** ba bảng số → mục 16 của `HANDOFF_2026-09-01.md`; một câu kết luận;
  cập nhật khối HÔM NAY trong `START_HERE.md` (**kèm sửa dòng commit thành `3dcab38`**).

### ⛔ Không làm hôm nay
bag `standstill_*` (15.2 — nửa buổi riêng) · bước 3 gộp nhiều scan · **đụng C++** · **tune ngưỡng**.

---

## 3. Chỗ đứng trong lịch 4 tuần

**Tuần 1** = 15.9 (hôm nay) + nửa buổi standstill + bước 3 → *ra: baseline đáng tin*.
Hôm nay **02/09**, hạn nộp đề tài NCKH sau 4 tuần; **M3 môn DS trước 06/09** đã trừ hao trong lịch.


---

## 4. Kết quả bước 0 — chạy lúc 02/09 13:13, **khớp hoàn toàn phiên 01/09**

| Mốc | Phải ra | Ra thật |
|---|---|---|
| số dòng | 126.941 | ✅ 126.941 |
| `is_ground.sum()` | 63.156 (49,7522%) | ✅ 63.156 |
| `baseline_label.sum()` | 1.445 | ✅ 1.445 |
| TP / FP / FN | 301 / 1.144 / 69 | ✅ 301 / 1.144 / 69 — P 0,208 · R 0,814 · F1 0,332 |
| map gộp | 1.141.089 → 118.574 | ✅ khớp |
| `observed_count` | min=0 max=1 | ✅ mọi thang |
| ứng viên sau REMOVE | 13.809 | ✅ 13.809 · REVERT 12.364 (89,54%) |

Thêm hai số đã kiểm trên chính CSV:
- `vote > observed` = **0** trên cả 4 thang, `vote max = observed max = 1` ⇒ **giả định của bảng 2 đúng**.
- Bảng chéo trên phần **không ground**: TN 62.275 · FP 1.144 · TP 301 · **FN 65** — đúng như dự đoán,
  4 FN còn lại nằm trong ground (luật ground nuốt oan 4 điểm động thật, mục 14.4).
- `gt_label.sum()` = **370** điểm động thật trong scan 150.

**File sẵn sàng:** `~/mini_removert/scan150_gt.csv` (14,9 MB, 33 cột). Vào thẳng BẢNG 1 được.
