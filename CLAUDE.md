# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> Ngôn ngữ: comment trong code và tài liệu bàn giao viết bằng tiếng Việt (có dấu),
> commit message viết tiếng Việt không dấu. Giữ đúng quy ước này khi sửa code.

## Cách làm việc trong repo này (quan trọng)

Chủ repo đang ở **giai đoạn học**, mục tiêu là tự hiểu từng hàm để tự phát triển module SLAM.
Vai trò mong muốn của Claude ở đây là **test + review + giảng giải**, KHÔNG phải viết hộ tính năng:

- Tìm chỗ code đang sai, chỉ rõ `file:dòng`, cơ chế gây lỗi, và hướng fix — rồi **hỏi trước
  khi sửa**. Không tự ý thay thế cả module (ví dụ: thay `ground_filter.cpp` bằng Patchwork++
  chỉ được nêu như một phát hiện, không tự implement).
- Ưu tiên **script đo đạc độc lập** (Python đọc `.bin`/`.label`) hơn là sửa code core — đó là
  cách các phiên trước đã bác bỏ 3 giả định sai về RANSAC mà không đụng vào C++.
- Kết luận sau mỗi lần kiểm tra thì **ghi vào `HANDOFF_<ngày>.md`** cho phiên sau.
- Giải thích luôn kèm ví dụ số cụ thể (bao nhiêu điểm/pixel, lệch bao nhiêu mét).

## Dự án là gì

Cài lại thuật toán **Removert** ("Remove, then Revert", Kim & Kim IROS 2020) ở mức tối giản:
tách điểm động (xe, người) khỏi điểm tĩnh trong một scan LiDAR, bằng cách so sánh
range image của scan với range image của N scan lân cận ("map").

Một **executable C++ độc lập**, KHÔNG phải ROS node (máy có ROS 2 Jazzy nhưng code không
dùng rclcpp; chỉ mượn `colcon` để build). Không có test framework — "test" ở đây là chạy
pipeline trên SemanticKITTI rồi đo Precision/Recall/F1 bằng `scripts/evaluate.py`.

## Build & chạy

```bash
cd ~/mini_removert
colcon build --packages-select mini_removert     # binary -> build/mini_removert/mini_removert
```

Package là CMake thuần (không có `package.xml`); `cmake -S . -B build_local && cmake --build build_local`
cũng chạy được nếu không muốn qua colcon. Phụ thuộc: PCL (common/io/segmentation) + Eigen.

Pipeline đầy đủ 3 bước (cấu hình đã chốt: `N=4`, `max_distance_m=4.0`, `threshold=0.5`):

```bash
# 1) chọn map_idx theo KHOẢNG CÁCH KHÔNG GIAN THẬT (mét), không theo số frame
python3 scripts/select_maps_by_distance.py \
    ~/kitti_data/dataset/poses/04.txt ~/kitti_data/dataset/sequences/04/calib.txt \
    150 4 4                                  # scan_idx N max_distance_m; map_idx nằm ở DÒNG CUỐI stdout

# 2) chạy (ghi dynamic_indices_scan<idx>.txt vào CWD)
./build/mini_removert/mini_removert \
    ~/kitti_data/dataset/sequences/04/pcd \
    ~/kitti_data/dataset/poses/04.txt \
    ~/kitti_data/dataset/sequences/04/calib.txt \
    150 0.5 148 149 151 152                  # scan_idx threshold map_idx...

# 3) đánh giá 1 scan
python3 scripts/evaluate.py \
    ~/kitti_data/dataset/sequences/04/labels/000150.label dynamic_indices_scan150.txt
```

Script chẩn đoán (mô phỏng lại pipeline bằng numpy, đọc thẳng `.bin`+`.label`, **không**
đụng C++ — dùng để kiểm chứng giả thuyết trước khi sửa code):
`scripts/diag_sign.py <scan_idx> <map_idx...>` và `scripts/diag_anti.py`.
Lưu ý mô phỏng chưa khớp C++ vì RANSAC fit ra mặt phẳng khác (F1 0.746 vs 0.623 ở scan 150)
— chỉ dùng để so sánh TƯƠNG ĐỐI giữa các luật, xem `HANDOFF_2026-08-13.md` mục 7.

Sweep (mỗi script tự lo cả 3 bước cho nhiều cấu hình, ghi `summary.txt`):
`scripts/run_n_sweep_v2.sh` (sweep N), `scripts/run_maxdist_sweep.sh` (sweep `max_distance_m`).
`run_n_sweep.sh` là bản cũ chọn map theo số frame — đã bị `_v2` thay thế, đừng dùng lại.

Dữ liệu (không nằm trong repo): `~/kitti_data/dataset/sequences/04/{pcd,labels,calib.txt}`
và `~/kitti_data/dataset/poses/04.txt`. `scripts/bin_to_pcd.py` convert `velodyne/*.bin` →
`pcd/*.pcd` (đường dẫn hard-code trong `__main__`, sửa trực tiếp khi đổi sequence).

## Kiến trúc

`main.cpp` là toàn bộ orchestration + mọi hằng số tinh chỉnh; các module dưới `src/*/`
đều là hàm thuần, không giữ trạng thái. Chỉ `pcd_dir/poses/calib/scan_idx/threshold/map_idx`
là tham số dòng lệnh — thang phân giải, `vote_threshold`, FOV dọc đều **hard-code trong
`main.cpp`**, muốn sweep thì phải sửa code hoặc viết script sinh biến thể.

Luồng cho mỗi map:

```
loadFrame (io/pcd_loader)                    pose = poses[idx] * Tr  (Tr: LiDAR->cam0, từ calib.txt)
  -> transform::transformPointCloud          scan về hệ toạ độ của map
  -> ground_filter::detectGroundMask         RANSAC 1 mặt phẳng (PCL SACMODEL_PLANE, 0.2m)
  -> range_image::buildRangeImage            projection cầu, mỗi pixel = min(range), ô trống = +inf
  -> discrepancy::computeDiscrepancy         |scan-map| > threshold ? diff : 0 ; +inf = không có data
  -> filter::getDynamicIndices / getObservedIndices   ngược từ pixel về index điểm
```

Vòng lặp **map ở NGOÀI, độ phân giải ở TRONG** (`main.cpp:85-127`) là cố ý: load PCD,
transform và ground mask không phụ thuộc độ phân giải nên dùng chung cho cả 3 thang —
3 thang chỉ tốn thêm ~40% thời gian thay vì 200%. Đừng đảo hai vòng lặp.

Tầng quyết định (`main.cpp:160-203`) — đây là phần dễ phá vỡ nhất:

- **Ground luôn static**: kiểm tra `scan_ground_mask[i]` ở tầng voting, KHÔNG chỉ dựa vào
  `exclude_mask` của `buildRangeImage`. `getDynamicIndices` duyệt mọi điểm rồi tra pixel,
  nên điểm ground vẫn dính phán quyết của pixel nó chia sẻ với vật khác (đo được 724 FP).
- **Ground mask tính MỘT LẦN ngoài vòng lặp map**: ground membership bất biến qua phép
  biến đổi cứng, tính lại mỗi map vừa thừa vừa thiếu nhất quán (RANSAC ngẫu nhiên).
- **Mẫu số voting là động**: `vote_count[li][i] / observed_count[li][i]`, không phải
  `map_indices.size()`. `observed_count == 0` → trả `false` (= không xác nhận), nghĩa là
  thiếu bằng chứng thì giữ điểm lại chứ không xoá.
- **REMOVE @ levels[0] (64×900), REVERT phải được xác nhận ở TẤT CẢ thang thô hơn**
  (32×450 và 16×225). Luật "ít nhất 1 thang" đã đo và kém hơn hẳn (F1 0.492 vs 0.630).

Sentinel cần nhớ khi sửa: range image dùng `+infinity` cho pixel trống; discrepancy image
dùng `0.0f` = static, `>0` = dynamic, `+infinity` = một trong hai ảnh không có data —
nên mọi chỗ đọc đều phải `std::isfinite()` trước.

Output ghi vào **CWD** với tên cố định `dynamic_indices_scan<scan_idx>.txt`; các script
sweep vì vậy `cd` vào thư mục kết quả rồi `mv` sang tên có tag.

## Mốc số liệu hiện tại (đừng để tụt)

seq04, 5 scan (50/100/150/200/250), cấu hình đã chốt **`N=4, max_distance_m=4.0,
threshold=1.0, vote_threshold=0.5`, 4 thang `64×900/32×450/16×225/8×112`**:
**F1 trung bình 0.720** (P≈0.71 / R≈0.74). Lịch sử: 0.275 (chỉ remove) → 0.629 (revert
3 thang, thr=0.5) → 0.691 (thr=1.0) → 0.720 (thêm thang thứ 4), xem
`HANDOFF_2026-08-13.md` mục 12 và 14.

Sau bất kỳ thay đổi nào ở tầng quyết định, chạy lại đủ 5 scan này rồi so — một scan đơn lẻ
(nhất là 150) không đủ kết luận. **Và nếu thay đổi có đụng tới ground segmentation thì một
lần chạy cũng không đủ**: F1 dao động ±0.05 chỉ vì RANSAC fit ra mặt phẳng khác, phải đo
trên nhiều mask (`scripts/diag_seed.py`), xem mục 11.

> **0.720 KHÔNG phải tính chất của thuật toán** mà của *(seq04 + đúng 5 scan đó + đúng
> ground mask đó)*. Trên seq 03/06/07 (đã rút sẵn về `~/kitti_data`) cùng cấu hình chỉ ra
> **0.145-0.495**; ngay trong seq04, đổi 5 scan đem đo đã lệch 0.071. Ba nguồn phương sai:
> ground mask ±0.05, chọn scan ±0.07, chọn sequence ±0.25 (`HANDOFF_2026-08-13.md` mục 16).
>
> Hệ quả thực dụng: **so sánh có cặp** (mọi nhánh dùng chung sequence/scan/mask) thì tin
> được; **con số tuyệt đối thì không**. Đo cải tiến mới nên chạy trên ít nhất 2 sequence —
> `scripts/run_generalization_test.py`.

FP còn lại chủ yếu là ground: road 40.2% + sidewalk 18.7% + terrain 8.6% ≈ 67.5%.
Ưu tiên tiếp theo (theo `HANDOFF_2026-08-12.md` mục 7): (1) thay `ground_filter.cpp` bằng
Patchwork++, (2) sweep `threshold`/`vote_threshold`, (3) sweep cấu hình thang phân giải,
(4) test trên sequence khác, (5) map tích luỹ thay vì 1 scan thô.

## Tài liệu bàn giao

`HANDOFF_2026-08-12.md` là trạng thái hiện tại (đọc file này trước). `HANDOFF_VIEC4.md`
là bối cảnh sâu hơn của các phiên trước, vẫn còn giá trị, không bị thay thế. Cả hai chứa
những kết luận đã ĐO và đã BÁC BỎ (ví dụ: threshold thích ứng theo range — đã gạch, vì
điểm động thật ở xa hơn điểm báo nhầm) — kiểm tra ở đó trước khi đề xuất lại một hướng.

Nguyên lý lặp lại 2 lần trong dự án, đáng nhớ: **gộp thêm bằng chứng (voting đa frame,
revert đa thang) chỉ triệt tiêu sai số NGẪU NHIÊN; sai số HỆ THỐNG (parallax do baseline
xa, lệch mặt đường) phải sửa tận gốc.**

Thư mục `results_*/`, `build/`, `install/`, `log/` đều nằm trong `.gitignore` — kết quả
sweep tái tạo được bằng script, không commit.
