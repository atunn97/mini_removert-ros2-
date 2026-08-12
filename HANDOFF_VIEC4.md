# Việc 4 — Báo cáo bàn giao đầy đủ: fix multi-frame voting bug trong mini_removert

> File này tổng hợp TOÀN BỘ quá trình làm việc, bao gồm cả những trao đổi/quyết
> định KHÔNG nằm trong code hay trong `NOTES_viec4_N_sweep_debug.md` (vốn chỉ tập
> trung vào debug số liệu). Đọc file này để nhớ lại được TOÀN BỘ bối cảnh, kể cả
> lý do đằng sau mỗi quyết định.

---

## 1. Bối cảnh & phạm vi gốc của Việc 4

Bug cần fix: hàm voting đa số trong `main.cpp` dùng mẫu số CỐ ĐỊNH
(`map_indices.size()`) để tính ngưỡng "điểm này có phải dynamic không", trong
khi mẫu số ĐÚNG phải là **số map thực sự quan sát được điểm đó** (nhiều điểm nằm
ngoài FOV hoặc bị occlusion ở một số map, không phải map nào cũng "thấy" được
mọi điểm).

Kiến trúc đã thống nhất trước khi code (không đổi trong suốt phiên làm việc):
- `range_image.hpp/cpp`: thêm `projectToPixel()` dùng chung, tách từ logic có
  sẵn trong `buildRangeImage` — hành vi giữ nguyên 100%.
- `filter.hpp/cpp`: thêm `filter::getObservedIndices()` — trả về index các điểm
  mà 1 map cụ thể THỰC SỰ quan sát được (`map_image != infinity`, check bằng
  `std::isfinite()` đúng sentinel thật của `buildRangeImage`).
- `main.cpp`: thêm `observed_count` song song `vote_count`, bỏ
  `min_votes = map_indices.size()/2 + 1` cố định, thay bằng
  `ratio = vote_count[i] / observed_count[i] > 0.5`, bỏ qua (không loại) điểm có
  `observed_count == 0`.

Ngoài phạm vi Việc 4 (không đụng vào): `discrepancy.cpp` — quyết định giữ
nguyên trong suốt phiên, kể cả sau khi phát hiện vấn đề lớn hơn (xem mục 4).

---

## 2. Đã hoàn thành (tóm tắt theo checklist gốc)

- [x] Áp code vào đúng 5 file thật trong repo (4 file core + main.cpp)
- [x] `colcon build` sạch, không lỗi biên dịch
- [x] Sanity-check N=2: `observed_count` phân bố hợp lý (không toàn 0/toàn N)
- [x] Viết script loop N=2,4,6,8,10, tự động chạy + evaluate + gom bảng F1
- [x] Chạy evaluate.py từng N, xác nhận **F1 tăng đơn điệu theo N** (sau khi fix
      thêm 1 bug về cách chọn map — xem mục 4-5)
- [ ] Push code lên `atunn97/mini_removert-ros2-` — **đang ở bước này, chưa
      xong** (xem mục 8)

---

## 3. Các trục trặc gặp phải trong lúc code (không phải bug logic, nhưng tốn thời gian)

Ghi lại để tránh lặp lại lần sau:

### 3.1. Mất code khi tự tay sửa file
Khi tự sửa `main.cpp` và `filter.cpp` theo hướng dẫn, bạn vô tình **xoá mất**
phần code cũ (`scan_frame` declaration trong `main.cpp`; `getDynamicIndices` và
`splitCloud` trong `filter.cpp`) — gây lỗi biên dịch/linker (`undefined
reference`). Bài học: khi sửa file bằng copy-paste thủ công, nên **ghi đè toàn
bộ file** bằng bản đầy đủ thay vì chỉnh từng đoạn nhỏ, để tránh mất code không
liên quan.

### 3.2. Hiểu lầm về cấu trúc thư mục `colcon build`
Thắc mắc: sao `colcon build` tạo `build/mini_removert/mini_removert` (lồng
folder trùng tên) — tưởng nhầm là mỗi lần build sinh thêm folder mới.
**Giải thích:** đây là cấu trúc CHUẨN của colcon — `build/<tên_package>/` (từ
`package.xml`) chứa `<tên_executable>` (từ `add_executable` trong
`CMakeLists.txt`). Trùng tên vì package và executable cùng tên `mini_removert`.
Không sinh thêm folder mỗi lần build — chỉ ghi đè file bên trong. Trước đó bạn
từng build bằng `cmake`/`make` thuần (không phải colcon), tạo ra file thực thi
`build/mini_removert` trực tiếp (không phải folder) → xung đột tên khi
`colcon build` sau đó muốn tạo folder cùng tên → phải `rm -rf build install log`
rồi build lại.

---

## 4. Phát hiện lớn nhất trong phiên: precision cực thấp sau khi fix bug gốc

Sau khi code chạy đúng kỹ thuật (không lỗi, sanity-check pass), chạy N-sweep lần
đầu (map chọn theo SỐ FRAME cách đều, window ±60 quanh scan_idx=150) cho kết quả
**bất thường nghiêm trọng**:

| N | Precision | Recall | F1 |
|---|---|---|---|
| 2 | 0.007 | 0.986 | 0.015 |
| 10 | 0.006 | 1.000 | 0.012 |

Predicted dynamic chiếm **38-49% tổng số điểm** trong khi GT dynamic thật chỉ
**0.29%** — thuật toán gần như đánh dấu "dynamic" bừa bãi.

### Root cause (đã giải thích kỹ, đây là phần quan trọng nhất cần nhớ)

`discrepancy.cpp::computeDiscrepancy` so sánh `scan_image[row][col]` với
`map_image[row][col]` tại CÙNG 1 pixel, giả định 2 pixel này chiếu tới CÙNG 1 bề
mặt vật lý ngoài đời thực. **Giả định này chỉ đúng khi 2 viewpoint (vị trí LiDAR
lúc quét scan và lúc quét map) gần nhau.**

Đo bằng `poses.txt` thật → baseline giữa scan_idx=150 và các map đã dùng là
**28-91 MÉT** (không phải vài mét như tưởng). Ở khoảng cách này, do
**PARALLAX** (thị sai hình học) và **thay đổi OCCLUSION** (che khuất), cùng 1
pixel `(row,col)` chiếu tới 2 bề mặt vật lý HOÀN TOÀN KHÁC NHAU dù cảnh vật hoàn
toàn tĩnh → `diff` giữa 2 range vượt xa `threshold=0.5m` dù không có gì di
chuyển thật.

`min()` trong `buildRangeImage` (giữ điểm gần nhất mỗi bin) khuếch đại lỗi này
thêm ở các mép/góc khuất vật thể — càng nhiều cạnh trong scene (nhà, cột, vỉa
hè), càng nhiều false positive.

**Tại sao voting đa số KHÔNG lọc được lỗi này:** lỗi này không phải nhiễu ngẫu
nhiên độc lập giữa các map (thứ mà voting đa số triệt tiêu tốt) — nó **tương
quan hệ thống** với baseline lớn, xảy ra ở HẦU HẾT các cặp scan-map có baseline
xa, bất kể chọn map nào. Vì vậy thêm map (tăng N) không giúp gì, thậm chí nhiều
map "đồng thuận sai" cùng lúc.

### Phát hiện phụ: seq04 không có loop closure
Xe trong seq04 di chuyển liên tục ~1.4-1.5 m/frame, không quay lại vị trí cũ.
→ Baseline luôn tỉ lệ thuận số frame cách nhau. **Không có cách nào chọn N lớn
mà vẫn giữ baseline nhỏ chỉ bằng cách né frame xa** — buộc phải đổi hẳn tiêu chí
chọn map.

---

## 5. Fix: chọn map theo khoảng cách không gian THẬT, không theo số frame

### Công cụ mới đã tạo
- `scripts/check_baseline_distance.py` — công cụ chẩn đoán, đo khoảng cách thật
  (mét) giữa scan_idx và các map_idx đã chọn (dùng đúng công thức
  `poses[idx] @ Tr` giống hệt C++ trong `main.cpp`/`pose_loader.hpp`/
  `calib_loader.hpp`).
- `scripts/select_maps_by_distance.py` — công cụ CHỌN map tự động, cách đều
  theo khoảng cách thật, trong 1 bán kính an toàn (`max_distance_m`) do bạn đặt.
  Dùng lại được cho MỌI N-sweep sau này, MỌI sequence (kể cả đường cong, không
  chỉ đường thẳng như seq04) — vì dùng khoảng cách 3D thật, không phụ thuộc số
  frame.
- `scripts/run_n_sweep_v2.sh` — script tổng hợp: tự gọi
  `select_maps_by_distance.py` lấy map_idx cho từng N, chạy `mini_removert`,
  chạy `evaluate.py`, gom bảng.

### Kết quả sau fix (scan_idx=150, max_distance_m=8.0, seq04)

| N | Precision | Recall | F1 | map_idx |
|---|---|---|---|---|
| 2 | 0.049 | 0.978 | 0.093 | 145 155 |
| 4 | 0.057 | 0.989 | 0.108 | 145 147 153 155 |
| 6 | 0.069 | 0.989 | 0.129 | 145 146 148 152 154 155 |
| 8 | 0.080 | 0.989 | 0.148 | 145 146 147 149 151 153 154 155 |
| 10 | 0.090 | 0.989 | 0.164 | 145 146 147 148 149 151 152 153 154 155 |

**F1 tăng đơn điệu theo N** — đúng giả thuyết ban đầu, xác nhận bug voting đã
fix đúng, và root cause precision-thấp nằm ở tầng chọn map (đã fix), không phải
ở logic voting mới.

**Lưu ý còn tồn đọng:** precision vẫn thấp về mặt tuyệt đối (0.09 ở N=10, so với
mốc F1=0.226 đạt được trước đây với cấu hình khác) — có thể cần thử
`max_distance_m` nhỏ hơn nữa, hoặc quay lại cân nhắc threshold thích ứng trong
`discrepancy.cpp` (xem mục 4, quyết định CHƯA sửa file này trong phiên này).

---

## 6. Giải đáp thắc mắc: vì sao chọn map theo "khoảng cách", không dùng "độ cao" (elevation map)

Bạn có đọc tài liệu và thấy nhiều nơi dùng "độ cao" thay vì "khoảng cách" để
mapping nhanh hơn — thắc mắc tại sao ở đây lại làm ngược lại. Giải đáp:

**"Chọn map theo khoảng cách"** ở đây là bài toán **keyframe/reference-frame
selection** — chỉ so sánh các ma trận pose 4x4 (vị trí robot) đã có sẵn, CỰC
NHẸ (271 pose, tính bằng mili-giây), KHÔNG động vào point cloud. Đây là kỹ thuật
chuẩn trong SLAM (giống keyframe selection của ORB-SLAM), không liên quan gì
tới khối lượng dữ liệu point cloud khổng lồ.

**"Bản đồ độ cao" (elevation map)** là kỹ thuật **biểu diễn bản đồ** khác hẳn —
lưới 2.5D, mỗi ô chỉ lưu 1 giá trị độ cao, dùng phổ biến cho robot 4 chân
(foothold planning — đặt chân ở đâu để không vấp, chính là bài toán cho Go2).
Nó **mất thông tin theo chiều thẳng đứng** tại mỗi ô → không phù hợp cho
Removert (cần phân biệt "điểm 3D này còn tồn tại hay không" cho TỪNG điểm riêng
lẻ, không thể gộp thành 1 độ cao đại diện).

→ 2 kỹ thuật giải quyết 2 bài toán khác nhau, không thay thế nhau được. Có thể
tồn tại song song trong hệ thống lifelong SLAM của bạn (change detection dùng
range-image + keyframe selection; terrain/foothold planning dùng elevation map).

---

## 7. Cân nhắc kiến trúc real-time (khi tích hợp Removert song song FAST-LIO2)

Câu hỏi đặt ra: N-sweep chạy có độ trễ — liệu sau này chạy Removert song song
FAST-LIO2 trên robot thật có ảnh hưởng tới việc FAST-LIO2 ghi map không?

### Latency đo được thật (laptop, không phải Jetson)
| N | thời gian (real) |
|---|---|
| 2 | 0.461s |
| 10 | 2.089s |

~0.2s/map, gần tuyến tính. Jetson trên Go2 là **Orin NX** (8-core ARM
Cortex-A78AE, ~2.0GHz) — mạnh hơn Xavier NX, nhưng `mini_removert` hiện chạy
ĐƠN LUỒNG nên vẫn phụ thuộc hiệu năng single-core, nơi ARM yếu hơn CPU laptop
x86. Ước tính: chậm ~2-4 lần so với laptop → N=10 trên Orin NX ~4-8 giây. Vẫn
**không thể chạy ở tần số LiDAR (10Hz = 100ms/frame)** dù N=2.

### 2 rủi ro cần tránh khi tích hợp song song
1. **Ghép nối đồng bộ (blocking)** — FAST-LIO2 phải đợi Removert: TUYỆT ĐỐI
   TRÁNH. Phải chạy như 2 node/process độc lập, giao tiếp bất đồng bộ qua topic.
2. **Tranh giành CPU trên Jetson** — dù chạy async, Removert ngốn CPU rảnh vẫn
   có thể làm FAST-LIO2 trễ/rớt scan. Cần: hàng đợi giới hạn (drop-oldest),
   throttle tần số Removert (không mỗi frame, chạy thưa), CPU
   affinity/priority ưu tiên FAST-LIO2 (`taskset`/`chrt`).

### Đề xuất kiến trúc: offboard compute sang laptop
FAST-LIO2 (real-time, tight IMU-LiDAR coupling) PHẢI giữ on-board Jetson. Removert
(không cần phản hồi tức thời) là ứng viên tốt để offboard sang laptop:
- Jetson: FAST-LIO2 như cũ + định kỳ (throttle) gửi scan qua WiFi/4G (dùng lại
  hạ tầng dual-NIC + WebRTC đã xây cho dự án dance sync).
- Laptop: chạy Removert không giới hạn tài nguyên Jetson, trả kết quả nhẹ
  (index điểm dynamic, vài KB) về nếu cần.
- Băng thông: point cloud thô ~1.5MB/scan — KHÔNG stream liên tục 10Hz (quá
  nặng qua 4G), chỉ gửi khi throttle trigger — khớp tự nhiên với khuyến nghị
  throttle ở trên.

### Việc cần làm khi bắt tay tích hợp thật (không phải hôm nay)
- Đo lại latency THẬT trên Jetson Orin NX (không chỉ ước tính).
- Thiết kế giao thức truyền dữ liệu Jetson↔laptop.
- Trao đổi với người phụ trách stream A (SLAM frontend) vì ảnh hưởng cách
  FAST-LIO2 publish dữ liệu.

---

## 8. Trạng thái Git — CHƯA push xong, đang ở bước này

Lần chạy `git add -A -n` gần nhất phát hiện 2 vấn đề cần xử lý trước khi commit:
1. `results_n_sweep/` và `results_n_sweep_v2/` (output N-sweep) bị lọt vào danh
   sách add — đây là kết quả tái tạo được bằng script, KHÔNG nên commit. Cần
   thêm vào `.gitignore` trước.
2. `NOTES_viec4_N_sweep_debug.md` và phần sửa `.gitignore` (thêm `install/`,
   `log/`) KHÔNG xuất hiện trong danh sách add — nghĩa là 2 file/thay đổi này
   **có thể chưa được tạo/lưu thật trên máy bạn** dù đã có nội dung đầy đủ ở
   các tin nhắn trước.

**Việc cần làm tiếp ngay:**
1. Xác nhận đã tạo `NOTES_viec4_N_sweep_debug.md` với nội dung đầy đủ (nội dung
   gốc đã đưa ở tin nhắn trước trong hội thoại).
2. Xác nhận `.gitignore` đã có `install/`, `log/`, và thêm
   `results_n_sweep/`, `results_n_sweep_v2/`.
3. Chạy lại `git add -A -n` — danh sách sạch phải gồm: 5 file code
   (`filter.hpp/cpp`, `range_image.hpp/cpp`, `main.cpp`), 4 script
   (`run_n_sweep.sh`, `run_n_sweep_v2.sh`, `check_baseline_distance.py`,
   `select_maps_by_distance.py`), `NOTES_viec4_N_sweep_debug.md`,
   `HANDOFF_VIEC4.md` (file này), `.gitignore` — KHÔNG có `results_n_sweep*/`.
4. `git add -A` → `git commit` (message mẫu đã đưa ở tin nhắn trước) → `git push`.

---

## 9. Danh sách đầy đủ file đã tạo/sửa trong phiên này

**Sửa (theo đúng thiết kế gốc Việc 4):**
- `include/range_image/range_image.hpp` — thêm `PixelCoord`, khai báo `projectToPixel`
- `src/range_image/range_image.cpp` — implement `projectToPixel`, refactor `buildRangeImage`
- `include/filter/filter.hpp` — khai báo `getObservedIndices`
- `src/filter/filter.cpp` — implement `getObservedIndices`
- `src/main.cpp` — `observed_count`, ratio voting, debug stats tạm thời

**Mới tạo (công cụ hỗ trợ, ngoài phạm vi code core):**
- `scripts/run_n_sweep.sh` — script N-sweep v1 (map theo frame, PHÁT HIỆN RA
  BUG precision thấp, đã lỗi thời, giữ lại làm tham khảo lịch sử)
- `scripts/check_baseline_distance.py` — đo khoảng cách thật scan↔map
- `scripts/select_maps_by_distance.py` — chọn map theo khoảng cách thật
- `scripts/run_n_sweep_v2.sh` — script N-sweep v2 (đã fix, dùng
  `select_maps_by_distance.py`)
- `NOTES_viec4_N_sweep_debug.md` — log debug chi tiết (số liệu, root cause)
- `HANDOFF_VIEC4.md` — file này (bối cảnh đầy đủ, kể cả phần thảo luận ngoài code)
- `.gitignore` — thêm `install/`, `log/`, `results_n_sweep/`, `results_n_sweep_v2/`

---

## 10. Việc cần làm tiếp theo (todo tổng hợp cuối cùng)

1. **Ngay lập tức:** hoàn tất bước push code (mục 8).
2. **Ngắn hạn:** thử `max_distance_m` nhỏ hơn 8m (vd 3-5m) xem precision có cải
   thiện thêm không, so với mốc F1=0.226 cũ.
3. **Trung hạn:** nếu precision vẫn thấp ở baseline rất nhỏ, quay lại cân nhắc
   threshold thích ứng trong `discrepancy.cpp` (hiện tại đang giữ nguyên).
4. **Dài hạn (kiến trúc):** cân nhắc đổi "map" từ 1 scan thô sang map TÍCH LUỸ
   (accumulate nhiều scan gần, đã đăng ký qua SLAM) — đúng tinh thần Removert
   gốc, giải quyết tận gốc giới hạn baseline.
5. **Dài hạn (tích hợp hệ thống):** thiết kế kiến trúc real-time cho Removert
   chạy song song FAST-LIO2 (mục 7) — cần trao đổi với người phụ trách stream A.

---

## 11. `max_distance_m` sweep — xác nhận baseline nhỏ hơn giúp precision (phiên tiếp theo)

Làm theo đề mục 10.2 ở trên. Script mới: `scripts/run_maxdist_sweep.sh` (sweep
`max_distance_m` ở N cố định, thay vì sweep N ở `max_distance_m` cố định như
`run_n_sweep_v2.sh`). Kết quả trên `results_maxdist_sweep/` (không commit, tái
tạo được).

**Tại scan_idx=150**, sweep N=2 (max_d 1.5-8m) và N=4 (max_d 3-8m):
precision giảm ĐƠN ĐIỆU khi `max_distance_m` tăng, ở CẢ 2 giá trị N — xác nhận
đúng giả thuyết baseline/parallax ở mục 4-5. Tốt nhất: N=4, max_d=3-4m
(precision=0.100, recall=0.976, F1=0.182) so với mốc cũ N=10/max_d=8m
(F1=0.164). N=2/max_d≤2m (sàn của dataset — frame cách nhau tối thiểu ~1.4m
trong seq04, không thể nhỏ hơn) cho F1 cao nhất tuyệt đối (0.191) nhưng đánh
đổi recall xuống 0.90.

**Kiểm tra generalization** trên 4 scan_idx khác (50, 100, 200, 250), so N=4/
max_d=4 với mốc cũ N=10/max_d=8 — F1 tăng ở **5/5 scan** (150, 50, 100, 200,
250): 0.164→0.182, 0.233→0.254, 0.414→0.469, 0.152→0.202, 0.154→0.175. Không
phải overfit vào scan_idx=150.

**Đã chốt baseline mới:** `N=4, max_distance_m=4.0` (cập nhật trong
`run_n_sweep_v2.sh`), thay cho `N=10, max_distance_m=8.0` cũ.

**Việc cần làm tiếp (chưa làm):**
- ~~RANSAC ground over-detect~~ → ĐÃ ĐIỀU TRA, xem mục 12 (kết quả bác bỏ phần
  lớn giả định cũ).
- Threshold vote (đang cố định 0.5) và threshold discrepancy (đang cố định,
  giữ nguyên `discrepancy.cpp`) chưa sweep lại với baseline N=4/max_d=4 mới.
- N=2/max_d≤2m (F1 cao nhất nhưng recall thấp) chưa test generalization trên
  nhiều scan như N=4 — nếu ưu tiên F1 tuyệt đối hơn recall, đáng thử.

---

## 12. Điều tra RANSAC ground over-detect — bác bỏ giả định cũ, tìm ra thủ phạm thật

### 12.1. Phương pháp

Nghi vấn cũ (mục 10.2 và `handover_notes.md`): "RANSAC detect ~69.5% điểm là
ground — hơi cao so với thực tế outdoor điển hình (30-50%), nghi ngờ RANSAC
1-mặt-phẳng đang gom nhầm cả các bề mặt phẳng khác (tường dài, vỉa hè)".

Cách kiểm chứng: KHÔNG sửa code core. Viết script Python độc lập đọc `.bin` +
`.label` (SemanticKITTI có nhãn ground-truth cho TỪNG điểm), tự chạy RANSAC với
đúng tham số PCL trong `ground_filter.cpp` (`SACMODEL_PLANE`, `SAC_RANSAC`,
`distance_threshold=0.2`, `max_iterations=200`, refit least-squares trên inliers
= `setOptimizeCoefficients(true)`). Class được coi là ground thật: road(40),
parking(44), sidewalk(48), other-ground(49), lane-marking(60), terrain(72).

Ba câu hỏi tách bạch: (a) tỷ lệ ground THẬT là bao nhiêu, (b) mặt phẳng tìm được
có NGANG không, (c) gom nhầm class nào.

### 12.2. Phát hiện 1 — con số 69.5% KHÔNG tái tạo được

| | scan 150 |
|---|---|
| Ground THẬT (ground-truth) | **43.7%** |
| RANSAC detect | **50.2%** |

Rất ổn định: 50.2% ±0.1% qua 5 seed khác nhau. Quét `distance_threshold` từ 0.1
đến 0.5 chỉ cho 41.5% → 62.0% — **không có tham số nào đạt tới 69.5%**. Con số
69.5% trong ghi chú cũ nhiều khả năng đã lỗi thời (đo trước các lần fix FOV/blind)
hoặc đo bằng cách khác.

Ngoài ra mốc so sánh "30-50% điển hình outdoor" cũng không đúng với seq04: ground
THẬT ở đây dao động 43.7-65.9% tùy scan (scan 250 tới 65.9%). Nên over-detect là
CÓ THẬT nhưng chỉ ~6.5 điểm phần trăm — nhẹ hơn nhiều so với lo ngại ban đầu.

### 12.3. Phát hiện 2 — giả thuyết "bám nhầm tường dài" là SAI

Đo độ nghiêng của mặt phẳng RANSAC tìm được so với mặt ngang (góc giữa vector
pháp tuyến và trục z): **~1° trên cả 5 scan** (nếu bám nhầm tường đứng thì phải
~90°). Mặt phẳng nằm ở độ cao -1.75m dưới sensor — khớp chiều cao gắn Velodyne
HDL-64E trên xe KITTI.

→ RANSAC bám ĐÚNG mặt đất. Loại bỏ hoàn toàn giả thuyết "gom nhầm tường".

Cái nó thật sự gom nhầm (scan 150, 13231 điểm sai):

| Class bị gom nhầm vào ground | % của tổng số gom nhầm | % class đó bị nuốt |
|---|---|---|
| **vegetation** (cỏ/bụi thấp) | **87.2%** | 23.8% |
| fence | 10.0% | 6.7% |
| unlabeled | 1.7% | 13.3% |

Và nó **bỏ sót 35.9% điểm sidewalk** — vì vỉa hè cao hơn mặt đường ~10-15cm nên
là một mặt phẳng KHÁC, RANSAC 1-mặt-phẳng về bản chất không mô tả được cả hai.

→ Kết luận đúng của ghi chú cũ ("nên cân nhắc Patchwork/Patchwork++") vẫn giữ
nguyên giá trị, nhưng vì lý do khác: không phải do bám tường, mà do (i) không
biểu diễn được nhiều mặt phẳng ở độ cao khác nhau, và (ii) nuốt cỏ/bụi thấp.

### 12.4. Phát hiện 3 (QUAN TRỌNG NHẤT) — ground KHÔNG phải chỗ đang mất precision

Phân tích thành phần 3241 điểm bị báo nhầm dynamic (FP) ở scan 150, cấu hình tốt
nhất N=4/max_d=4:

| Thành phần FP | Tỷ lệ |
|---|---|
| **vegetation** | **70.7%** |
| sidewalk + road (ground thật) | 14.3% |
| unlabeled | 8.3% |
| trunk, fence, building, pole | 6.7% |

→ **Kể cả ground segmentation hoàn hảo cũng chỉ cứu được ~14% FP.** Vegetation
mới là thủ phạm chính (đo trên 5 scan: 31-71% FP là vegetation).

Lý do bản chất: tán lá/bụi là **bề mặt xốp (porous)** — tia laser xuyên qua khe
lá khác nhau ở mỗi viewpoint, nên range đo được lệch >0.5m NGAY CẢ KHI baseline
rất nhỏ và cây hoàn toàn đứng yên. Đây không phải lỗi thuật toán, mà là giới hạn
của việc so sánh range theo từng tia đơn lẻ.

### 12.5. Phát hiện 4 — lỗ hổng logic thật, ĐÃ SỬA

Ground bị loại khỏi **range image** (`buildRangeImage` nhận `exclude_mask`),
nhưng **không bị loại khỏi danh sách ứng viên dynamic**: `getDynamicIndices`
duyệt MỌI điểm rồi tra `discrepancy_image[row][col]`, nên một điểm mặt đường vẫn
bị gán "dynamic" theo phán quyết của pixel mà nó chia sẻ với vật khác — rõ ràng
không ai cố ý.

Đo được (scan 150): **745/3602 (20.7%)** điểm báo dynamic nằm trong mask ground,
trong đó **724 là FP, chỉ 21 là TP** — tỷ lệ đánh đổi rất tốt.

**Đã sửa trong `main.cpp`:**
1. Thêm `if (scan_ground_mask[i]) continue;` ở tầng voting — ground không bao giờ
   là dynamic.
2. Tiện thể: `detectGroundMask(scan_in_map_frame)` trước đây gọi lại MỖI vòng lặp
   map. Ground membership **bất biến qua phép biến đổi cứng** (phép quay/tịnh
   tiến bảo toàn khoảng cách điểm→mặt phẳng), nên chuyển ra ngoài vòng lặp, tính
   1 lần trên `scan_frame.cloud`. Vừa đúng hơn (nhất quán giữa các map — RANSAC
   có yếu tố ngẫu nhiên) vừa nhanh hơn ~15% (N=4: ~0.87s → 0.74s).

**Kết quả thật sau khi sửa** (N=4, max_distance_m=4, threshold=0.5):

| scan | F1 trước | F1 sau |
|---|---|---|
| 50 | 0.254 | **0.273** |
| 100 | 0.469 | **0.506** |
| 150 | 0.182 | **0.203** |
| 200 | 0.202 | 0.200 |
| 250 | 0.175 | **0.192** |
| **trung bình** | **0.256** | **0.275** |

Tăng ở 4/5 scan, 1 scan đi ngang. Recall giảm nhẹ ở vài scan (một số điểm dynamic
thật sát mặt đất bị RANSAC coi là ground → mất) nhưng precision tăng bù lại nhiều
hơn.

### 12.6. Hướng đã LOẠI BỎ — threshold thích ứng theo range

Mục 10.3 từng đề xuất "threshold thích ứng trong `discrepancy.cpp`". Đo phân bố
FP theo khoảng cách tới sensor (scan 150):

| range (m) | 0-5 | 5-10 | 10-15 | 15-20 | 20-30 | 30-40 | 40-60 | 60+ |
|---|---|---|---|---|---|---|---|---|
| % của FP | 6.3 | 17.1 | **26.2** | 18.0 | 11.7 | 6.7 | 9.3 | 4.8 |

FP phân bố đều, đỉnh ở 10-15m (không phải ở xa). Khoảng cách trung bình: FP =
20.8m, TP = **27.1m** — điểm dynamic thật còn XA HƠN điểm báo nhầm.

→ Nới threshold theo range sẽ giết TP nhiều hơn FP. **Gạch hướng này khỏi danh
sách việc tồn đọng.**

### 12.7. Vegetation được xử lý thế nào trong literature? (câu hỏi đặt ra khi bàn giao)

Vegetation là vấn đề đã được nhận diện rõ trong SLAM/mapping dài hạn. Các hướng
chính (ghi lại để tham khảo, CHƯA áp dụng cái nào):

**(a) Removert gốc — multi-resolution + "revert" (Kim & Kim, IROS 2020).**
Đây là điều đáng chú ý nhất: `mini_removert` hiện chỉ implement nửa đầu của
thuật toán gốc. Tên đầy đủ là *"Remove, then Revert"* — sau bước remove hung hãn
(chấp nhận nhiều FP), có bước **revert** khôi phục lại các điểm static bị xoá
oan, dùng bằng chứng từ range image ở **độ phân giải THÔ hơn**. Ở độ phân giải
thô, mỗi pixel gộp nhiều điểm nên nhiễu do bề mặt xốp/lệch nhỏ bị trung bình hoá
đi. Chính cơ chế này là câu trả lời của Removert gốc cho đúng vấn đề vegetation
đang gặp. → Hướng cải tiến tiềm năng nhất, và trung thành với thiết kế gốc nhất.

**(b) ERASOR (Lim et al., ICRA 2021) — so sánh THỐNG KÊ theo bin, không theo tia.**
Chia không gian thành các bin toạ độ cực quanh robot (R-POD), rồi so sánh **tỷ lệ
chiều cao bị chiếm** giữa scan và map trong từng bin (Scan Ratio Test), thay vì
so range của từng tia. Thống kê phân bố theo bin ít nhạy cảm hơn nhiều với việc
tia laser trúng đúng chiếc lá nào. Kèm R-GPF (ground plane fitting theo từng
vùng) thay cho RANSAC toàn cục.

**(c) Patchwork / Patchwork++ (Lim et al., RA-L 2021 / IROS 2022) — ground
segmentation theo vùng đồng tâm.** Chia mặt đất thành các vành khuyên × quạt,
fit mặt phẳng RIÊNG cho từng bin. Giải quyết trực tiếp đúng 2 vấn đề đo được ở
mục 12.3: vỉa hè ở độ cao khác (mỗi bin có mặt phẳng riêng) và cỏ/bụi thấp bị
nuốt (kiểm tra độ dày/độ phẳng dọc trong bin sẽ loại bin quá "xù").

**(d) Đặc trưng hình học cục bộ (eigenvalue-based) — proxy rẻ nhất, dễ implement
nhất.** Tính ma trận hiệp phương sai của lân cận mỗi điểm (kNN hoặc bán kính),
lấy 3 trị riêng λ1≥λ2≥λ3, rồi suy ra: *linearity* = (λ1-λ2)/λ1, *planarity* =
(λ2-λ3)/λ1, *scattering* = λ3/λ1. Vegetation có **scattering cao** (điểm tán ra
3 chiều), còn tường/mặt đường có planarity cao, cột/thân cây có linearity cao.
Dùng scattering làm cờ "bề mặt không đáng tin" → bỏ qua hoặc nâng ngưỡng cho các
điểm đó. Tham khảo: Demantké et al. 2011, Weinmann et al. 2015.

**(e) Ngữ nghĩa (semantic) — dùng mạng phân loại điểm.** RangeNet++, SalsaNext,
Cylinder3D... gán class cho từng điểm; vegetation thành một class riêng, có chính
sách xử lý riêng. Thực hành phổ biến trong lifelong mapping: chia bản đồ thành
nhiều tầng theo độ ổn định — (1) kết cấu vĩnh viễn (nhà, mặt đường), (2) bán tĩnh
(xe đỗ), (3) biến dạng/không ổn định (vegetation), (4) động thật. SemanticKITTI
sinh ra chính là để phục vụ hướng này.

**(f) Mô hình xác suất tồn tại thay cho nhị phân static/dynamic.** Ví dụ
persistence filter (Rosen, Mason, Leonard, ICRA 2016) dùng phân tích sống sót
(survival analysis) để gán cho mỗi feature một xác suất "còn tồn tại". Vegetation
sẽ có persistence thấp → bị giảm trọng số dần, thay vì bị xoá cứng một lần.

**(g) Nhất quán không-thời gian của không gian trống.** Ví dụ Dynablox (Schmid
et al., RA-L 2023): chỉ kết luận dynamic khi có bằng chứng nhất quán rằng vùng
đó từng được quan sát là TRỐNG. Vegetation bị trúng tia lúc có lúc không sẽ không
tích luỹ đủ bằng chứng nhất quán → tự nhiên bị loại. Không cần mạng semantic.

**(h) Đa hồi (multi-echo/multi-return) — mẹo phần cứng.** LiDAR nhiều hồi trả về
nhiều echo cho một tia khi xuyên qua tán lá, còn bề mặt cứng chỉ trả một. Đây là
tín hiệu phân biệt vegetation gần như miễn phí. Velodyne HDL-64E trong KITTI chỉ
có single return nên không dùng được ở đây, nhưng **đáng kiểm tra khi chuyển sang
dữ liệu Go2 thật** (một số dòng Livox có hỗ trợ).

> Lưu ý: các tham chiếu trên ghi theo trí nhớ, nên **kiểm tra lại tên/năm trước
> khi trích dẫn chính thức**. Nếu có paper cụ thể muốn đối chiếu, đưa link/PDF
> để đọc trực tiếp.

**Xếp hạng đề xuất theo tỷ lệ lợi ích/công sức cho `mini_removert`:**
1. **(a) revert stage đa độ phân giải** — đúng thiết kế gốc, nhắm trúng vegetation,
   không cần thư viện ngoài.
2. **(d) scattering từ trị riêng** — rẻ, ~50 dòng, dùng ngay PCL có sẵn.
3. **(c) Patchwork++** — thay `ground_filter.cpp`, có source công khai, nhưng chỉ
   cứu được ~14% FP nên ưu tiên thấp hơn dù nghe hấp dẫn.

### 12.8. Script chẩn đoán

Ba script dùng cho điều tra này đặt ở thư mục tạm (`$CLAUDE_JOB_DIR/tmp`), KHÔNG
commit vì chỉ dùng một lần: `diag_ground.py` (RANSAC + so GT + phân tích class
gom nhầm), `diag_fp.py` (thành phần FP theo class), `diag_range.py` (FP/TP theo
khoảng cách + ước tính lợi ích khi ép ground=static). Nếu cần chạy lại, viết lại
theo mô tả ở mục 12.1-12.6 — logic đều đơn giản, chỉ cần numpy.