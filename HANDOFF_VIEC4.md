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