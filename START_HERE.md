# START_HERE — đọc 2 phút đầu mỗi buổi

> Đây KHÔNG phải handoff. Handoff là tài liệu tra cứu (hiện ~300 dòng/file, 9 file).
> File này chỉ trả lời: **ta đang làm gì, vì sao, và hôm nay đụng vào đâu.**
> Khối HÔM NAY viết lại mỗi buổi. Phần dưới gần như không đổi.

---

## 🔴 HÔM NAY (cập nhật **02/09/2026**, cuối phiên Linux)

**Trạng thái:** mục 15.9 **ĐÃ ĐÓNG** — ba bảng khám nghiệm FP chạy xong, kết luận có bằng chứng số.
Toàn bộ chi tiết: **`HANDOFF_2026-09-02.md`**. Không đụng một dòng C++ nào trong phiên.

**✅ Câu hỏi mở "FP ở đâu ra?" — đã trả lời:**

> FP chủ yếu từ **ĐƯỜNG ỐNG**, và **tách được theo cự ly**: ngoài 40 m là hình học/mật độ (41% FP,
> lệch âm trung vị 8,5 m) · 20–40 m là luật quyết định (25% FP, lệch trung vị 1,2 m) · dưới 20 m
> (34% FP) còn hỗn hợp, **chưa gọi tên sạch**.

Ba kết quả phụ, đều quan trọng:

- **REVERT hoàn toàn lành mạnh** — giết 12.364 ứng viên, **không mất một TP nào** (301/301 sống sót).
  Cả 69 FN đều mất ở bước **REMOVE**, không phải do revert quá tay. Tỉ lệ REVERT 89,5% hết đáng nghi.
- **Trong vành 20–40 m, baseline đạt P = 0,516** — ngưỡng đặt tay không hề đặt sai chỗ, nó chỉ chết
  ở hai đầu cự ly.
- **TP có trung vị |lệch| = 15 m** trong khi ngưỡng đặt ở 0,5 m. Luận điểm đề tài ở dạng số.

**⭐ Phát hiện cuối phiên, đáng giá ngang ba bảng:** cả **65 FN** không-ground đều mất vì **map MÙ
ở thang mịn** (`observed_L0 = 0`), **không** phải vì lệch dưới ngưỡng — 0 điểm nào lệch yếu. Ở thang
thô chúng lệch **13,7–15,3 m**, 100% trên 2 m, mạnh ngang TP. Gốc rễ: map gộp+voxel chỉ lấp được
**57,5%** số pixel ở 64×900 (mù 42,5%; L1 17,3% · L2 11,2% · L3 9,5%), mà REMOVE chỉ chạy ở L0.
Đây là **giới hạn cấu trúc**, không ngưỡng nào cứu được — và là lập luận **kiến trúc** cho hướng học
máy. ⚠️ Kèm một cái bẫy: *"vậy REMOVE ở L1 là xong, cần gì ML?"* — 51/65 điểm sẽ được cứu. Phải đo
`REMOVE@L1` như một baseline biến thể. Xem `HANDOFF_2026-09-02.md` mục 10.

**⭐ Việc đầu tiên của phiên tới (đụng C++):** cắt bán kính **40 m** vào baseline — bỏ 471 FP, mất 1 TP,
dự kiến P 0,208 → **0,308**, F1 0,332 → **0,446**. Không phải tune ngưỡng: removert gốc vốn có giới hạn
bán kính. Đo lại cả ba mốc rồi mới đi bước 3. Xem `HANDOFF_2026-09-02.md` mục 7.
Kèm theo: đo `REMOVE@L1` (mục 10.6) và kiểm tỉ lệ map mù trên seq06 — **rất có thể đó là lời giải
cho F1 0,061 của seq06/349**.

❌ **Đã bác bỏ bằng số, đừng đi lại** (danh sách đầy đủ ở cuối `CHECKLIST-15.9-FP.md`):
- "hai mặt phẳng RANSAC riêng gây FP" · "mặt đường cong" · chênh lệch ground 49,8/38,3%
- ⭐ **bỏ `fabs()` (luật một chiều)** — bác bỏ lần 3 ngày 02/09: dưới bộ nguồn mới (±10–14 frame),
  luật một chiều giữ lại **0/301 TP**. Nguồn xa hơn về thời gian làm vệt vật động dịch **mạnh hơn**
  về phía âm, không hề sạch hơn. **Cấm đụng, vĩnh viễn.**

**Mạch CSV:** bước 1 + 2 ✅. Còn bước 3 (nối nhiều scan) và bước 4 (MLP numpy). Xem mục 14 handoff 01/09.

**Chưa làm, còn nợ:** 391 FP vùng <20 m chưa gọi tên cơ chế · bag `standstill_*` (mục 15.2, nửa buổi
riêng, quân bài rẻ nhất chưa lật) · `scripts/viz_scan.py` · **đồng bộ tài liệu ổ D sang repo** (repo
đang thiếu hẳn mục 15 của handoff 01/09).

> ⚠️ **Môi trường:** `python3` hệ thống **KHÔNG có pandas**. Dùng `~/pointcloud_env/bin/python`
> (pandas 3.0.3 + matplotlib 3.10.9). `add_gt_label.py` chỉ cần numpy nên `python3` thường vẫn chạy.

---

## Ta đang làm gì

Cài lại **Removert** (Kim & Kim, IROS 2020) ở mức tối giản: tách điểm động (xe, người) khỏi
điểm tĩnh trong một scan LiDAR.

Cách làm, bốn câu:
1. Chiếu scan và map xuống **range image** (ảnh 2D, mỗi pixel giữ khoảng cách gần nhất).
2. Căn hai ảnh về **cùng một hệ toạ độ**. Vật tĩnh sẽ chồng khít lên nhau.
3. Pixel nào **lệch quá ngưỡng** → ứng viên động (REMOVE).
4. Kiểm lại ở các thang phân giải **thô hơn**. Không xác nhận được thì trả về tĩnh (REVERT).

## Đề tài khẳng định điều gì

**Learning-Based Dynamic Point Removal for Low-Cost LiDAR Mapping**
Base: LMNet (Chen 2021) · Baseline: Removert

Removert quyết định động/tĩnh bằng **ngưỡng đặt tay**. Ngưỡng đó yếu đi trên LiDAR rẻ nhiễu
nặng. Đề tài thay chỗ quyết định đó bằng một bộ phân loại **học từ dữ liệu**.

> ⭐ **Vì sao baseline là tất cả:** không có "ngưỡng đặt tay làm được bao nhiêu" thì câu
> "mô hình học tốt hơn" không có nghĩa. **Không so sánh = không có bài báo.**
> Đó cũng là lý do cột `baseline_label` trong CSV quan trọng hơn mọi cột khác.

> ⭐ **Chỗ ngược đời phải nhớ:** `mini_removert` làm chưa tốt **là nguyên liệu**, không phải
> trở ngại. Ngưỡng cứng sai ở đâu — đó chính là baseline để đối chiếu. Nó mà hoàn hảo thì
> không có đề tài nào cả.

## Năm ý tưởng phải tự làm chủ

Thầy và hội đồng hỏi cột này. Không ai hỏi cú pháp CMake hay `std::vector<bool>`.

1. **Hệ toạ độ** — vì sao phải căn frame, và vì sao căn sai thì vật tĩnh bị vu là động.
2. **Remove rồi revert** — vì sao thang mịn nhạy, thang thô chắc, và vì sao đòi *mọi* thang
   thô xác nhận (F1 0.630) chứ không phải *một* thang (0.492).
3. **Range image** — vì sao chiếu 3D xuống 2D, vì sao mỗi pixel lấy `min(range)`.
4. **Vì sao phải có baseline** — xem trên.
5. **FP đến từ đâu** — đã trả lời 02/09: đường ống ở >40 m, luật quyết định ở 20–40 m,
   vùng <20 m còn hỗn hợp. Xem `HANDOFF_2026-09-02.md`.

## Đọc log: con số nào nghĩa là gì

| Thấy gì | Nghĩa |
|---|---|
| `ground seed=0` ≈ **49.75%** (seq04/150) | bình thường. Lệch nhiều ⇒ ground filter có chuyện |
| `ground cua map` **38,3%** vs `ground cua scan` **49,8%** | **lành tính**, chỉ do phân bố bán kính (mục 13.2). Đừng đi lại |
| seq04/150 gộp **1.141.089 → 118.574** | khớp mốc. Sai ⇒ bước gộp/voxel hỏng |
| seq06/939 gộp **993.711 → 133.939** | khớp mốc |
| `observed_count` `min=0 max=1` | đúng sau việc 4. Thấy `max=9` ⇒ còn vòng lặp map cũ |
| **ΔF1 < 0.07** trên 1 ground mask | ⛔ **KHÔNG kết luận được gì.** Cần `run_seed_sweep.py` |
| seq06/349 hoặc 06/939 **= 0.000** | ⛔ lỗi CÀI ĐẶT, không phải ý tưởng sai |
| REVERT ~80-90% | **bình thường, đã điều tra 02/09**: giết 12.364 ứng viên, mất **0** TP |
| Kết quả lạ bất kỳ | **xem `#nguồn` và `observed_count` TRƯỚC TIÊN** |

## Ba lệnh để tự chạy

```bash
# 1) chọn nguồn — map_idx nằm ở DÒNG CUỐI stdout
python3 scripts/select_source_ladder.py ~/kitti_data/dataset/poses/04.txt \
    ~/kitti_data/dataset/sequences/04/calib.txt 150

# 2) chạy — ghi dynamic_indices_scan150.txt vào thư mục hiện tại
./build/mini_removert/mini_removert ~/kitti_data/dataset/sequences/04/pcd \
    ~/kitti_data/dataset/poses/04.txt ~/kitti_data/dataset/sequences/04/calib.txt \
    150 0.5 <map_idx...>

# 3) chấm điểm
python3 scripts/evaluate.py \
    ~/kitti_data/dataset/sequences/04/labels/000150.label dynamic_indices_scan150.txt
```

Build: `colcon build --packages-select mini_removert`

## Luật bất di bất dịch

- ❌ **Không tune ngưỡng cho ra số đẹp.** Ngưỡng đặt tay CHÍNH LÀ baseline của đề tài.
  Tune nó là tự phá baseline của mình.
- ❌ **Không "làm map đẹp hơn".** Livox L1 nhiễu tự phản xạ 87-92% là **giới hạn phần cứng**.
  Ghi vào Limitations, đừng sửa.
- ✅ **Đo trước, sửa sau.** Viết script Python đọc thẳng `.bin`/`.label` để kiểm giả thuyết,
  đừng đụng C++ trước. Cách này đã bác bỏ 3 giả định sai về RANSAC mà không sửa dòng nào.
- ✅ **Một mask không kết luận được gì** (ΔF1 < 0.07 là nhiễu).

## Cách làm việc (chốt 01/09)

1. **Bạn** chạy lệnh, đọc log, đoán chỗ hỏng.
2. **Claude** đề xuất hướng + nói cái giá.
3. **Bạn** gật.
4. **Claude** sửa code.
5. Trước mỗi lần Claude viết: bạn nói **một hai câu** đoán trước *"chỗ này phải làm X, vì Y"*.
   Đoán sai thì nhớ rất lâu — đó mới là chỗ học, không phải bài kiểm tra cuối buổi.

> Chẩn đoán KHÓ HƠN gõ code. Vài buổi đầu thấy bí hơn là **bình thường và đúng hướng**.
