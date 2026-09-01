# START_HERE — đọc 2 phút đầu mỗi buổi

> Đây KHÔNG phải handoff. Handoff là tài liệu tra cứu (hiện ~300 dòng/file, 9 file).
> File này chỉ trả lời: **ta đang làm gì, vì sao, và hôm nay đụng vào đâu.**
> Khối HÔM NAY viết lại mỗi buổi. Phần dưới gần như không đổi.

---

## 🔴 HÔM NAY (cập nhật 01/09/2026)

**Trạng thái:** E2 việc 3 + 4 xong, build sạch, binary cho số ĐỌC ĐƯỢC. Chưa commit.

**Câu hỏi mở duy nhất — precision quá tệ, FP ở đâu ra?**

```
seq04/150:  P 0.208   R 0.814   F1 0.332      301 TP / 1144 FP
seq06/349:  P 0.032   R 0.546   F1 0.061      100 TP / 3013 FP
seq06/939:  P 0.062   R 0.782   F1 0.115      140 TP / 2110 FP
```

Recall ổn — nó **tìm được** vật động. Precision sụp — nó vơ kèm quá nhiều thứ tĩnh.

**Manh mối mạnh nhất đang có:** cùng một mặt phẳng ground, nhưng
`scan = 49.8%` còn `map tích luỹ = 38.3%`. Lệch 11.4 điểm phần trăm.
Giả thuyết: map trải dài cả trăm mét, mặt đường cong ra khỏi mặt phẳng fit tại chỗ của
scan, phần cong quá 0.2m không bị mask → lọt vào range image như thể là vật thể.

**Việc của bạn buổi tới:** chia `map_cloud` theo vành đai khoảng cách tới gốc scan, in tỷ lệ
ground từng vành đai. Tụt dần theo khoảng cách ⇒ giả thuyết đúng.
❌ Đã bác bỏ rồi, đừng đi lại: "hai mặt phẳng RANSAC riêng gây FP" — đã hợp nhất, số không đổi.

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
5. **FP đến từ đâu** — câu hỏi mở hiện tại.

## Đọc log: con số nào nghĩa là gì

| Thấy gì | Nghĩa |
|---|---|
| `ground seed=0` ≈ **49.75%** (seq04/150) | bình thường. Lệch nhiều ⇒ ground filter có chuyện |
| `ground cua map` vs `ground cua scan` | lệch nhiều ⇒ manh mối HÔM NAY |
| seq04/150 gộp **1.141.089 → 118.574** | khớp mốc. Sai ⇒ bước gộp/voxel hỏng |
| seq06/939 gộp **993.711 → 133.939** | khớp mốc |
| `observed_count` `min=0 max=1` | đúng sau việc 4. Thấy `max=9` ⇒ còn vòng lặp map cũ |
| **ΔF1 < 0.07** trên 1 ground mask | ⛔ **KHÔNG kết luận được gì.** Cần `run_seed_sweep.py` |
| seq06/349 hoặc 06/939 **= 0.000** | ⛔ lỗi CÀI ĐẶT, không phải ý tưởng sai |
| REVERT ~80-90% | hiện tại là vậy. Đáng nghi, chưa điều tra |
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
