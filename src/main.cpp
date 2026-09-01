#include "discrepancy/discrepancy.hpp"
#include "range_image/range_image.hpp"
#include "io/pcd_loader.hpp"
#include "io/pose_loader.hpp"
#include "io/calib_loader.hpp"
#include "filter/filter.hpp"
#include "transform/transform.hpp"
#include "ground_filter/ground_filter.hpp"
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <sstream>

int main(int argc, char** argv)
{
    if (argc < 7)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <pcd_dir> <poses.txt> <calib.txt> <scan_idx> <threshold> <map_idx_1> [map_idx_2] [map_idx_3] ...\n";
        return 1;
    }

    std::string pcd_dir    = argv[1];
    std::string poses_path = argv[2];
    std::string calib_path = argv[3];
    int scan_idx = std::stoi(argv[4]);
    float threshold = std::stof(argv[5]);
    std::string levels_arg = "64x900,32x450,16x225,8x112";
    float vote_threshold = 0.5f;
    int ground_seed = 0;
    std::vector<int> map_indices;
    for (int i = 6; i < argc; )
{
    std::string arg = argv[i];// --- Nhánh 1: cờ đã biết ---
    if (arg == "--vote-threshold")
    {   
        if (i + 1 >= argc) { std::cerr << "thieu gia tri sau " << arg << '\n'; return 1; }
        vote_threshold = std::stof(argv[i + 1]);
        i += 2;
    }
    else if (arg == "--levels")
    {
        if (i + 1 >= argc) { std::cerr << "thieu gia tri sau " << arg << '\n'; return 1; }
        levels_arg = argv[i + 1];
        i += 2;
    }
    else if  (arg == "--ground-seed")
    {   
        if (i + 1 >= argc) { std::cerr << "thieu gia tri sau " << arg << '\n'; return 1; }
        ground_seed = std::stoi(argv[i + 1]);
        i += 2;
    }
    // --- Nhánh 2: bắt đầu bằng "--" nhưng không khớp nhánh 1 -> gõ nhầm ---
    else if (arg.rfind("--", 0) == 0)
    {
        std::cerr << "co la: " << arg << '\n';
        return 1;
    }
    // --- Nhánh 3: không phải cờ -> map_idx ---
    else
    {
        map_indices.push_back(std::stoi(arg));
        i += 1;
    }
}
    auto poses = loadKittiPoses(poses_path);
    Eigen::Matrix4f Tr = loadKittiTr(calib_path);

    char buf[32];
    snprintf(buf, sizeof(buf), "%06d.pcd", scan_idx);
    std::string scan_path = pcd_dir + "/" + buf;

    Eigen::Matrix4f scan_pose = poses[scan_idx] * Tr;
    Frame scan_frame = loadFrame(scan_path, scan_pose, 0.0);

    std::cout << "Loaded scan [" << scan_idx << "]: " << scan_frame.cloud->points.size() << " points\n";
    std::cout << "So sánh với " << map_indices.size() << " map: ";
    for (int idx : map_indices) std::cout << idx << " ";
    std::cout << '\n';

    constexpr float v_angle_min = -24.8f;
    constexpr float v_angle_max = 2.0f;

    // Cạnh ô voxel cho map tích luỹ (đề bài E2). 0.2m là giá trị đã dùng xuyên suốt phép
    // đo bộ nguồn (`scripts/src_quality.py`, VOXEL = 0.2), nên đổi số này là mọi mốc AUC
    // của HANDOFF_2026-08-20 hết đối chiếu được.
    constexpr float VOXEL_LEAF_M = 0.2f;

    // === Multi-resolution: nền tảng của "remove, then revert" (Removert gốc) ===
    // Thang MỊN thì nhạy (bắt gần hết vật động thật) nhưng nhiều báo nhầm: lệch
    // viewpoint nhỏ hoặc bề mặt xốp (tán lá) làm 2 pixel cùng vị trí trúng 2 bề mặt
    // vật lý khác nhau. Thang THÔ gộp nhiều điểm vào 1 pixel nên nhiễu ngẫu nhiên đó
    // bị triệt tiêu, trong khi vật động THẬT vẫn bất nhất ở mọi thang.
    //   levels[0]      = thang REMOVE (nhạy, quyết định ứng viên)
    //   levels[1..n-1] = thang REVERT (chắc, phải xác nhận thì ứng viên mới được giữ)
    // Đo trên seq04 (5 scan): vật động thật còn 98% ở thang thô, báo nhầm chỉ còn 24%
    // -> F1 trung bình 0.275 (chỉ remove) lên 0.629 (remove + revert).
    //
    // Số thang đã sweep (HANDOFF_2026-08-13 mục 14, chạy ở threshold=1.0):
    //   2 thang 0.599-0.673 | 3 thang 0.691 | 4 thang 0.720 | 5 thang 0.698 | 6 thang 0.546
    // Phần lớn sức nặng nằm ở ĐẦU THÔ chứ không ở thang giữa: bỏ 16x225 mất rất nhiều
    // (0.599), bỏ 32x450 gần như không mất (0.673). Nhưng thô quá thì 1 pixel nuốt cả
    // vật động lẫn nền nên vật động bị che mất -> recall sụp (6 thang: R chỉ còn 0.459).
    struct Resolution { int height; int width; std::string name; };  // KHÔI PHỤC, name đổi kiểu
    std::vector<Resolution> levels;                                   // rỗng, KHÔNG const

    std::stringstream ss(levels_arg);
    std::string tok;
    while (std::getline(ss, tok, ','))
    {
        size_t xpos = tok.find('x');
        if (xpos == std::string::npos) { std::cerr << "--levels sai dinh dang (thieu 'x'): " << tok << '\n'; return 1; }   // bẫy 64-900

        int h = std::stoi(tok.substr(0, xpos));       // chỉ MỘT cặp h/w
        int w = std::stoi(tok.substr(xpos + 1));
        if (h <= 0 || w <= 0) { std::cerr << "--levels co kich thuoc khong hop le: " << tok << '\n'; return 1; }   
        levels.push_back({h, w, std::to_string(h) + "x" + std::to_string(w)});
    }
    if (levels.empty()) { std::cerr << "--levels rong\n"; return 1; }
    const size_t n_levels = levels.size();
    const size_t n_points = scan_frame.cloud->points.size();

    // vote_count[level][i]:     số map kết luận điểm i là dynamic, ở thang `level`
    // observed_count[level][i]: số map THỰC SỰ quan sát được điểm i, ở thang `level`
    //                           (mẫu số động, thay cho map_indices.size() cố định)
    std::vector<std::vector<int>> vote_count(n_levels, std::vector<int>(n_points, 0));
    std::vector<std::vector<int>> observed_count(n_levels, std::vector<int>(n_points, 0));

    // Ground mask của scan: tính MỘT LẦN, dùng cho mọi thang phân giải.
    // Sau E2 việc 4 thì scan GIỮ NGUYÊN hệ của nó, nên mask này áp thẳng lên
    // `scan_frame.cloud` — không còn bản `scan_in_map_frame` nào để phải giữ nhất quán.
    // (Ghi chú cũ vẫn đúng nhưng giờ không cần tới: ground membership bất biến qua phép
    // biến đổi cứng, vì quay/tịnh tiến bảo toàn khoảng cách điểm→mặt phẳng.)
    //
    // E2 VIỆC 3: fit mặt phẳng MỘT LẦN trên scan, rồi dùng CHUNG hệ số đó cho cả hai cloud.
    // Trước đây scan và map mỗi bên chạy một lần RANSAC riêng ⇒ ra hai mặt phẳng khác nhau
    // ⇒ một điểm mặt đường bị loại khỏi ảnh này mà còn nguyên ở ảnh kia ⇒ sinh chênh lệch
    // range GIẢ ⇒ false positive hàng loạt.
    //
    // Vì sao dùng thẳng được hệ số của scan cho map, không phải biến đổi mặt phẳng:
    // `map_cloud` ĐÃ ở hệ scan (đưa về từ bước gộp, `relative_pose` = scan⁻¹·map). Cùng hệ
    // toạ độ thì cùng phương trình mặt phẳng — "đề bài C" tự tan biến.
    const Eigen::Vector4f ground_plane =
        ground_filter::detectGroundPlane(scan_frame.cloud, ground_seed);
    auto scan_ground_mask = ground_filter::maskFromPlane(scan_frame.cloud, ground_plane);
    // DEBUG: kích cỡ ground mask — dùng để chấm `--ground-seed` (mốc seq04 ~50%, mục 3.1
    // phiên 12/8). In CẢ số đếm lẫn phần trăm: số đếm là bằng chứng gốc, còn phần trăm mới
    // là thứ so sánh được giữa các scan (5 scan mốc lệch nhau tới 1.455 điểm) và giữa các
    // cảm biến (Livox L1 của Go2 chỉ cho ~20k điểm/scan, số đếm tuyệt đối vô nghĩa ở đó).
    const auto n_ground = std::count(scan_ground_mask.begin(), scan_ground_mask.end(), true);
    const size_t n_scan_points = scan_frame.cloud->points.size();
    std::cout << "ground seed=" << ground_seed << ": " << n_ground << "/" << n_scan_points
              << " diem (" << 100.0f * n_ground / n_scan_points << "%)\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZ>);        
    for (int map_idx : map_indices)
    {
        snprintf(buf, sizeof(buf), "%06d.pcd", map_idx);
        std::string map_path = pcd_dir + "/" + buf;
        Eigen::Matrix4f map_pose = poses[map_idx] * Tr;
        Frame map_frame = loadFrame(map_path, map_pose, 0.0);

        Eigen::Matrix4f relative_pose = scan_frame.pose.inverse() * map_frame.pose;
        auto map_in_scan = transform::transformPointCloud(map_frame.cloud, relative_pose);
        *accumulated += *map_in_scan;
    }
    // === VOXEL: đưa map tích luỹ về mật độ trần đồng đều (đề bài E2, mục 9 việc 2) ===
    // Mỗi ô lập phương 0.2m chỉ giữ lại MỘT điểm = trọng tâm các điểm rơi vào ô đó.
    // Không phải để chạy nhanh: 9 frame chụp cùng một chỗ làm map dày gấp 9 lần scan,
    // mà range image lấy min(range) mỗi pixel nên map dày bất thường sẽ chiếm pixel theo
    // cách scan không thể. Đây là nguyên lý 3 — hai ảnh phải đi qua CÙNG một phép rút gọn.
    //
    // Voxel phải chạy SAU khi gom đủ mọi nguồn. Voxel từng frame rồi mới nối thì 9 điểm
    // gần trùng nhau vẫn sống sót vì chúng nằm ở 9 cloud khác nhau, không ô nào gộp được.
    //
    // Trọng tâm (không phải "điểm đại diện") là điều kiện để đối chiếu được với
    // scripts/src_quality.py — voxel_downsample() bên đó cũng lấy trọng tâm.
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(accumulated);
    voxel.setLeafSize(VOXEL_LEAF_M, VOXEL_LEAF_M, VOXEL_LEAF_M);
    voxel.filter(*map_cloud);

    // In CẢ hai số: tỷ lệ co lại mới là phép kiểm, không phải con số tuyệt đối.
    // Mốc đối chiếu seq04/150 với 9 nguồn (src_quality.py): 1.141.089 -> ~118.575.
    //   còn vài trăm nghìn  -> sai CHIỀU biến đổi ở bước gộp: các frame văng lệch nhau
    //                          nên ít điểm rơi trùng ô. Số ở bước gộp MÙ với lỗi này.
    //   không giảm chút nào -> xem stderr: PCL_WARN "Leaf size is too small" thì VoxelGrid
    //                          đã trả nguyên cloud gốc (voxel_grid.hpp:255-257), không rỗng.
    std::cout << "map tich luy: " << accumulated->points.size() << " diem tu "
              << map_indices.size() << " nguon"
              << "  -> sau voxel " << VOXEL_LEAF_M << "m: " << map_cloud->points.size()
              << " diem (con " << 100.0 * map_cloud->points.size() / accumulated->points.size()
              << "%)\n";
    // Ground mask của MAP TÍCH LUỸ — E2 việc 3: KHÔNG chạy RANSAC lần hai ở đây, mà áp lại
    // đúng `ground_plane` đã fit trên scan. Đó chính là điểm mấu chốt của việc 3: hai ảnh
    // phải đi qua CÙNG một mặt phẳng thì phần mặt đường mới triệt tiêu lẫn nhau.
    //
    // Mask vẫn phải tính trên `map_cloud` (bản đã gộp + voxel), không phải trên một frame
    // nguồn lẻ: `buildRangeImage` đọc `(*exclude_mask)[i]` mà `std::vector<bool>::operator[]`
    // KHÔNG kiểm biên (`range_image.cpp:56`), nên mask lệch kích cỡ so với cloud là hành vi
    // không xác định — thường KHÔNG crash, chỉ lặng lẽ cho ra số sai.
    auto map_ground_mask = ground_filter::maskFromPlane(map_cloud, ground_plane);

    // DEBUG: tỷ lệ ground của MAP, đối chiếu với tỷ lệ ground của SCAN in ở trên. Sau việc 3
    // hai bên dùng CHUNG một mặt phẳng, nên hai tỷ lệ lệch nhau nhiều là tín hiệu đáng đọc:
    // map tích luỹ trải dài hơn scan nên mặt đường của nó cong đi khỏi mặt phẳng fit tại chỗ
    // của scan, phần cong ra ngoài 0.2m sẽ KHÔNG bị loại và đi thẳng vào range image.
    const auto n_map_ground = std::count(map_ground_mask.begin(), map_ground_mask.end(), true);
    std::cout << "ground cua map tich luy: " << n_map_ground << "/" << map_cloud->points.size()
              << " diem (" << 100.0f * n_map_ground / map_cloud->points.size() << "%)\n";

    // Giờ chỉ còn MỘT map (bản tích luỹ), nên mỗi thang phân giải chỉ có MỘT lượt so sánh,
    // thay vì một lượt cho mỗi map nguồn như trước. Ground mask không phụ thuộc độ phân
    // giải nên đã tính một lần ở trên, dùng lại cho mọi thang.
    //
    // Cả hai ảnh giờ cùng nằm trong HỆ SCAN: `scan_frame.cloud` là hệ gốc của scan, còn
    // `map_cloud` đã được đưa về hệ scan ngay ở bước gộp (`relative_pose` = scan⁻¹·map).
    // Đây đúng là chỗ bug cũ nằm: `scan_in_map_frame` đem phép "map → scan" áp lên chính
    // scan, tức áp nghịch đảo của thứ cần áp, nên hai ảnh ở hai hệ chẳng liên quan.
    for (size_t li = 0; li < n_levels; ++li)
    {
        const int h = levels[li].height;
        const int w = levels[li].width;

        auto scan_image = range_image::buildRangeImage(
            scan_frame.cloud, h, w, v_angle_min, v_angle_max, &scan_ground_mask);
        auto map_image = range_image::buildRangeImage(
            map_cloud, h, w, v_angle_min, v_angle_max, &map_ground_mask);

        auto discrepancy_image = discrepancy::computeDiscrepancy(scan_image, map_image, threshold);

        auto dynamic_indices = filter::getDynamicIndices(
            scan_frame.cloud, discrepancy_image, h, w, v_angle_min, v_angle_max);

        auto observed_indices = filter::getObservedIndices(
            scan_frame.cloud, map_image, h, w, v_angle_min, v_angle_max);

        for (int idx : dynamic_indices)
            vote_count[li][idx]++;
        for (int idx : observed_indices)
            observed_count[li][idx]++;

        // `map_idx` đã ra khỏi phạm vi cùng vòng gộp. In số NGUỒN đã gộp thay cho một chỉ
        // số lẻ, vì từ đây trở đi chỉ còn một map duy nhất.
        std::cout << "  vs map tich luy (" << map_indices.size() << " nguon) @"
                  << levels[li].name << ": "
                  << dynamic_indices.size() << " dynamic candidates, "
                  << observed_indices.size() << " observed\n";
    }

    // ---- DEBUG sanity-check observed_count, giờ theo TỪNG thang phân giải ----
    // (mẫu số voting phải hợp lý ở mọi thang: không được toàn 0 hoặc toàn N)
    {
        // Mẫu số giờ là SỐ LƯỢT SO SÁNH chứ không phải số frame nguồn: sau E2 mọi nguồn
        // đã gộp thành một map duy nhất nên mỗi thang chỉ có 1 lượt. Để nguyên
        // `map_indices.size()` thì `count==N` luôn in 0% và trông y như pipeline hỏng.
        const int n_maps = 1;
        std::cout << "\n[DEBUG] observed_count stats over " << n_points
                  << " points (N=" << n_maps << " luot so sanh, tu "
                  << map_indices.size() << " nguon da gop):\n";
        for (size_t li = 0; li < n_levels; ++li)
        {
            int min_obs = n_maps, max_obs = 0;
            long sum_obs = 0;
            int count_zero = 0, count_full = 0;
            for (int v : observed_count[li])
            {
                min_obs = std::min(min_obs, v);
                max_obs = std::max(max_obs, v);
                sum_obs += v;
                if (v == 0) count_zero++;
                if (v == n_maps) count_full++;
            }
            std::cout << "  @" << levels[li].name
                      << ": min=" << min_obs << " max=" << max_obs
                      << " mean=" << (double)sum_obs / n_points
                      << "  count==0: " << (100.0 * count_zero / n_points) << "%"
                      << "  count==N: " << (100.0 * count_full / n_points) << "%\n";
        }
    }
    // ---- END DEBUG ----

    // Voting: per-point ratio = vote_count[i] / observed_count[i] (mẫu số động,
    // thay cho map_indices.size() cố định). Điểm nào observed_count == 0 (không map
    // nào quan sát được) thì BỎ QUA, không loại điểm đó (an toàn, tránh false dynamic).
    // Điểm i có bị kết luận dynamic ở thang `li` không?
    // observed_count == 0 nghĩa là KHÔNG map nào quan sát được điểm đó ở thang này
    // -> "không có ý kiến", tính là KHÔNG xác nhận (an toàn: thà revert còn hơn xoá oan).
    auto is_dynamic_at = [&](size_t li, size_t i) -> bool
    {
        if (observed_count[li][i] == 0) return false;
        return static_cast<float>(vote_count[li][i]) /
               static_cast<float>(observed_count[li][i]) > vote_threshold;
    };

    std::vector<int> final_dynamic_indices;
    int skipped_ground = 0;
    int n_candidates = 0;   // qua được bước REMOVE
    int n_reverted = 0;     // bị bước REVERT trả về static
    for (size_t i = 0; i < n_points; ++i)
    {
        // Điểm ground KHÔNG BAO GIỜ là dynamic. Trước đây ground chỉ bị loại khỏi
        // range image (exclude_mask của buildRangeImage) nhưng vẫn nằm trong danh
        // sách ứng viên: getDynamicIndices duyệt MỌI điểm rồi tra
        // discrepancy_image[row][col], nên 1 điểm mặt đường vẫn bị gán dynamic theo
        // phán quyết của pixel mà nó chia sẻ với vật khác. Đo trên seq04: 20.7% số
        // điểm bị báo dynamic nằm trong mask ground, trong đó 724 FP / chỉ 21 TP.
        // Xem HANDOFF_VIEC4.md mục 12.
        if (scan_ground_mask[i]) { skipped_ground++; continue; }

        // --- REMOVE: thang mịn quyết định ứng viên (nhạy, chấp nhận nhiều báo nhầm) ---
        if (!is_dynamic_at(0, i)) continue;
        n_candidates++;

        // --- REVERT: MỌI thang thô hơn đều phải XÁC NHẬN thì ứng viên mới được giữ ---
        // Đòi hỏi xác nhận ở TẤT CẢ các thang chứ không phải chỉ 1 thang: revert là
        // bước khẳng định nên bằng chứng phải chặt. Đo trên seq04 (5 scan):
        // luật "tất cả" cho F1=0.630, luật "ít nhất 1 thang" chỉ 0.492.
        bool confirmed = true;
        for (size_t li = 1; li < n_levels; ++li)
        {
            if (!is_dynamic_at(li, i)) { confirmed = false; break; }
        }
        if (!confirmed) { n_reverted++; continue; }

        final_dynamic_indices.push_back(static_cast<int>(i));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filter::splitCloud(scan_frame.cloud, final_dynamic_indices, static_cloud, dynamic_cloud);

    std::cout << "\n=== Kết quả (remove @" << levels[0].name << ", revert @ "
              << (n_levels - 1) << " thang thô hơn, vote ratio > " << vote_threshold << ") ===\n";
    std::cout << "static points:  " << static_cloud->points.size()  << '\n';
    std::cout << "dynamic points: " << dynamic_cloud->points.size() << '\n';
    std::cout << "  ép ground = static:         " << skipped_ground << '\n';
    std::cout << "  ứng viên sau REMOVE:        " << n_candidates << '\n';
    std::cout << "  bị REVERT (không xác nhận): " << n_reverted
              << " (" << (n_candidates ? 100.0 * n_reverted / n_candidates : 0.0) << "%)\n";

    std::string out_path = "dynamic_indices_scan" + std::to_string(scan_idx) + ".txt";
    std::ofstream out_file(out_path);
    for (int idx : final_dynamic_indices)
        out_file << idx << '\n';
    out_file.close();
    std::cout << "Đã ghi " << final_dynamic_indices.size()
              << " dynamic indices vào " << out_path << '\n';

    return 0;
}