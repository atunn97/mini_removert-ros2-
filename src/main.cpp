#include "discrepancy/discrepancy.hpp"
#include "range_image/range_image.hpp"
#include "io/pcd_loader.hpp"
#include "io/pose_loader.hpp"
#include "io/calib_loader.hpp"
#include "filter/filter.hpp"
#include "transform/transform.hpp"
#include "ground_filter/ground_filter.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>

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

    std::vector<int> map_indices;
    for (int i = 6; i < argc; ++i)
        map_indices.push_back(std::stoi(argv[i]));

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

    constexpr int height = 64;
    constexpr int width = 900;
    constexpr float v_angle_min = -24.8f;
    constexpr float v_angle_max = 2.0f;

    // Đếm số lần mỗi điểm trong scan bị đánh dấu "dynamic" qua các lần so sánh
    std::vector<int> vote_count(scan_frame.cloud->points.size(), 0);
    // Đếm số lần mỗi điểm THỰC SỰ được quan sát bởi 1 map (mẫu số động, thay cho map_indices.size() cố định)
    std::vector<int> observed_count(scan_frame.cloud->points.size(), 0);

    // Ground mask của scan: tính MỘT LẦN ngoài vòng lặp, không tính lại mỗi map.
    // Ground membership bất biến qua phép biến đổi cứng (phép quay/tịnh tiến bảo toàn
    // khoảng cách điểm→mặt phẳng), nên mask trên scan_frame.cloud và trên
    // scan_in_map_frame là như nhau — tính lại mỗi vòng vừa thừa vừa gây thiếu nhất
    // quán (RANSAC có yếu tố ngẫu nhiên). Thứ tự điểm không đổi nên index vẫn khớp.
    auto scan_ground_mask = ground_filter::detectGroundMask(scan_frame.cloud);

    for (int map_idx : map_indices)
    {
        snprintf(buf, sizeof(buf), "%06d.pcd", map_idx);
        std::string map_path = pcd_dir + "/" + buf;
        Eigen::Matrix4f map_pose = poses[map_idx] * Tr;
        Frame map_frame = loadFrame(map_path, map_pose, 0.0);

        Eigen::Matrix4f relative_pose = map_frame.pose.inverse() * scan_frame.pose;
        auto scan_in_map_frame = transform::transformPointCloud(scan_frame.cloud, relative_pose);

        auto map_ground_mask = ground_filter::detectGroundMask(map_frame.cloud);

        auto scan_image = range_image::buildRangeImage(
            scan_in_map_frame, height, width, v_angle_min, v_angle_max, &scan_ground_mask);
        auto map_image = range_image::buildRangeImage(
            map_frame.cloud, height, width, v_angle_min, v_angle_max, &map_ground_mask);

        auto discrepancy_image = discrepancy::computeDiscrepancy(scan_image, map_image, threshold);

        auto dynamic_indices = filter::getDynamicIndices(
            scan_in_map_frame, discrepancy_image, height, width, v_angle_min, v_angle_max);

        auto observed_indices = filter::getObservedIndices(
            scan_in_map_frame, map_image, height, width, v_angle_min, v_angle_max);

        for (int idx : dynamic_indices)
            vote_count[idx]++;
        for (int idx : observed_indices)
            observed_count[idx]++;

        std::cout << "  vs map[" << map_idx << "]: " << dynamic_indices.size() << " dynamic candidates, "
                   << observed_indices.size() << " observed\n";
    }

    // ---- DEBUG sanity-check observed_count (tạm thời, để kiểm tra bug 3 checklist) ----
    {
        int n_maps = static_cast<int>(map_indices.size());
        int min_obs = n_maps, max_obs = 0;
        long sum_obs = 0;
        int count_zero = 0, count_full = 0;
        for (int v : observed_count)
        {
            min_obs = std::min(min_obs, v);
            max_obs = std::max(max_obs, v);
            sum_obs += v;
            if (v == 0) count_zero++;
            if (v == n_maps) count_full++;
        }
        double mean_obs = observed_count.empty() ? 0.0 : (double)sum_obs / observed_count.size();
        std::cout << "\n[DEBUG] observed_count stats over " << observed_count.size() << " points (N=" << n_maps << " maps):\n"
                  << "  min=" << min_obs << " max=" << max_obs << " mean=" << mean_obs << "\n"
                  << "  count==0: " << count_zero << " (" << (100.0 * count_zero / observed_count.size()) << "%)\n"
                  << "  count==N: " << count_full << " (" << (100.0 * count_full / observed_count.size()) << "%)\n";
    }
    // ---- END DEBUG ----

    // Voting: per-point ratio = vote_count[i] / observed_count[i] (mẫu số động,
    // thay cho map_indices.size() cố định). Điểm nào observed_count == 0 (không map
    // nào quan sát được) thì BỎ QUA, không loại điểm đó (an toàn, tránh false dynamic).
    constexpr float vote_threshold = 0.5f;
    std::vector<int> final_dynamic_indices;
    int skipped_ground = 0;
    for (size_t i = 0; i < vote_count.size(); ++i)
    {
        // Điểm ground KHÔNG BAO GIỜ là dynamic. Trước đây ground chỉ bị loại khỏi
        // range image (exclude_mask của buildRangeImage) nhưng vẫn nằm trong danh
        // sách ứng viên: getDynamicIndices duyệt MỌI điểm rồi tra
        // discrepancy_image[row][col], nên 1 điểm mặt đường vẫn bị gán dynamic theo
        // phán quyết của pixel mà nó chia sẻ với vật khác. Đo trên seq04: 20.7% số
        // điểm bị báo dynamic nằm trong mask ground, trong đó 724 FP / chỉ 21 TP.
        // Xem HANDOFF_VIEC4.md mục 12.
        if (scan_ground_mask[i]) { skipped_ground++; continue; }

        if (observed_count[i] == 0) continue; // không đủ dữ liệu để kết luận -> giữ nguyên (static)

        float ratio = static_cast<float>(vote_count[i]) / static_cast<float>(observed_count[i]);
        if (ratio > vote_threshold)
            final_dynamic_indices.push_back(static_cast<int>(i));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filter::splitCloud(scan_frame.cloud, final_dynamic_indices, static_cloud, dynamic_cloud);

    std::cout << "\n=== Kết quả sau voting (ratio > " << vote_threshold << ", mẫu số = observed_count per-point) ===\n";
    std::cout << "static points:  " << static_cloud->points.size()  << '\n';
    std::cout << "dynamic points: " << dynamic_cloud->points.size() << '\n';
    std::cout << "(đã ép " << skipped_ground << " điểm ground = static, không xét dynamic)\n";

    std::string out_path = "dynamic_indices_scan" + std::to_string(scan_idx) + ".txt";
    std::ofstream out_file(out_path);
    for (int idx : final_dynamic_indices)
        out_file << idx << '\n';
    out_file.close();
    std::cout << "Đã ghi " << final_dynamic_indices.size()
              << " dynamic indices vào " << out_path << '\n';

    return 0;
}