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

    for (int map_idx : map_indices)
    {
        snprintf(buf, sizeof(buf), "%06d.pcd", map_idx);
        std::string map_path = pcd_dir + "/" + buf;
        Eigen::Matrix4f map_pose = poses[map_idx] * Tr;
        Frame map_frame = loadFrame(map_path, map_pose, 0.0);

        Eigen::Matrix4f relative_pose = map_frame.pose.inverse() * scan_frame.pose;
        auto scan_in_map_frame = transform::transformPointCloud(scan_frame.cloud, relative_pose);

        auto scan_ground_mask = ground_filter::detectGroundMask(scan_in_map_frame);
        auto map_ground_mask  = ground_filter::detectGroundMask(map_frame.cloud);

        auto scan_image = range_image::buildRangeImage(
            scan_in_map_frame, height, width, v_angle_min, v_angle_max, &scan_ground_mask);
        auto map_image = range_image::buildRangeImage(
            map_frame.cloud, height, width, v_angle_min, v_angle_max, &map_ground_mask);

        auto discrepancy_image = discrepancy::computeDiscrepancy(scan_image, map_image, threshold);

        auto dynamic_indices = filter::getDynamicIndices(
            scan_in_map_frame, discrepancy_image, height, width, v_angle_min, v_angle_max);

        for (int idx : dynamic_indices)
            vote_count[idx]++;

        std::cout << "  vs map[" << map_idx << "]: " << dynamic_indices.size() << " dynamic candidates\n";
    }

    // Voting: chỉ giữ điểm bị đánh dấu dynamic ở ĐA SỐ (>50%) các lần so sánh
    int min_votes = static_cast<int>(map_indices.size()) / 2 + 1;
    std::vector<int> final_dynamic_indices;
    for (size_t i = 0; i < vote_count.size(); ++i)
        if (vote_count[i] >= min_votes)
            final_dynamic_indices.push_back(static_cast<int>(i));

    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filter::splitCloud(scan_frame.cloud, final_dynamic_indices, static_cloud, dynamic_cloud);

    std::cout << "\n=== Kết quả sau voting (cần >= " << min_votes << "/" << map_indices.size() << " lần xác nhận) ===\n";
    std::cout << "static points:  " << static_cloud->points.size()  << '\n';
    std::cout << "dynamic points: " << dynamic_cloud->points.size() << '\n';

    std::string out_path = "dynamic_indices_scan" + std::to_string(scan_idx) + ".txt";
    std::ofstream out_file(out_path);
    for (int idx : final_dynamic_indices)
        out_file << idx << '\n';
    out_file.close();
    std::cout << "Đã ghi " << final_dynamic_indices.size()
              << " dynamic indices vào " << out_path << '\n';

    return 0;
}