#include "discrepancy/discrepancy.hpp"
#include "range_image/range_image.hpp"
#include "io/pcd_loader.hpp"
#include "io/pose_loader.hpp"
#include "io/calib_loader.hpp"
#include "filter/filter.hpp"
#include "transform/transform.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc < 6)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <pcd_dir> <poses.txt> <calib.txt> <scan_idx> <map_idx> [threshold]\n";
        return 1;
    }

    std::string pcd_dir   = argv[1];
    std::string poses_path = argv[2];
    std::string calib_path = argv[3];
    int scan_idx = std::stoi(argv[4]);
    int map_idx  = std::stoi(argv[5]);
    float threshold = (argc >= 7) ? std::stof(argv[6]) : 0.5f;

    auto poses = loadKittiPoses(poses_path);
    Eigen::Matrix4f Tr = loadKittiTr(calib_path);

    char buf[32];
    snprintf(buf, sizeof(buf), "%06d.pcd", scan_idx);
    std::string scan_path = pcd_dir + "/" + buf;
    snprintf(buf, sizeof(buf), "%06d.pcd", map_idx);
    std::string map_path = pcd_dir + "/" + buf;

    // Pose LiDAR = pose camera (world<-cam) * Tr (cam<-lidar)
    Eigen::Matrix4f scan_pose = poses[scan_idx] * Tr;
    Eigen::Matrix4f map_pose  = poses[map_idx]  * Tr;

    Frame scan_frame = loadFrame(scan_path, scan_pose, 0.0);
    Frame map_frame  = loadFrame(map_path,  map_pose,  0.0);

    std::cout << "Loaded scan [" << scan_idx << "]: " << scan_frame.cloud->points.size() << " points\n";
    std::cout << "Loaded map  [" << map_idx  << "]: " << map_frame.cloud->points.size()  << " points\n";

    Eigen::Matrix4f relative_pose = map_frame.pose.inverse() * scan_frame.pose;
    auto scan_in_map_frame = transform::transformPointCloud(scan_frame.cloud, relative_pose);

    constexpr int height = 16;
    constexpr int width = 32;
    constexpr float vertical_fov_deg = 26.9f;   // Velodyne HDL-64E thực tế ~26.9° (KITTI dùng loại này)
    constexpr float horizontal_fov_deg = 360.0f;

    auto scan_image = range_image::buildRangeImage(scan_in_map_frame, height, width, vertical_fov_deg, horizontal_fov_deg);
    auto map_image  = range_image::buildRangeImage(map_frame.cloud, height, width, vertical_fov_deg, horizontal_fov_deg);

    auto discrepancy_image = discrepancy::computeDiscrepancy(scan_image, map_image, threshold);

    int diff_count = 0;
    for (int row = 0; row < height; ++row)
        for (int col = 0; col < width; ++col)
            if (std::isfinite(discrepancy_image[row][col]) && discrepancy_image[row][col] > 0.0f)
                ++diff_count;

    auto dynamic_indices = filter::getDynamicIndices(scan_in_map_frame, discrepancy_image, height, width, vertical_fov_deg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filter::splitCloud(scan_in_map_frame, dynamic_indices, static_cloud, dynamic_cloud);

    std::cout << "discrepancy count: " << diff_count << '\n';
    std::cout << "static points:  " << static_cloud->points.size()  << '\n';
    std::cout << "dynamic points: " << dynamic_cloud->points.size() << '\n';

    return 0;
}