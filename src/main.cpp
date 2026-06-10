#include "discrepancy/discrepancy.hpp"
#include "range_image/range_image.hpp"
#include "io/pcd_loader.hpp"
#include "filter/filter.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
    using PointT = pcl::PointXYZ;

    pcl::PointCloud<PointT>::Ptr scan_cloud(
        new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr map_cloud(
        new pcl::PointCloud<PointT>);

    PointT scan_p1;
    scan_p1.x = 10;
    scan_p1.y = 0;
    scan_p1.z = 0;

    PointT scan_p2;
    scan_p2.x = 10;
    scan_p2.y = 1;
    scan_p2.z = 0;

    PointT map_p1;
    map_p1.x = 9;
    map_p1.y = 0;
    map_p1.z = 0;

    PointT map_p2;
    map_p2.x = 10;
    map_p2.y = 1;
    map_p2.z = 0;
    // Thêm điểm chỉ có trong scan, không có trong map
    PointT scan_p3;
    scan_p3.x = 5; scan_p3.y = 5; scan_p3.z = 0;
    scan_cloud->points.push_back(scan_p3);
    // map không có điểm tương ứng → pixel đó phải là infinity (unknown)

    scan_cloud->points.push_back(scan_p1);
    scan_cloud->points.push_back(scan_p2);

    map_cloud->points.push_back(map_p1);
    map_cloud->points.push_back(map_p2);
    
    constexpr int height = 16;
    constexpr int width = 32;
    constexpr float vertical_fov_deg = 60.0f;
    constexpr float horizontal_fov_deg = 90.0f;
    constexpr float threshold = 0.5f;

    auto scan_image = range_image::buildRangeImage(
        scan_cloud,
        height,
        width,
        vertical_fov_deg,
        horizontal_fov_deg);

    auto map_image = range_image::buildRangeImage(
        map_cloud,
        height,
        width,
        vertical_fov_deg,
        horizontal_fov_deg);

    auto discrepancy_image = discrepancy::computeDiscrepancy(
        scan_image,
        map_image,
        threshold);

    int diff_count = 0;
    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col)
        {
            float diff = discrepancy_image[row][col];
            if (std::isfinite(diff))
            {
                std::cout << "discrepancy at (" << row << ", " << col << ") = "
                          << std::fixed << std::setprecision(3)
                          << diff << '\n';
                ++diff_count;
            }
        }
    }
     auto dynamic_indices = filter::getDynamicIndices(
        scan_cloud,
        discrepancy_image,                              // ← cái này mới đúng
        height, width, vertical_fov_deg);
    pcl::PointCloud<PointT>::Ptr static_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr dynamic_cloud(new pcl::PointCloud<PointT>);   

    filter::splitCloud(scan_cloud, dynamic_indices, static_cloud, dynamic_cloud);
    std::cout << "scan points: " << scan_cloud->points.size() << '\n';
    std::cout << "map points: " << map_cloud->points.size() << '\n';
    std::cout << "discrepancy count: " << diff_count << '\n';
    std::cout << "static points:  " << static_cloud->points.size()  << '\n';
    std::cout << "dynamic points: " << dynamic_cloud->points.size() << '\n';
    return 0;
}
