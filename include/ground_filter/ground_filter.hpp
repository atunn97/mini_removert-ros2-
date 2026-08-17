#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace ground_filter
{
    using PointT = pcl::PointXYZ;

    // Trả về mask cùng kích thước với cloud->points:
    // true = điểm thuộc mặt đường (ground), false = không phải ground
    std::vector<bool> detectGroundMask(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        int seed=0,
        float distance_threshold = 0.2f);
}