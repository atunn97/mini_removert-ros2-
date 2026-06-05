#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace range_image
{
    using PointT = pcl::PointXYZ;

    using RangeImage =
        std::vector<std::vector<float>>;

    RangeImage buildRangeImage(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        int height,
        int width,
        float vertical_fov_deg,
        float horizontal_fov_deg);
}