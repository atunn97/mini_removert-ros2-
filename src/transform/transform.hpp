#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

namespace transform
{
    using PointT = pcl::PointXYZ;

    pcl::PointCloud<PointT>::Ptr transformPointCloud(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const Eigen::Matrix4f& T);
}