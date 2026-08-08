#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "range_image/range_image.hpp"

namespace filter
{
using PointT = pcl::PointXYZ;
using RangeImage = range_image::RangeImage;

// Tìm index của những điểm dynamic trong scan_cloud
std::vector<int> getDynamicIndices(
    const pcl::PointCloud<PointT>::Ptr& scan_cloud,
    const RangeImage& discrepancy_image,
    int height,
    int width,
    float vertical_angle_min_deg,
    float vertical_angle_max_deg);

// Tách scan_cloud thành static và dynamic
void splitCloud(
    const pcl::PointCloud<PointT>::Ptr& scan_cloud,
    const std::vector<int>& dynamic_indices,
    pcl::PointCloud<PointT>::Ptr& static_cloud,
    pcl::PointCloud<PointT>::Ptr& dynamic_cloud);

} // namespace filter