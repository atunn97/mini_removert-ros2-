#include "filter/filter.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_set>

namespace filter
{

std::vector<int> getDynamicIndices(
    const pcl::PointCloud<PointT>::Ptr& scan_cloud,
    const RangeImage& discrepancy_image,
    int height,
    int width,
    float vertical_angle_min_deg,
    float vertical_angle_max_deg)
{
    std::vector<int> dynamic_indices;
    float v_min = vertical_angle_min_deg * M_PI / 180.0f;
    float v_max = vertical_angle_max_deg * M_PI / 180.0f;

    for (int i = 0; i < (int)scan_cloud->points.size(); ++i)
    {
        auto pixel = range_image::projectToPixel(scan_cloud->points[i], height, width, v_min, v_max);
        if (!pixel.valid) continue;

        float disc = discrepancy_image[pixel.row][pixel.col];
        if (std::isfinite(disc) && disc > 0.0f)
            dynamic_indices.push_back(i);
    }
    return dynamic_indices;
}

std::vector<int> getObservedIndices(
    const pcl::PointCloud<PointT>::Ptr& scan_cloud,
    const RangeImage& map_image,
    int height,
    int width,
    float vertical_angle_min_deg,
    float vertical_angle_max_deg)
{
    std::vector<int> observed_indices;
    float v_min = vertical_angle_min_deg * M_PI / 180.0f;
    float v_max = vertical_angle_max_deg * M_PI / 180.0f;

    for (int i = 0; i < (int)scan_cloud->points.size(); ++i)
    {
        auto pixel = range_image::projectToPixel(scan_cloud->points[i], height, width, v_min, v_max);
        if (!pixel.valid) continue;

        float map_range = map_image[pixel.row][pixel.col];
        if (std::isfinite(map_range))
            observed_indices.push_back(i);
    }
    return observed_indices;
}

void splitCloud(
    const pcl::PointCloud<PointT>::Ptr& scan_cloud,
    const std::vector<int>& dynamic_indices,
    pcl::PointCloud<PointT>::Ptr& static_cloud,
    pcl::PointCloud<PointT>::Ptr& dynamic_cloud)
{
    std::unordered_set<int> dynamic_set(dynamic_indices.begin(), dynamic_indices.end());

    for (int i = 0; i < (int)scan_cloud->points.size(); ++i)
    {
        if (dynamic_set.count(i))
            dynamic_cloud->points.push_back(scan_cloud->points[i]);
        else
            static_cloud->points.push_back(scan_cloud->points[i]);
    }
}

} // namespace filter