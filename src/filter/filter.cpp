#include "filter/filter.hpp"
#include <cmath>
#include <unordered_set>

namespace filter
{

std::vector<int> getDynamicIndices(
    const pcl::PointCloud<PointT>::Ptr& scan_cloud,
    const RangeImage& discrepancy_image,
    int height,
    int width,
    float vertical_fov_deg)
{
    std::vector<int> dynamic_indices;
    float vertical_fov_rad = vertical_fov_deg * M_PI / 180.0f;

    for (int i = 0; i < (int)scan_cloud->points.size(); ++i)
    {
        const auto& point = scan_cloud->points[i];

        float distance = std::sqrt(
            point.x * point.x +
            point.y * point.y +
            point.z * point.z);

        if (distance < 0.1f) continue;

        // Tính lại row/col — phải dùng công thức giống hệt buildRangeImage
        float vertical_angle = std::atan2(
            point.z,
            std::sqrt(point.x * point.x + point.y * point.y));

        float horizontal_angle = std::atan2(point.y, point.x);

        int row = static_cast<int>(
            (vertical_angle + vertical_fov_rad / 2)
            / vertical_fov_rad * height);

        int col = static_cast<int>(
            (horizontal_angle + M_PI) / (2.0f * M_PI) * width);

        row = std::clamp(row, 0, height - 1);
        col = std::clamp(col, 0, width - 1);

        // Pixel có giá trị finite và > 0 nghĩa là dynamic
        float disc = discrepancy_image[row][col];
        if (std::isfinite(disc) && disc > 0.0f)
        {
            dynamic_indices.push_back(i);
        }
    }

    return dynamic_indices;
}

void splitCloud(
    const pcl::PointCloud<PointT>::Ptr& scan_cloud,
    const std::vector<int>& dynamic_indices,
    pcl::PointCloud<PointT>::Ptr& static_cloud,
    pcl::PointCloud<PointT>::Ptr& dynamic_cloud)
{
    // Dùng set để lookup O(1)
    std::unordered_set<int> dynamic_set(
        dynamic_indices.begin(),
        dynamic_indices.end());

    for (int i = 0; i < (int)scan_cloud->points.size(); ++i)
    {
        if (dynamic_set.count(i))
            dynamic_cloud->points.push_back(scan_cloud->points[i]);
        else
            static_cloud->points.push_back(scan_cloud->points[i]);
    }
}

} // namespace filter