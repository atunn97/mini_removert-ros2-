#include "range_image/range_image.hpp"

#include <cmath>
#include <limits>

namespace range_image
{

RangeImage buildRangeImage(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    int height,
    int width,
    float vertical_fov_deg,
    float horizontal_fov_deg)
{
    RangeImage range_image(height, std::vector<float>(width, std::numeric_limits<float>::infinity()));

    float vertical_fov_rad = vertical_fov_deg * M_PI / 180.0f;
    float horizontal_fov_rad = horizontal_fov_deg * M_PI / 180.0f;

    for (const auto& point : cloud->points)
    {
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance == 0) continue; // Skip points at the origin

        float vertical_angle = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));
        float horizontal_angle = std::atan2(point.y, point.x);

        int row = static_cast<int>((vertical_angle + vertical_fov_rad / 2) / vertical_fov_rad * height);
        int col = static_cast<int>((horizontal_angle + horizontal_fov_rad / 2) / horizontal_fov_rad * width);

        if (row >= 0 && row < height && col >= 0 && col < width)
        {
            range_image[row][col] = std::min(range_image[row][col], distance);
        }
    }

    return range_image;
}

} // namespace range_image
       