#include "range_image/range_image.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

namespace range_image
{

PixelCoord projectToPixel(
    const PointT& point,
    int height,
    int width,
    float v_min,
    float v_max)
{
    PixelCoord result;

    float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (distance < 0.1f) return result; // valid = false

    float vertical_angle = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));
    float horizontal_angle = std::atan2(point.y, point.x);

    if (vertical_angle < v_min || vertical_angle > v_max) return result; // valid = false

    float v_span = v_max - v_min;
    int row = static_cast<int>((vertical_angle - v_min) / v_span * height);
    int col = static_cast<int>((horizontal_angle + M_PI) / (2.0f * M_PI) * width);
    row = std::clamp(row, 0, height - 1);
    col = std::clamp(col, 0, width - 1);

    result.row = row;
    result.col = col;
    result.distance = distance;
    result.valid = true;
    return result;
}

RangeImage buildRangeImage(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    int height,
    int width,
    float vertical_angle_min_deg,
    float vertical_angle_max_deg,
    const std::vector<bool>* exclude_mask)
{
    RangeImage range_image(height, std::vector<float>(width, std::numeric_limits<float>::infinity()));

    float v_min = vertical_angle_min_deg * M_PI / 180.0f;
    float v_max = vertical_angle_max_deg * M_PI / 180.0f;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (exclude_mask && (*exclude_mask)[i]) continue;

        PixelCoord pixel = projectToPixel(cloud->points[i], height, width, v_min, v_max);
        if (!pixel.valid) continue;

        range_image[pixel.row][pixel.col] = std::min(range_image[pixel.row][pixel.col], pixel.distance);
    }

    return range_image;
}

} // namespace range_image