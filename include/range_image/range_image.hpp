#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace range_image
{
    using PointT = pcl::PointXYZ;

    using RangeImage =
        std::vector<std::vector<float>>;

    // Kết quả chiếu 1 điểm 3D xuống pixel (row, col) của range image.
    // valid = false nếu điểm quá gần (< 0.1m) hoặc nằm ngoài FOV dọc [v_min, v_max].
    struct PixelCoord
    {
        int row = 0;
        int col = 0;
        float distance = 0.0f;
        bool valid = false;
    };

    // Dùng chung cho buildRangeImage (range_image.cpp) và các hàm trong filter.cpp
    // (getDynamicIndices hiện tại, getObservedIndices mới).
    // v_min, v_max: góc dọc tối thiểu/tối đa, đơn vị RADIAN (đã convert từ độ).
    PixelCoord projectToPixel(
        const PointT& point,
        int height,
        int width,
        float v_min,
        float v_max);

    RangeImage buildRangeImage(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        int height,
        int width,
        float vertical_angle_min_deg,
        float vertical_angle_max_deg,
        const std::vector<bool>* exclude_mask = nullptr);
}