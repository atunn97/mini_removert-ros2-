#include "discrepancy/discrepancy.hpp"

#include <cmath>
#include <limits>

namespace discrepancy
{

RangeImage computeDiscrepancy(
    const RangeImage& scan_image,
    const RangeImage& map_image,
    float threshold)
{
    if (scan_image.empty() || map_image.empty())
    {
        return {};
    }

    if (scan_image.size() != map_image.size())
    {
        return {};
    }

    int height =
        static_cast<int>(scan_image.size());

    int width =
        static_cast<int>(scan_image[0].size());

    RangeImage discrepancy_image(
        height,
        std::vector<float>(
            width,
            std::numeric_limits<float>::infinity()));

    for (int row = 0; row < height; ++row)
    {
        if (scan_image[row].size() != map_image[row].size())
        {
            continue;
        }

        for (int col = 0; col < width; ++col)
        {
            float scan_range =
                scan_image[row][col];

            float map_range =
                map_image[row][col];

            if (!std::isfinite(scan_range) ||
                !std::isfinite(map_range))
            {
                continue;
            }

            float diff =
                std::fabs(scan_range - map_range);

            if (diff > threshold)
            {
                discrepancy_image[row][col] =
                    diff;
            }
            else {
                discrepancy_image[row][col] = 0.0f;     // static
            }
        }
    }

    return discrepancy_image;
}

} // namespace discrepancy