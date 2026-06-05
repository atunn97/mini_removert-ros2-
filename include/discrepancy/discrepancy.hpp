#pragma once

#include "range_image/range_image.hpp"

namespace discrepancy
{
    using RangeImage = range_image::RangeImage;

    RangeImage computeDiscrepancy(
        const RangeImage& scan_image,
        const RangeImage& map_image,
        float threshold);
}