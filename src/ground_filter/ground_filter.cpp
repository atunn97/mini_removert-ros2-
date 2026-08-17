#include "ground_filter/ground_filter.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <random>
#include <numeric>
#include <algorithm>
namespace ground_filter
{

std::vector<bool> detectGroundMask(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    int seed,
    float distance_threshold)
{
    std::vector<bool> is_ground(cloud->points.size(), false);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setMaxIterations(200);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(cloud);
    // xáo shuffled_indices_ → PCL bốc bộ 3 khác → mặt phẳng hợp lệ khác;
    if (seed != 0)
    {   pcl::IndicesPtr v(new pcl::Indices(cloud->size()));
        std::iota(v->begin(), v->end(), 0);
        std::mt19937 rng(seed);
        std::shuffle(v->begin(), v->end(), rng); 
        seg.setIndices(v);}
    seg.segment(*inliers, *coefficients);
    for (int idx : inliers->indices)
        is_ground[idx] = true;

    return is_ground;
}

} // namespace ground_filter