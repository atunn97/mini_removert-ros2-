#include "ground_filter/ground_filter.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

namespace ground_filter
{

std::vector<bool> detectGroundMask(
    const pcl::PointCloud<PointT>::Ptr& cloud,
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
    seg.segment(*inliers, *coefficients);

    for (int idx : inliers->indices)
        is_ground[idx] = true;

    return is_ground;
}

} // namespace ground_filter