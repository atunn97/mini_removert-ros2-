#include "transform.hpp"

namespace transform
{

pcl::PointCloud<PointT>::Ptr transformPointCloud(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const Eigen::Matrix4f& T)
{
    pcl::PointCloud<PointT>::Ptr output(
        new pcl::PointCloud<PointT>);

    output->points.reserve(cloud->points.size());

    for(const auto& p : cloud->points)
    {
        Eigen::Vector4f pt;

        pt << p.x, p.y, p.z, 1.0f;

        Eigen::Vector4f transformed = T * pt;

        PointT new_point;

        new_point.x = transformed(0);
        new_point.y = transformed(1);
        new_point.z = transformed(2);

        output->points.push_back(new_point);
    }

    return output;
}

}