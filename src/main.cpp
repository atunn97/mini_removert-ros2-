#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include "transform/transform.hpp"

int main()
{
    using PointT = pcl::PointXYZ;

    pcl::PointCloud<PointT>::Ptr cloud(
        new pcl::PointCloud<PointT>);

    PointT p;

    p.x = 1.0;
    p.y = 2.0;
    p.z = 3.0;

    cloud->points.push_back(p);

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    T(0,3) = 10.0f;

    auto transformed =
        transform::transformPointCloud(cloud, T);

    std::cout << transformed->points[0].x << std::endl;

    return 0;
}