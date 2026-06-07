#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

struct Frame
{
    double timestamp;

    Eigen::Matrix4f pose;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};