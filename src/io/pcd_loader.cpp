#include "io/pcd_loader.hpp"

#include <pcl/io/pcd_io.h>

Frame loadFrame(
    const std::string& pcd_path,
    const Eigen::Matrix4f& pose,
    double timestamp)
{
    Frame frame;

    frame.timestamp = timestamp;

    frame.pose = pose;

    frame.cloud.reset(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile(
            pcd_path,
            *frame.cloud) == -1)
    {
        throw std::runtime_error(
            "Failed to load PCD file");
    }

    return frame;
}