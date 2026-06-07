#pragma once

#include "io/pcd_frame.hpp"

#include <string>

Frame loadFrame(
    const std::string& pcd_path,
    const Eigen::Matrix4f& pose,
    double timestamp);