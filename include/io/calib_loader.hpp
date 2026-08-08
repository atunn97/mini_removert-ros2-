#pragma once
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>

// calib.txt của KITTI odometry có dòng "Tr: r11 r12 r13 tx ... r33 tz"
// đây là ma trận 3x4 biến đổi từ LiDAR frame sang camera (cam0) frame
inline Eigen::Matrix4f loadKittiTr(const std::string& calib_path)
{
    std::ifstream file(calib_path);
    std::string line;
    while (std::getline(file, line))
    {
        if (line.rfind("Tr:", 0) == 0)
        {
            std::istringstream ss(line.substr(3));
            Eigen::Matrix4f Tr = Eigen::Matrix4f::Identity();
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 4; ++j)
                    ss >> Tr(i, j);
            return Tr;
        }
    }
    throw std::runtime_error("Tr not found in calib file: " + calib_path);
}
