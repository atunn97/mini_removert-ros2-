#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

inline std::vector<Eigen::Matrix4f> loadKittiPoses(const std::string& path)
{
    std::vector<Eigen::Matrix4f> poses;
    std::ifstream file(path);
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j)
                ss >> T(i, j);
        poses.push_back(T);
    }
    return poses;
}
