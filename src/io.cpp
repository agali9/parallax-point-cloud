#include "pointcloud_pipeline/io.hpp"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pointcloud_pipeline {

std::vector<PointXYZ> loadKittiBin(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open KITTI bin: " + path.string());
    }

    std::vector<PointXYZ> cloud;
    float record[4];
    while (input.read(reinterpret_cast<char*>(record), sizeof(record))) {
        cloud.push_back(PointXYZ{record[0], record[1], record[2]});
    }
    return cloud;
}

}  // namespace pointcloud_pipeline
