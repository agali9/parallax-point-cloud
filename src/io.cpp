#include "pointcloud_pipeline/io.hpp"

#include <cstdint>
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

    input.seekg(0, std::ios::end);
    const auto file_size = static_cast<std::uintmax_t>(input.tellg());
    input.seekg(0, std::ios::beg);

    constexpr std::uintmax_t kRecordBytes = 16U;  // x, y, z, intensity as float32
    if (file_size % kRecordBytes != 0U) {
        throw std::runtime_error("truncated KITTI bin (size not multiple of 16): " +
                                 path.string());
    }

    const std::size_t point_count = static_cast<std::size_t>(file_size / kRecordBytes);
    std::vector<PointXYZ> cloud;
    cloud.reserve(point_count);

    for (std::size_t i = 0; i < point_count; ++i) {
        float record[4];
        input.read(reinterpret_cast<char*>(record), static_cast<std::streamsize>(kRecordBytes));
        if (!input) {
            throw std::runtime_error("failed reading KITTI bin: " + path.string());
        }
        cloud.push_back(PointXYZ{record[0], record[1], record[2]});
    }

    return cloud;
}

}  // namespace pointcloud_pipeline
