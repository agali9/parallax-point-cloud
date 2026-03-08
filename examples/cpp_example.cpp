#include <iostream>
#include <vector>

#include "pointcloud_pipeline/types.hpp"

int main() {
    std::vector<pointcloud_pipeline::PointXYZ> cloud{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};
    std::cout << "Prototype cloud has " << cloud.size() << " points\n";
    return 0;
}