#include "test_common.hpp"

#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "pointcloud_pipeline/io.hpp"

using pointcloud_pipeline::loadKittiBin;
using pointcloud_pipeline::PointXYZ;

namespace {

std::filesystem::path fixturePath(const char* name) {
    const std::filesystem::path candidates[] = {
        std::filesystem::path("tests") / "fixtures" / name,
        std::filesystem::path("..") / "tests" / "fixtures" / name,
        std::filesystem::path("..") / ".." / "tests" / "fixtures" / name,
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return candidates[0];
}

bool throwsRuntimeError(const std::function<void()>& fn) {
    try {
        fn();
    } catch (const std::runtime_error&) {
        return true;
    } catch (...) {
        return false;
    }
    return false;
}

}  // namespace

TEST(Io, LoadsKittiBinFixture) {
    const auto path = fixturePath("kitti_tiny.bin");
    EXPECT_TRUE(std::filesystem::exists(path));

    const auto cloud = loadKittiBin(path);
    ASSERT_EQ(cloud.size(), 3U);
    EXPECT_NEAR(cloud[0].x, 1.0F, 1.0e-5F);
    EXPECT_NEAR(cloud[0].y, 2.0F, 1.0e-5F);
    EXPECT_NEAR(cloud[0].z, 3.0F, 1.0e-5F);
    EXPECT_NEAR(cloud[1].x, 4.0F, 1.0e-5F);
    EXPECT_NEAR(cloud[1].y, 5.0F, 1.0e-5F);
    EXPECT_NEAR(cloud[1].z, 6.0F, 1.0e-5F);
    EXPECT_NEAR(cloud[2].x, -1.0F, 1.0e-5F);
    EXPECT_NEAR(cloud[2].y, 0.0F, 1.0e-5F);
    EXPECT_NEAR(cloud[2].z, 1.5F, 1.0e-5F);
}

TEST(Io, RejectsTruncatedKittiBin) {
    const auto path = std::filesystem::temp_directory_path() / "kitti_truncated.bin";
    {
        std::ofstream out(path, std::ios::binary);
        const float partial[] = {1.0F, 2.0F, 3.0F};  // 12 bytes, not a full record
        out.write(reinterpret_cast<const char*>(partial), sizeof(partial));
    }

    EXPECT_TRUE(throwsRuntimeError([&] { (void)loadKittiBin(path); }));
    std::filesystem::remove(path);
}

TEST(Io, RejectsMissingKittiBin) {
    EXPECT_TRUE(throwsRuntimeError(
        [] { (void)loadKittiBin("tests/fixtures/does_not_exist.bin"); }));
}
