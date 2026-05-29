#include "test_common.hpp"

#include <filesystem>
#include <functional>
#include <stdexcept>

#include "pointcloud_pipeline/io.hpp"

using pointcloud_pipeline::loadKittiBin;

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
}

TEST(Io, RejectsMissingKittiBin) {
    EXPECT_TRUE(throwsRuntimeError(
        [] { (void)loadKittiBin("tests/fixtures/does_not_exist.bin"); }));
}
