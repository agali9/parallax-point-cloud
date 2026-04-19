#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "pointcloud_pipeline/pipeline.hpp"

namespace pcp = pointcloud_pipeline;

namespace {

struct Stats {
    double mean_ms = 0.0;
    double median_ms = 0.0;
    double p95_ms = 0.0;
};

struct BenchmarkRow {
    std::size_t point_count = 0;
    Stats baseline;
    Stats downsampled;
    double speedup = 0.0;
};

std::vector<pcp::PointXYZ> makeSyntheticLidarCloud(std::size_t point_count) {
    std::vector<pcp::PointXYZ> cloud;
    cloud.reserve(point_count);

    std::mt19937 rng(42U);
    std::uniform_real_distribution<float> tiny_noise(-0.035F, 0.035F);

    const std::size_t repeats_per_voxel = 10U;
    const std::size_t unique_voxels = (point_count + repeats_per_voxel - 1U) / repeats_per_voxel;
    const std::size_t row_width = static_cast<std::size_t>(std::sqrt(unique_voxels)) + 1U;

    for (std::size_t voxel = 0; cloud.size() < point_count; ++voxel) {
        const float base_x = static_cast<float>(voxel % row_width) * 0.55F;
        const float base_y = static_cast<float>(voxel / row_width) * 0.55F;
        const float base_z = 0.2F * std::sin(base_x * 0.07F) + 0.1F * std::cos(base_y * 0.05F);

        for (std::size_t repeat = 0; repeat < repeats_per_voxel && cloud.size() < point_count;
             ++repeat) {
            cloud.push_back(pcp::PointXYZ{base_x + tiny_noise(rng),
                                          base_y + tiny_noise(rng),
                                          base_z + tiny_noise(rng)});
        }
    }

    return cloud;
}

pcp::PipelineConfig benchmarkConfig(bool enable_downsampling) {
    pcp::PipelineConfig config;
    config.enable_downsampling = enable_downsampling;
    config.filter.enable_statistical_outlier_removal = false;
    config.filter.z_min = -3.0F;
    config.filter.z_max = 3.0F;
    config.voxel.voxel_size = 0.25F;
    config.segmentation.cluster_tolerance = 0.28F;
    config.segmentation.min_cluster_size = 1U;
    config.segmentation.max_cluster_size = 1000000U;
    return config;
}

Stats summarize(std::vector<double> samples) {
    std::sort(samples.begin(), samples.end());
    const double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    const auto p95_index = static_cast<std::size_t>(
        std::ceil(static_cast<double>(samples.size()) * 0.95) - 1.0);

    Stats stats;
    stats.mean_ms = sum / static_cast<double>(samples.size());
    stats.median_ms = samples[samples.size() / 2U];
    stats.p95_ms = samples[std::min(p95_index, samples.size() - 1U)];
    return stats;
}

Stats runTimed(const pcp::PointCloudPipeline& pipeline,
               const std::vector<pcp::PointXYZ>& cloud,
               int iterations) {
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(iterations));

    for (int i = 0; i < iterations; ++i) {
        const auto start = std::chrono::steady_clock::now();
        const auto result = pipeline.process(cloud);
        const auto end = std::chrono::steady_clock::now();
        if (result.filtered_cloud.empty()) {
            throw std::runtime_error("benchmark generated an empty filtered cloud");
        }
        samples.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    return summarize(std::move(samples));
}

std::string renderMarkdownTable(const std::vector<BenchmarkRow>& rows) {
    std::ostringstream out;
    out << "| Points | Baseline mean ms | Downsampled mean ms | Baseline P95 ms | "
           "Downsampled P95 ms | Speedup |\n";
    out << "|---:|---:|---:|---:|---:|---:|\n";
    out << std::fixed << std::setprecision(2);
    for (const BenchmarkRow& row : rows) {
        out << "| " << row.point_count << " | " << row.baseline.mean_ms << " | "
            << row.downsampled.mean_ms << " | " << row.baseline.p95_ms << " | "
            << row.downsampled.p95_ms << " | " << row.speedup << "x |\n";
    }
    return out.str();
}

void updateReadmeTable(const std::filesystem::path& readme_path, const std::string& table) {
    std::ifstream input(readme_path);
    if (!input) {
        throw std::runtime_error("could not open README.md for benchmark table update");
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    std::string text = buffer.str();

    const std::string begin = "<!-- BENCHMARK_TABLE_BEGIN -->";
    const std::string end = "<!-- BENCHMARK_TABLE_END -->";
    const std::size_t begin_pos = text.find(begin);
    const std::size_t end_pos = text.find(end);
    if (begin_pos == std::string::npos || end_pos == std::string::npos || begin_pos > end_pos) {
        throw std::runtime_error("README benchmark markers were not found");
    }

    const std::string replacement = begin + "\n" + table + end;
    text.replace(begin_pos, end_pos + end.size() - begin_pos, replacement);

    std::ofstream output(readme_path);
    output << text;
}

}  // namespace

int main(int argc, char** argv) {
    const bool update_readme =
        argc > 1 && std::string(argv[1]) == std::string("--update-readme");
    const int iterations = 5;
    const std::vector<std::size_t> sizes{100000U, 250000U, 500000U, 1000000U};

    std::vector<BenchmarkRow> rows;
    rows.reserve(sizes.size());

    for (const std::size_t size : sizes) {
        std::cout << "Generating " << size << " synthetic LiDAR points\n";
        const std::vector<pcp::PointXYZ> cloud = makeSyntheticLidarCloud(size);

        const pcp::PointCloudPipeline baseline_pipeline(benchmarkConfig(false));
        const pcp::PointCloudPipeline downsampled_pipeline(benchmarkConfig(true));

        BenchmarkRow row;
        row.point_count = size;
        row.baseline = runTimed(baseline_pipeline, cloud, iterations);
        row.downsampled = runTimed(downsampled_pipeline, cloud, iterations);
        row.speedup = row.baseline.mean_ms / row.downsampled.mean_ms;
        rows.push_back(row);
    }

    const std::string table = renderMarkdownTable(rows);
    std::cout << '\n' << table;

    const std::filesystem::path results_path = "benchmarks/latest_results.md";
    std::ofstream results(results_path);
    results << table;
    std::cout << "\nWrote " << results_path.string() << '\n';

    if (update_readme) {
        updateReadmeTable("README.md", table);
        std::cout << "Updated README.md benchmark table\n";
    }

    return 0;
}
