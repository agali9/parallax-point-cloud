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

#include "pointcloud_pipeline/cuda_backend.hpp"
#include "pointcloud_pipeline/io.hpp"
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

struct CudaBenchmarkRow {
    std::size_t point_count = 0;
    Stats cpu;
    Stats gpu_total;
    Stats gpu_transfer;
    Stats gpu_compute;
    double speedup = 0.0;
};

struct KittiFrameMetrics {
    std::string frame_name;
    std::size_t input_count = 0;
    std::size_t filtered_count = 0;
    std::size_t voxelized_count = 0;
    std::size_t cluster_count = 0;
    Stats cpu;
    Stats gpu_total;
    Stats gpu_transfer;
    Stats gpu_compute;
    double speedup = 0.0;
    bool measured_cuda = false;
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

// Outdoor vehicle ROI tuned for KITTI Velodyne scans (sensor frame).
pcp::PipelineConfig kittiConfig(pcp::ExecutionBackend backend) {
    pcp::PipelineConfig config;
    config.enable_downsampling = true;
    config.backend = backend;
    config.filter.enable_statistical_outlier_removal = false;
    config.filter.x_min = 0.0F;
    config.filter.x_max = 50.0F;
    config.filter.y_min = -15.0F;
    config.filter.y_max = 15.0F;
    config.filter.z_min = -2.5F;
    config.filter.z_max = 1.0F;
    config.voxel.voxel_size = 0.25F;
    config.segmentation.cluster_tolerance = 0.65F;
    config.segmentation.min_cluster_size = 8U;
    config.segmentation.max_cluster_size = 250000U;
    return config;
}

pcp::PipelineConfig cudaBenchmarkConfig() {
    pcp::PipelineConfig config = benchmarkConfig(true);
    config.backend = pcp::ExecutionBackend::CUDA;
    return config;
}

pcp::PipelineConfig cpuBenchmarkConfig() {
    pcp::PipelineConfig config = benchmarkConfig(true);
    config.backend = pcp::ExecutionBackend::CPU;
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

CudaBenchmarkRow runCudaComparison(const std::vector<pcp::PointXYZ>& cloud, int iterations) {
    const pcp::PointCloudPipeline cpu_pipeline(cpuBenchmarkConfig());
    const pcp::PointCloudPipeline cuda_pipeline(cudaBenchmarkConfig());

    // Warm up CUDA runtime and kernels before timing.
    (void)cuda_pipeline.process(cloud);

    std::vector<double> cpu_samples;
    std::vector<double> gpu_total_samples;
    std::vector<double> transfer_samples;
    std::vector<double> compute_samples;
    cpu_samples.reserve(static_cast<std::size_t>(iterations));
    gpu_total_samples.reserve(static_cast<std::size_t>(iterations));
    transfer_samples.reserve(static_cast<std::size_t>(iterations));
    compute_samples.reserve(static_cast<std::size_t>(iterations));

    for (int i = 0; i < iterations; ++i) {
        const auto cpu_start = std::chrono::steady_clock::now();
        (void)cpu_pipeline.process(cloud);
        const auto cpu_end = std::chrono::steady_clock::now();
        cpu_samples.push_back(
            std::chrono::duration<double, std::milli>(cpu_end - cpu_start).count());

        const auto gpu_start = std::chrono::steady_clock::now();
        const auto gpu_result = cuda_pipeline.process(cloud);
        const auto gpu_end = std::chrono::steady_clock::now();
        if (!gpu_result.timings.used_cuda) {
            throw std::runtime_error("CUDA benchmark expected the GPU backend");
        }

        gpu_total_samples.push_back(
            std::chrono::duration<double, std::milli>(gpu_end - gpu_start).count());
        transfer_samples.push_back(gpu_result.timings.h2d_ms + gpu_result.timings.d2h_ms);
        compute_samples.push_back(gpu_result.timings.filter_ms + gpu_result.timings.downsample_ms);
    }

    CudaBenchmarkRow row;
    row.cpu = summarize(std::move(cpu_samples));
    row.gpu_total = summarize(std::move(gpu_total_samples));
    row.gpu_transfer = summarize(std::move(transfer_samples));
    row.gpu_compute = summarize(std::move(compute_samples));
    row.speedup = row.cpu.mean_ms / row.gpu_total.mean_ms;
    return row;
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

std::string renderCudaMarkdownTable(const std::vector<CudaBenchmarkRow>& rows) {
    std::ostringstream out;
    out << "| Points | CPU mean ms | GPU total mean ms | GPU compute mean ms | "
           "H2D+D2H mean ms | Speedup |\n";
    out << "|---:|---:|---:|---:|---:|---:|\n";
    out << std::fixed << std::setprecision(2);
    for (const CudaBenchmarkRow& row : rows) {
        out << "| " << row.point_count << " | " << row.cpu.mean_ms << " | "
            << row.gpu_total.mean_ms << " | " << row.gpu_compute.mean_ms << " | "
            << row.gpu_transfer.mean_ms << " | " << row.speedup << "x |\n";
    }
    return out.str();
}

std::string renderKittiMarkdownTable(const std::vector<KittiFrameMetrics>& rows, bool with_cuda) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    if (with_cuda) {
        out << "| Frame | Input | Filtered | Voxelized | Clusters | CPU mean ms | "
               "GPU total mean ms | GPU compute mean ms | H2D+D2H mean ms | Speedup |\n";
        out << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
        for (const KittiFrameMetrics& row : rows) {
            out << "| " << row.frame_name << " | " << row.input_count << " | "
                << row.filtered_count << " | " << row.voxelized_count << " | "
                << row.cluster_count << " | " << row.cpu.mean_ms << " | "
                << row.gpu_total.mean_ms << " | " << row.gpu_compute.mean_ms << " | "
                << row.gpu_transfer.mean_ms << " | " << row.speedup << "x |\n";
        }
    } else {
        out << "| Frame | Input | Filtered | Voxelized | Clusters | CPU mean ms | "
               "CPU P95 ms |\n";
        out << "|---|---:|---:|---:|---:|---:|---:|\n";
        for (const KittiFrameMetrics& row : rows) {
            out << "| " << row.frame_name << " | " << row.input_count << " | "
                << row.filtered_count << " | " << row.voxelized_count << " | "
                << row.cluster_count << " | " << row.cpu.mean_ms << " | "
                << row.cpu.p95_ms << " |\n";
        }
    }
    return out.str();
}

KittiFrameMetrics averageKittiRows(const std::vector<KittiFrameMetrics>& rows, bool with_cuda) {
    KittiFrameMetrics avg;
    avg.frame_name = "mean";
    if (rows.empty()) {
        return avg;
    }

    double input = 0.0;
    double filtered = 0.0;
    double voxelized = 0.0;
    double clusters = 0.0;
    double cpu = 0.0;
    double cpu_p95 = 0.0;
    double gpu_total = 0.0;
    double gpu_compute = 0.0;
    double gpu_transfer = 0.0;
    double speedup = 0.0;

    for (const KittiFrameMetrics& row : rows) {
        input += static_cast<double>(row.input_count);
        filtered += static_cast<double>(row.filtered_count);
        voxelized += static_cast<double>(row.voxelized_count);
        clusters += static_cast<double>(row.cluster_count);
        cpu += row.cpu.mean_ms;
        cpu_p95 += row.cpu.p95_ms;
        if (with_cuda) {
            gpu_total += row.gpu_total.mean_ms;
            gpu_compute += row.gpu_compute.mean_ms;
            gpu_transfer += row.gpu_transfer.mean_ms;
            speedup += row.speedup;
        }
    }

    const double n = static_cast<double>(rows.size());
    avg.input_count = static_cast<std::size_t>(std::llround(input / n));
    avg.filtered_count = static_cast<std::size_t>(std::llround(filtered / n));
    avg.voxelized_count = static_cast<std::size_t>(std::llround(voxelized / n));
    avg.cluster_count = static_cast<std::size_t>(std::llround(clusters / n));
    avg.cpu.mean_ms = cpu / n;
    avg.cpu.p95_ms = cpu_p95 / n;
    avg.measured_cuda = with_cuda;
    if (with_cuda) {
        avg.gpu_total.mean_ms = gpu_total / n;
        avg.gpu_compute.mean_ms = gpu_compute / n;
        avg.gpu_transfer.mean_ms = gpu_transfer / n;
        avg.speedup = speedup / n;
    }
    return avg;
}

void updateMarkedTable(const std::filesystem::path& readme_path,
                       const std::string& table,
                       const std::string& begin_marker,
                       const std::string& end_marker) {
    std::ifstream input(readme_path);
    if (!input) {
        throw std::runtime_error("could not open README.md for benchmark table update");
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    std::string text = buffer.str();

    const std::size_t begin_pos = text.find(begin_marker);
    const std::size_t end_pos = text.find(end_marker);
    if (begin_pos == std::string::npos || end_pos == std::string::npos || begin_pos > end_pos) {
        throw std::runtime_error("README benchmark markers were not found: " + begin_marker);
    }

    const std::string replacement = begin_marker + "\n" + table + end_marker;
    text.replace(begin_pos, end_pos + end_marker.size() - begin_pos, replacement);

    std::ofstream output(readme_path);
    output << text;
}

void updateReadmeTable(const std::filesystem::path& readme_path, const std::string& table) {
    updateMarkedTable(readme_path, table, "<!-- BENCHMARK_TABLE_BEGIN -->",
                      "<!-- BENCHMARK_TABLE_END -->");
}

void updateCudaReadmeTable(const std::filesystem::path& readme_path, const std::string& table) {
    updateMarkedTable(readme_path, table, "<!-- CUDA_BENCHMARK_TABLE_BEGIN -->",
                      "<!-- CUDA_BENCHMARK_TABLE_END -->");
}

void updateKittiReadmeTable(const std::filesystem::path& readme_path, const std::string& table) {
    updateMarkedTable(readme_path, table, "<!-- KITTI_BENCHMARK_TABLE_BEGIN -->",
                      "<!-- KITTI_BENCHMARK_TABLE_END -->");
}

std::vector<std::filesystem::path> listKittiBins(const std::filesystem::path& dir) {
    if (!std::filesystem::is_directory(dir)) {
        throw std::runtime_error("KITTI directory not found: " + dir.string());
    }

    std::vector<std::filesystem::path> bins;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        if (entry.path().extension() == ".bin") {
            bins.push_back(entry.path());
        }
    }
    std::sort(bins.begin(), bins.end());
    if (bins.empty()) {
        throw std::runtime_error("no .bin frames found in " + dir.string() +
                                 " (run: python scripts/fetch_kitti_sample.py)");
    }
    return bins;
}

KittiFrameMetrics evaluateKittiFrame(const std::filesystem::path& path,
                                     int iterations,
                                     bool run_cuda) {
    const std::vector<pcp::PointXYZ> cloud = pcp::loadKittiBin(path);
    if (cloud.empty()) {
        throw std::runtime_error("empty KITTI frame: " + path.string());
    }

    const pcp::PointCloudPipeline cpu_pipeline(kittiConfig(pcp::ExecutionBackend::CPU));
    const pcp::PipelineResult ref = cpu_pipeline.process(cloud);
    if (ref.filtered_cloud.empty()) {
        throw std::runtime_error("KITTI frame produced an empty filtered cloud: " + path.string() +
                                 " (check ROI config)");
    }

    KittiFrameMetrics metrics;
    metrics.frame_name = path.filename().string();
    metrics.input_count = cloud.size();
    metrics.filtered_count = ref.filtered_cloud.size();
    metrics.voxelized_count = ref.downsampled_cloud.size();
    metrics.cluster_count = ref.clusters.size();
    metrics.cpu = runTimed(cpu_pipeline, cloud, iterations);

    if (run_cuda) {
        if (!pcp::isCudaAvailable()) {
            throw std::runtime_error("CUDA requested for KITTI eval but no compatible GPU is available");
        }
        const pcp::PointCloudPipeline cuda_pipeline(kittiConfig(pcp::ExecutionBackend::CUDA));
        (void)cuda_pipeline.process(cloud);

        std::vector<double> gpu_total_samples;
        std::vector<double> transfer_samples;
        std::vector<double> compute_samples;
        gpu_total_samples.reserve(static_cast<std::size_t>(iterations));
        transfer_samples.reserve(static_cast<std::size_t>(iterations));
        compute_samples.reserve(static_cast<std::size_t>(iterations));

        for (int i = 0; i < iterations; ++i) {
            const auto gpu_start = std::chrono::steady_clock::now();
            const auto gpu_result = cuda_pipeline.process(cloud);
            const auto gpu_end = std::chrono::steady_clock::now();
            if (!gpu_result.timings.used_cuda) {
                throw std::runtime_error("KITTI CUDA eval expected the GPU backend");
            }
            gpu_total_samples.push_back(
                std::chrono::duration<double, std::milli>(gpu_end - gpu_start).count());
            transfer_samples.push_back(gpu_result.timings.h2d_ms + gpu_result.timings.d2h_ms);
            compute_samples.push_back(gpu_result.timings.filter_ms +
                                      gpu_result.timings.downsample_ms);
        }

        metrics.gpu_total = summarize(std::move(gpu_total_samples));
        metrics.gpu_transfer = summarize(std::move(transfer_samples));
        metrics.gpu_compute = summarize(std::move(compute_samples));
        metrics.speedup = metrics.cpu.mean_ms / metrics.gpu_total.mean_ms;
        metrics.measured_cuda = true;
    }

    return metrics;
}

int runKittiBenchmark(const std::filesystem::path& kitti_dir,
                      bool run_cuda,
                      bool update_readme) {
    const int iterations = 5;
    const std::vector<std::filesystem::path> bins = listKittiBins(kitti_dir);

    std::cout << "KITTI directory: " << kitti_dir.string() << '\n';
    std::cout << "Frames: " << bins.size() << '\n';
    if (run_cuda) {
        const char* device_name = pcp::cudaDeviceName();
        if (device_name != nullptr) {
            std::cout << "CUDA device: " << device_name << '\n';
        }
    }

    std::vector<KittiFrameMetrics> rows;
    rows.reserve(bins.size());
    for (const auto& path : bins) {
        std::cout << "Evaluating " << path.filename().string() << " ("
                  << std::filesystem::file_size(path) / 16U << " points)\n";
        rows.push_back(evaluateKittiFrame(path, iterations, run_cuda));
    }

    const KittiFrameMetrics mean_row = averageKittiRows(rows, run_cuda);
    std::vector<KittiFrameMetrics> table_rows = rows;
    table_rows.push_back(mean_row);

    const std::string table = renderKittiMarkdownTable(table_rows, run_cuda);
    std::cout << '\n' << table;

    const std::filesystem::path results_path = "benchmarks/latest_kitti_results.md";
    std::ofstream results(results_path);
    results << "# KITTI real-LiDAR validation\n\n";
    results << "Directory: `" << kitti_dir.string() << "`\n\n";
    results << "Config: outdoor ROI x[0,50] y[-15,15] z[-2.5,1.0], voxel 0.25 m, "
               "cluster_tolerance 0.65 m, min_cluster_size 8, SOR off.\n\n";
    results << table;
    std::cout << "\nWrote " << results_path.string() << '\n';

    if (update_readme) {
        updateKittiReadmeTable("README.md", table);
        std::cout << "Updated README.md KITTI benchmark table\n";
    }

    return 0;
}

int runCpuBenchmark(bool update_readme) {
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

int runCudaBenchmark(bool update_readme) {
    if (!pcp::isCudaAvailable()) {
        std::cerr << "CUDA benchmark requested but no compatible GPU is available\n";
        return 1;
    }

    const char* device_name = pcp::cudaDeviceName();
    if (device_name != nullptr) {
        std::cout << "CUDA device: " << device_name << '\n';
    }

    const int iterations = 5;
    const std::vector<std::size_t> sizes{100000U, 250000U, 500000U, 1000000U};

    std::vector<CudaBenchmarkRow> rows;
    rows.reserve(sizes.size());

    for (const std::size_t size : sizes) {
        std::cout << "Generating " << size << " synthetic LiDAR points for CUDA benchmark\n";
        const std::vector<pcp::PointXYZ> cloud = makeSyntheticLidarCloud(size);

        CudaBenchmarkRow row = runCudaComparison(cloud, iterations);
        row.point_count = size;
        rows.push_back(row);
    }

    const std::string table = renderCudaMarkdownTable(rows);
    std::cout << '\n' << table;

    const std::filesystem::path results_path = "benchmarks/latest_cuda_results.md";
    std::ofstream results(results_path);
    results << table;
    std::cout << "\nWrote " << results_path.string() << '\n';

    if (update_readme) {
        updateCudaReadmeTable("README.md", table);
        std::cout << "Updated README.md CUDA benchmark table\n";
    }

    return 0;
}

void printUsage() {
    std::cerr
        << "Usage:\n"
        << "  pointcloud_pipeline_benchmark [--update-readme]\n"
        << "  pointcloud_pipeline_benchmark --cuda [--update-readme]\n"
        << "  pointcloud_pipeline_benchmark --kitti <dir> [--cuda] [--update-readme]\n";
}

}  // namespace

int main(int argc, char** argv) {
    bool update_readme = false;
    bool run_cuda = false;
    bool run_kitti = false;
    std::filesystem::path kitti_dir = "data/kitti";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--update-readme") {
            update_readme = true;
        } else if (arg == "--cuda") {
            run_cuda = true;
        } else if (arg == "--kitti") {
            run_kitti = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                kitti_dir = argv[++i];
            }
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else {
            std::cerr << "unknown argument: " << arg << '\n';
            printUsage();
            return 1;
        }
    }

    if (run_kitti) {
        return runKittiBenchmark(kitti_dir, run_cuda, update_readme);
    }
    if (run_cuda) {
        return runCudaBenchmark(update_readme);
    }
    return runCpuBenchmark(update_readme);
}
