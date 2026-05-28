# CUDA Backend

The optional CUDA backend accelerates the structured stages of the pipeline on NVIDIA GPUs:

- pass-through filtering (finite-point check plus axis-aligned bounds)
- voxel-grid downsampling via Thrust `sort_by_key` and `reduce_by_key`

Statistical outlier removal and Euclidean clustering remain on the CPU.

## Architecture

```mermaid
flowchart LR
    A["Host PointXYZ cloud"] --> B["H2D upload"]
    B --> C["GPU pass-through filter"]
    C --> D["GPU voxel sort + reduce"]
    D --> E["D2H downsampled cloud"]
    E --> F["CPU Euclidean clustering"]
    F --> G["PipelineResult"]
```

When statistical outlier removal is enabled, the CPU runs the full filter stage first, then the GPU performs voxel downsampling on the filtered cloud.

## Build

CUDA is optional. The core library still builds when the CUDA toolkit is absent.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPOINTCLOUD_PIPELINE_USE_CUDA=ON
cmake --build build --config Release
```

Requirements:

- CUDA Toolkit 12.x or newer
- NVIDIA driver with a compatible GPU
- CMake 3.20+

On Windows, use the MSVC + CUDA toolchain installed with the CUDA SDK.

## API

```cpp
pointcloud_pipeline::PipelineConfig config;
config.backend = pointcloud_pipeline::ExecutionBackend::CUDA;
config.filter.enable_statistical_outlier_removal = false;

pointcloud_pipeline::PointCloudPipeline pipeline(config);
pointcloud_pipeline::PipelineResult result = pipeline.process(points);
```

`ExecutionBackend::Auto` uses CUDA when the library was built with GPU support and a device is present.

Runtime helpers:

```cpp
pointcloud_pipeline::isCudaAvailable();
pointcloud_pipeline::cudaDeviceName();
```

## Python

```python
import pointcloud_pipeline_py as pcp

print(pcp.is_cuda_available())
print(pcp.cuda_device_name())

result = pcp.run_pipeline(cloud, use_cuda=True, enable_statistical_outlier_removal=False)
print(result["timings"]["used_cuda"])
print(result["timings"]["h2d_ms"], result["timings"]["d2h_ms"])
```

## ROS 2

Set `use_cuda:=true` on `pointcloud_pipeline_node`. The default is `false` so robots without NVIDIA GPUs keep the CPU path.

## Benchmarks

```bash
cmake --build build --target pointcloud_pipeline_benchmark --config Release
./build/Release/pointcloud_pipeline_benchmark --cuda
./build/Release/pointcloud_pipeline_benchmark --cuda --update-readme
```

The CUDA benchmark compares the full CPU pipeline against the hybrid GPU preprocess + CPU clustering path. It reports:

- CPU total mean time
- GPU total mean time
- GPU compute mean time (filter + voxel stages)
- H2D + D2H transfer mean time
- speedup

Results are written to `benchmarks/latest_cuda_results.md`.

## Parity

`tests/test_cuda_parity.cpp` compares CPU and GPU outputs for filtering, voxel downsampling, and end-to-end cluster counts. These tests skip automatically when no GPU is available.

## Limitations

- No GPU statistical outlier removal
- No GPU Euclidean clustering
- Host/device transfers are included in real pipeline timings
- macOS is unsupported because CUDA is unavailable there
