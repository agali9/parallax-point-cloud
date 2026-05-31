# Parallax (point-cloud)

Parallax is a C++20 LiDAR point-cloud processing pipeline with an optional CUDA path. On five KITTI Velodyne scans averaging 122K points, the CPU pipeline runs end to end in 10.3 ms per frame.

The core pipeline does three things:

1. Filters invalid points, applies pass-through bounds, and can remove statistical outliers.
2. Downsamples with a hash-based voxel grid using voxel centroids.
3. Segments the downsampled cloud with Euclidean clustering.

The repo also includes Python bindings and a ROS 2 wrapper package. They have not been built or tested in the environment used for the results below (see [Python Build and Usage](#python-build-and-usage) and [ROS 2 Usage](#ros-2-usage)).

## Results

### Real data: KITTI

Five KITTI Velodyne scans (frames 000021-000025), 5 timed iterations per frame, MSVC Release:

| Stage | Average points |
|---|---:|
| Input | 122,280 |
| After filtering | 56,356 |
| After voxel downsampling | 8,086 |
| Clusters | 22.6 |

| Path | End-to-end mean per frame |
|---|---:|
| CPU | 10.3 ms |
| CUDA/CPU hybrid | 14.8 ms |

At this cloud size the CPU path is faster: only 4.7 ms of the hybrid's 14.8 ms is GPU compute.

Settings: ROI x 0 to 50 m, y -15 to 15 m, z -2.5 to 1.0 m; 0.25 m voxels; 0.65 m cluster tolerance; minimum cluster size 8; statistical outlier removal off. GPU: NVIDIA GeForce RTX 5070 Laptop, CUDA 13.3.

### Synthetic benchmarks

See [Benchmarks](#benchmarks): voxel downsampling before clustering cuts runtime 2.0-5.0x on 100K-1M synthetic points, and the CUDA hybrid path reaches up to 1.37x end to end at 500K points.

## Repository Layout

```text
include/                  Public C++ headers
src/                      Shared library implementation
src/cuda/                 Optional CUDA preprocess (filter + voxel)
python/                   pybind11 bindings
ros2/pointcloud_pipeline_ros/
                           ROS 2 wrapper package
tests/                    C++ and Python tests
benchmarks/               Synthetic LiDAR benchmark
examples/                 Small C++ and Python examples
docs/                     Architecture, performance, and CUDA notes
```

## Architecture

The important design choice is that all real processing lives in the shared C++
library. Python and ROS 2 only convert data at the boundary.

```mermaid
flowchart LR
    A["Input points"] --> B["PointCloudPipeline"]
    B --> C["Filtering"]
    C --> D["Voxel grid"]
    D --> E["Euclidean clustering"]
    E --> F["C++ result"]
    E --> G["Python NumPy"]
    E --> H["ROS 2 topics"]
```

The main API is:

```cpp
pointcloud_pipeline::PointCloudPipeline pipeline(config);
pointcloud_pipeline::PipelineResult result = pipeline.process(points);
```

## Native Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build --output-on-failure
```

The build uses `-Wall -Wextra -Wpedantic` on GCC/Clang and `/W4` on MSVC. Open3D,
pybind11, and CUDA are detected when installed. The core C++ library and tests
still build without them.

## Python Build and Usage

> Status: the bindings are in source but have not been built or tested in the environment used for the results above.

Install pybind11 and NumPy in the Python environment CMake will find:

```bash
python -m pip install pybind11 numpy pytest
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target pointcloud_pipeline_py --config Release
```

Example:

```python
import numpy as np
import pointcloud_pipeline_py as pcp

cloud = np.random.rand(10000, 3).astype(np.float32)
result = pcp.run_pipeline(
    cloud,
    enable_statistical_outlier_removal=False,
    voxel_size=0.25,
    cluster_tolerance=0.65,
    min_cluster_size=8,
)

print(result["downsampled_cloud"].shape)
print(len(result["clusters"]))
```

The Python test checks zero-copy input with:

```python
assert pcp.numpy_data_address(cloud) == cloud.ctypes.data
```

The binding requires a C-contiguous `numpy.ndarray` with shape `(N, 3)` and
`dtype=float32`. C++ reads that buffer directly as `PointXYZ` data through
`std::span<const PointXYZ>`. Output clouds are NumPy arrays backed by C++ vectors
owned by pybind11 capsules.

## ROS 2 Usage

> Status: the ROS 2 package is in source but has not been built or run in the environment used for the results above.

Build and install the core library first:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PWD/install
cmake --build build --config Release
cmake --install build
```

Then build the ROS 2 package:

```bash
source /opt/ros/humble/setup.bash
cd ros2
colcon build --cmake-args -DCMAKE_PREFIX_PATH=$OLDPWD/install
source install/setup.bash
ros2 launch pointcloud_pipeline_ros pipeline.launch.py
```

The node subscribes to `/points_raw`, publishes `processed_cloud`, and publishes
`cluster_markers` for RViz. Processing is done by the shared library, not by a
second ROS-only implementation.

## CUDA Backend

The optional CUDA path uses Thrust (`copy_if`, `sort_by_key`, `reduce_by_key`) to accelerate pass-through filtering and voxel downsampling on NVIDIA GPUs, then runs Euclidean clustering on the CPU. Build with `-DPOINTCLOUD_PIPELINE_USE_CUDA=ON` when the CUDA toolkit is installed.

```cpp
pointcloud_pipeline::PipelineConfig config;
config.backend = pointcloud_pipeline::ExecutionBackend::CUDA;
config.filter.enable_statistical_outlier_removal = false;
```

Python:

```python
result = pcp.run_pipeline(cloud, use_cuda=True, enable_statistical_outlier_removal=False)
```

ROS 2:

```bash
ros2 launch pointcloud_pipeline_ros pipeline.launch.py use_cuda:=true
```

See `docs/cuda.md` for architecture notes, parity tests, and benchmark commands.

## Benchmarks

Build and run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target pointcloud_pipeline_benchmark --config Release
./build/pointcloud_pipeline_benchmark --update-readme
```

The benchmark generates deterministic synthetic LiDAR point clouds at 100k,
250k, 500k, and 1M points. It compares:

- Baseline: filtering plus segmentation.
- Downsampled: filtering, voxel grid, then segmentation.

The program measures real wall-clock frame times and computes mean, median, P95,
and speedup.

The two tables below come from separate benchmark runs, so their CPU timings are not directly comparable with each other.

CPU voxel downsampling vs baseline (Windows, MSVC Release):

<!-- BENCHMARK_TABLE_BEGIN -->
| Points | Baseline mean ms | Downsampled mean ms | Baseline P95 ms | Downsampled P95 ms | Speedup |
|---:|---:|---:|---:|---:|---:|
| 100000 | 184.00 | 37.11 | 562.30 | 39.28 | 4.96x |
| 250000 | 226.33 | 72.02 | 246.19 | 85.39 | 3.14x |
| 500000 | 538.94 | 234.55 | 559.70 | 255.75 | 2.30x |
| 1000000 | 1020.39 | 509.63 | 1192.57 | 572.27 | 2.00x |
<!-- BENCHMARK_TABLE_END -->

CUDA hybrid path (same machine: CUDA 13.3, NVIDIA GeForce RTX 5070 Laptop GPU). GPU filter + voxel, then CPU clustering. GPU compute is filter+voxel only; H2D+D2H is PCIe transfer.

```bash
./build/Release/pointcloud_pipeline_benchmark --cuda --update-readme
```

<!-- CUDA_BENCHMARK_TABLE_BEGIN -->
| Points | CPU mean ms | GPU total mean ms | GPU compute mean ms | H2D+D2H mean ms | Speedup |
|---:|---:|---:|---:|---:|---:|
| 100000 | 15.00 | 22.83 | 4.75 | 2.64 | 0.66x |
| 250000 | 61.05 | 50.94 | 6.79 | 5.45 | 1.20x |
| 500000 | 207.85 | 151.91 | 10.22 | 12.81 | 1.37x |
| 1000000 | 579.07 | 444.44 | 32.04 | 23.83 | 1.30x |
<!-- CUDA_BENCHMARK_TABLE_END -->

The hybrid path is slower than the CPU path at 100K points (0.66x) and faster from 250K points up (1.20-1.37x).

## Performance Discussion

Voxel-grid downsampling is the main optimization. The implementation hashes
`floor(x / voxel_size)`, `floor(y / voxel_size)`, and `floor(z / voxel_size)`,
stores count and coordinate sums, then emits one centroid per occupied voxel.
This keeps the larger geometry while reducing the number of points that
clustering has to search.

The benchmark data is generated so nearby LiDAR returns form small dense groups,
which is where voxel downsampling helps most. On a typical Release build, the
downsampled pipeline should be much faster than the baseline, and the exact
speedup should be measured on the target machine instead of copied from someone
else's laptop.

## Memory Model

Point clouds are stored as `std::vector<PointXYZ>`, which is compact and
contiguous. Algorithms accept `std::span<const PointXYZ>` so callers can pass
existing buffers without an input copy. The pipeline returns owned vectors in
`PipelineResult` because ROS publishers, Python bindings, and examples need the
processed clouds to remain valid after the call returns.

The Python bindings are designed for zero-copy input, and output avoids an extra
copy by moving C++ vectors into capsule-owned NumPy arrays.

## Testing and Coverage

14 C++ unit tests pass through ctest, including 3 CUDA CPU/GPU parity tests run on the RTX 5070. They cover filtering, voxel indexing and centroid calculation, segmentation, bounding boxes, and end-to-end pipeline behavior:

```bash
ctest --test-dir build --output-on-failure
```

Python tests cover NumPy interoperability and pointer equality (not yet run; see the status note above):

```bash
PYTHONPATH=build pytest tests/test_python_bindings.py
```

For coverage with GCC or Clang:

```bash
cmake -S . -B build-coverage -DCMAKE_BUILD_TYPE=Debug -DPOINTCLOUD_PIPELINE_ENABLE_COVERAGE=ON
cmake --build build-coverage
ctest --test-dir build-coverage --output-on-failure
```
