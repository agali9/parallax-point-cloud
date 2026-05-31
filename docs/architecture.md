# Architecture

This project is organized around one reusable C++ shared library. The ROS 2 node
and Python module both call the same library code, so there is only one version
of the filtering, downsampling, and clustering logic to maintain.

## Data Flow

```mermaid
flowchart LR
    A["Raw LiDAR points<br/>PointXYZ or PointCloud2"] --> B["Filtering"]
    B --> C["Voxel-grid downsampling"]
    C --> D["Euclidean clustering"]
    D --> E["PipelineResult"]
    E --> F["C++ application"]
    E --> G["Python NumPy result"]
    E --> H["ROS 2 processed_cloud and cluster_markers"]
```

## Processing Pipeline

```mermaid
flowchart TD
    A["Input cloud"] --> B["Remove NaN and infinity"]
    B --> C["Pass-through bounds"]
    C --> D["Statistical outlier removal"]
    D --> E["Hash voxel key: floor(x/s), floor(y/s), floor(z/s)"]
    E --> F["Accumulate count and xyz sums"]
    F --> G["Emit voxel centroids"]
    G --> H["Build spatial search grid"]
    H --> I["Breadth-first Euclidean clusters"]
    I --> J["Centroid and bounding box per cluster"]
```

## ROS 2 Integration

```mermaid
flowchart LR
    A["sensor_msgs/PointCloud2<br/>/points_raw"] --> B["pointcloud_pipeline_node"]
    B --> C["PointCloudPipeline shared library"]
    C --> D["sensor_msgs/PointCloud2<br/>processed_cloud"]
    C --> E["visualization_msgs/MarkerArray<br/>cluster_markers"]
    D --> F["RViz point cloud display"]
    E --> G["RViz marker display"]
```

The ROS package does message conversion and visualization only. Processing stays
inside the shared library, which makes the same code usable from offline C++,
Python, and ROS 2.

## Python Binding Architecture

```mermaid
flowchart LR
    A["numpy.ndarray float32 (N, 3)"] --> B["pybind11 buffer_info"]
    B --> C["std::span<const PointXYZ>"]
    C --> D["C++ pipeline"]
    D --> E["std::vector<PointXYZ> outputs"]
    E --> F["NumPy arrays with capsule-owned C++ memory"]
```

Input is zero-copy when the array is C-contiguous `float32` with shape `(N, 3)`.
The binding reads the raw buffer pointer and creates a `std::span<const PointXYZ>`
over that memory. Output clouds are backed by C++ vectors owned by a pybind11
capsule, so Python can read them as NumPy arrays without a second output copy.

## Main Files

- `include/pointcloud_pipeline/types.hpp`: simple data structures.
- `src/filtering.cpp`: invalid point removal, pass-through filtering, and
  statistical outlier removal.
- `src/voxel_grid.cpp`: hash-based voxel indexing and centroid downsampling.
- `src/segmentation.cpp`: grid-accelerated Euclidean clustering.
- `python/bindings.cpp`: pybind11 module.
- `ros2/pointcloud_pipeline_ros/src/pipeline_node.cpp`: ROS 2 wrapper node.
