# Performance Notes

The pipeline is designed around the idea that segmentation is usually the most
expensive stage for dense LiDAR frames. Voxel-grid downsampling reduces the
number of points that reach clustering while preserving the larger geometry of
cars, walls, poles, and other objects.

## Benchmark Methodology

The benchmark executable generates deterministic synthetic LiDAR-like point
clouds at these sizes:

- 100,000 points
- 250,000 points
- 500,000 points
- 1,000,000 points

It compares two real executions:

- Pipeline A: filtering plus segmentation.
- Pipeline B: filtering, voxel-grid downsampling, and segmentation.

Each size is run several times. The executable reports mean, median, and P95
frame time, then computes speedup from the measured means. It also writes
`benchmarks/latest_results.md` and can update the README table:

```bash
./build/pointcloud_pipeline_benchmark --update-readme
```

No benchmark values are stored by hand. The table is produced by the executable
from the current machine, compiler, and build type.

## Why Voxel Downsampling Helps

The Euclidean clustering stage builds a spatial hash grid and searches nearby
cells for every point. Even with a grid, repeated points inside the same local
area cause many distance checks. Voxel downsampling merges those repeated points
into one centroid per voxel:

```text
many raw hits in one voxel -> one representative centroid
```

That gives two benefits:

- The clustering loop visits fewer points.
- Dense local neighborhoods have fewer pairwise distance checks.

The benchmark generator intentionally creates local groups of nearby points, as
real LiDAR often does on visible surfaces. That makes the downsampled pipeline
much cheaper without pretending the data is random noise.

## Memory Considerations

The code favors contiguous storage:

- Point clouds are `std::vector<PointXYZ>`.
- Processing functions accept `std::span<const PointXYZ>` so callers can pass
  vectors or external buffers without copying.
- Voxel downsampling uses one hash table of accumulators and emits centroids
  into a compact vector.
- Python input uses the NumPy buffer directly.

The pipeline returns owned output vectors because downstream code often needs to
publish, save, or inspect those clouds after `process()` returns. That ownership
choice is slightly more memory-heavy than a purely in-place design, but it keeps
the API simple and safe.

## Scaling Discussion

Filtering is linear in the number of input points. Voxel downsampling is expected
linear time because each point does one hash lookup and accumulator update.
Clustering is also near-linear for moderate point density because each point only
checks nearby grid cells. The difficult case is a very dense cluster where many
points occupy the same search cells; downsampling directly targets that case.

For real robots, the best voxel size depends on the sensor and object scale. A
small ground robot might use 0.05 to 0.15 meters, while an outdoor vehicle might
use 0.20 to 0.35 meters. The right value is usually the largest voxel that still
keeps obstacles shaped well enough for the planner.
