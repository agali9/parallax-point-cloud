# Performance Notes

The first benchmark made it obvious that segmentation dominates once the input
cloud gets large. The next pass should reduce copies and avoid scanning every
point against every other point when looking for neighbors.

## Profiling Plan

- Time filtering, downsampling, segmentation, and total frame time separately.
- Compare baseline segmentation against the downsampled pipeline.
- Run deterministic synthetic clouds so changes are easy to compare.

## Memory and Search Changes

The pipeline now accepts `std::span<const PointXYZ>` so callers can pass vectors,
arrays, or Python buffers without copying the input. Voxel keys use integer cells
instead of strings, downsampled voxels are sorted for deterministic output, and
segmentation checks neighboring grid cells instead of doing a full all-pairs scan.