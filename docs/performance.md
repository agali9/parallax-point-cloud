# Performance Notes

The first benchmark made it obvious that segmentation dominates once the input
cloud gets large. The next pass should reduce copies and avoid scanning every
point against every other point when looking for neighbors.

## Profiling Plan

- Time filtering, downsampling, segmentation, and total frame time separately.
- Compare baseline segmentation against the downsampled pipeline.
- Run deterministic synthetic clouds so changes are easy to compare.