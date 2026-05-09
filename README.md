# Parallax (point-cloud)

C++20 LiDAR point-cloud pipeline. Python bindings accept a C-contiguous
`(N, 3) float32` NumPy array and avoid copying the input buffer.
