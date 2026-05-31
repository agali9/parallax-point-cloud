import numpy as np

import pointcloud_pipeline_py as pcp


cloud = np.array(
    [
        [0.0, 0.0, 0.0],
        [0.05, 0.0, 0.0],
        [0.10, 0.02, 0.0],
        [4.0, 4.0, 0.0],
        [4.05, 4.0, 0.0],
        [4.10, 4.02, 0.0],
    ],
    dtype=np.float32,
)

result = pcp.run_pipeline(
    cloud,
    enable_statistical_outlier_removal=False,
    voxel_size=0.05,
    cluster_tolerance=0.25,
    min_cluster_size=2,
)

print("NumPy data pointer:", cloud.ctypes.data)
print("C++ saw pointer:", result["input_address"])
print("Downsampled shape:", result["downsampled_cloud"].shape)
print("Clusters:", len(result["clusters"]))
