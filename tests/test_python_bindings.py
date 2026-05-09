import numpy as np

import pointcloud_pipeline_py as pcp


def test_numpy_input_is_seen_zero_copy_by_cpp():
    cloud = np.array(
        [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [4.0, 4.0, 0.0]],
        dtype=np.float32,
    )

    assert pcp.numpy_data_address(cloud) == cloud.ctypes.data


def test_run_pipeline_reports_same_input_address():
    cloud = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.05, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.05, 1.0, 0.0],
        ],
        dtype=np.float32,
    )

    result = pcp.run_pipeline(
        cloud,
        enable_statistical_outlier_removal=False,
        voxel_size=0.02,
        cluster_tolerance=0.2,
        min_cluster_size=2,
    )

    assert result["input_address"] == cloud.ctypes.data
    assert result["filtered_cloud"].shape == (4, 3)
    assert result["downsampled_cloud"].shape == (4, 3)
    assert len(result["clusters"]) == 2


def test_downsample_cloud_returns_numpy_array():
    cloud = np.array(
        [[0.0, 0.0, 0.0], [0.05, 0.05, 0.0], [2.0, 0.0, 0.0]],
        dtype=np.float32,
    )

    downsampled = pcp.downsample_cloud(cloud, voxel_size=0.25)

    assert isinstance(downsampled, np.ndarray)
    assert downsampled.dtype == np.float32
    assert downsampled.shape == (2, 3)
