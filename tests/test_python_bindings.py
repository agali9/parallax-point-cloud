def test_point_class_imports():
    import pointcloud_pipeline_py as pcp
    point = pcp.PointXYZ()
    point.x = 1.0
    assert point.x == 1.0