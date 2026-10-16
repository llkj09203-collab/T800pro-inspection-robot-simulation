import numpy as np

from t800_mujoco_lidar.pointcloud import make_pointcloud, POINT_DTYPE


def test_pointcloud_schema_and_sample_time():
    message = make_pointcloud(
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
        np.array([7, 8]), 123.25, "lidar_link")
    assert message.header.stamp.sec == 123
    assert message.header.stamp.nanosec == 250_000_000
    assert message.header.frame_id == "lidar_link"
    assert message.width == 2
    assert message.point_step == 24
    assert [field.name for field in message.fields] == [
        "x", "y", "z", "intensity", "ring", "time"]
    decoded = np.frombuffer(message.data, dtype=POINT_DTYPE)
    np.testing.assert_allclose(decoded["x"], [1.0, 4.0])
    np.testing.assert_array_equal(decoded["ring"], [7, 8])
    np.testing.assert_array_equal(decoded["intensity"], 0.0)
    np.testing.assert_array_equal(decoded["time"], 0.0)
