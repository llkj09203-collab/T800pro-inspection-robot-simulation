"""Construction of the package's public PointCloud2 interface."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from .model import stamp_parts


POINT_DTYPE = np.dtype([
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("intensity", "<f4"),
    ("ring", "<u2"),
    ("_padding", "<u2"),
    ("time", "<f4"),
])


def make_pointcloud(points, rings, stamp, frame_id):
    points = np.asarray(points, dtype=np.float32)
    rings = np.asarray(rings, dtype=np.uint16)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) != len(rings):
        raise ValueError("points must be Nx3 and match rings")
    if not np.isfinite(points).all():
        raise ValueError("point cloud contains non-finite coordinates")
    packed = np.zeros(len(points), dtype=POINT_DTYPE)
    packed["x"], packed["y"], packed["z"] = points.T
    packed["ring"] = rings
    # The current CPU ray model is an instantaneous, global-shutter capture.
    # Reflectivity and per-point firing time are therefore explicitly unknown.
    packed["intensity"] = 0.0
    packed["time"] = 0.0

    message = PointCloud2()
    message.header.stamp.sec, message.header.stamp.nanosec = stamp_parts(stamp)
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(packed)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = POINT_DTYPE.itemsize
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = packed.tobytes()
    return message
