"""Ray layout and coordinate conversions for the simulated 3D lidar."""

import math

import numpy as np


CAMERA_FROM_LIDAR = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
])


def validate_layout(horizontal_rays, vertical_rays, stride):
    if horizontal_rays <= 0 or vertical_rays <= 0:
        raise ValueError("ray dimensions must be positive")
    if stride < 1 or horizontal_rays % stride or vertical_rays % stride:
        raise ValueError("angular_stride must divide both ray dimensions")


def build_ray_directions(horizontal_rays, vertical_rays, horizontal_fov_deg,
                         vertical_up_deg, vertical_down_deg, stride=1):
    """Return camera-frame rays and their source ring indices.

    Public lidar coordinates are +X forward, +Y left, +Z up.  The retained
    MuJoCo camera mount is -Z forward, +X right, +Y up.
    """
    validate_layout(horizontal_rays, vertical_rays, stride)
    width = horizontal_rays // stride
    height = vertical_rays // stride
    horizontal_step = horizontal_fov_deg / horizontal_rays * stride
    vertical_step = (vertical_up_deg + vertical_down_deg) / vertical_rays * stride
    azimuth = np.radians(
        -horizontal_fov_deg / 2.0 + (np.arange(width) + 0.5) * horizontal_step
    )
    elevation = np.radians(
        vertical_up_deg - (np.arange(height) + 0.5) * vertical_step
    )
    az, el = np.meshgrid(azimuth, elevation)
    forward = np.cos(el) * np.cos(az)
    left = np.cos(el) * np.sin(az)
    up = np.sin(el)
    camera_rays = np.stack([-left, up, -forward], axis=-1).reshape(-1, 3)
    rings = np.repeat(np.arange(height, dtype=np.uint16) * stride, width)
    return camera_rays, rings


def world_points_to_lidar(points, origin, camera_matrix):
    """Express MuJoCo world points in ROS lidar coordinates."""
    world_from_lidar = np.asarray(camera_matrix, dtype=float).reshape(3, 3) @ CAMERA_FROM_LIDAR
    return (np.asarray(points, dtype=float) - np.asarray(origin, dtype=float)) @ world_from_lidar


def stamp_parts(stamp):
    """Convert a finite positive Unix sample time to ROS seconds/nanoseconds."""
    stamp = float(stamp)
    if not math.isfinite(stamp) or stamp <= 0.0:
        raise ValueError("sample stamp must be finite and positive")
    seconds = math.floor(stamp)
    nanoseconds = int(round((stamp - seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    return int(seconds), nanoseconds
