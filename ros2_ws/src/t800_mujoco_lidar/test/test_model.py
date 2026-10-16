import numpy as np
import pytest

from t800_mujoco_lidar.model import (
    build_ray_directions, stamp_parts, world_points_to_lidar,
)


def test_default_layout():
    rays, rings = build_ray_directions(240, 180, 120.0, 33.1, 56.9)
    assert rays.shape == (43200, 3)
    assert rings.shape == (43200,)
    assert rings.min() == 0
    assert rings.max() == 179
    np.testing.assert_allclose(np.linalg.norm(rays, axis=1), 1.0, atol=1e-12)


def test_stride_layout():
    rays, rings = build_ray_directions(240, 180, 120.0, 33.1, 56.9, stride=2)
    assert rays.shape == (10800, 3)
    assert set(np.unique(rings)) == set(range(0, 180, 2))


def test_invalid_stride():
    with pytest.raises(ValueError):
        build_ray_directions(240, 180, 120.0, 33.1, 56.9, stride=7)


def test_lidar_coordinate_conversion():
    camera_from_lidar = np.array([[0.0, -1.0, 0.0],
                                  [0.0, 0.0, 1.0],
                                  [-1.0, 0.0, 0.0]])
    lidar_point = np.array([[2.0, 1.0, 0.5]])
    world_point = lidar_point @ camera_from_lidar.T
    np.testing.assert_allclose(
        world_points_to_lidar(world_point, np.zeros(3), np.eye(3)), lidar_point)


def test_stamp_parts():
    assert stamp_parts(123.25) == (123, 250_000_000)
    with pytest.raises(ValueError):
        stamp_parts(0.0)
