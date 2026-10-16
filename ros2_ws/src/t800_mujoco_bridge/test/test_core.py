import math

import numpy as np

from t800_mujoco_bridge.core import (
    matrix_to_quaternion,
    planar_velocity,
    relative_transform,
    velocity_to_policy,
)


def rotation_z(angle):
    return np.array([[math.cos(angle), -math.sin(angle), 0.0],
                     [math.sin(angle), math.cos(angle), 0.0],
                     [0.0, 0.0, 1.0]])


def test_relative_transform_reconstructs_child_pose():
    parent_origin = np.array([1.2, -0.4, 1.1])
    parent_rotation = rotation_z(0.7)
    local_origin = np.array([0.1, 0.0, 0.4])
    local_rotation = rotation_z(-0.35)
    child_origin = parent_origin + parent_rotation @ local_origin
    child_rotation = parent_rotation @ local_rotation
    translation, rotation = relative_transform(
        parent_origin, parent_rotation, child_origin, child_rotation)
    np.testing.assert_allclose(translation, local_origin, atol=1e-12)
    np.testing.assert_allclose(rotation, local_rotation, atol=1e-12)


def test_helpers_preserve_existing_command_and_odometry_behavior():
    assert velocity_to_policy(2.0, -2.0) == (0.5, -0.5)
    velocity = planar_velocity((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 2.0)
    np.testing.assert_allclose(velocity, (0.5, 0.0, 0.0))


def test_quaternion_is_normalized():
    quaternion = np.asarray(matrix_to_quaternion(rotation_z(0.8)))
    np.testing.assert_allclose(np.linalg.norm(quaternion), 1.0, atol=1e-12)
