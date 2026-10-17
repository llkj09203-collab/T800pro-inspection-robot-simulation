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
    forward, yaw = velocity_to_policy(2.0, -2.0)
    np.testing.assert_allclose(math.hypot(forward, yaw), 0.5, atol=1e-12)
    np.testing.assert_allclose(forward, -yaw, atol=1e-12)
    velocity = planar_velocity((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 2.0)
    np.testing.assert_allclose(velocity, (0.5, 0.0, 0.0))


def test_forward_command_is_fixed_and_reverse_is_blocked():
    assert velocity_to_policy(0.0, 0.0)[0] == 0.0
    assert velocity_to_policy(0.01, 0.0)[0] == 0.5
    assert velocity_to_policy(1.0, 0.0)[0] == 0.5
    assert velocity_to_policy(-1.0, 0.0)[0] == 0.0


def test_yaw_command_compensates_t800_dead_zone():
    assert velocity_to_policy(0.0, 0.0) == (0.0, 0.0)
    assert velocity_to_policy(0.0, 0.079)[1] == 0.0
    assert velocity_to_policy(0.0, -0.079)[1] == 0.0
    _, small_positive = velocity_to_policy(0.0, 0.081)
    _, small_negative = velocity_to_policy(0.0, -0.081)
    assert small_positive == 0.5
    assert small_negative == -0.5
    assert velocity_to_policy(0.0, 0.5)[1] == 0.5
    assert velocity_to_policy(0.0, -0.5)[1] == -0.5


def test_combined_command_preserves_normalized_input_direction():
    forward, yaw = velocity_to_policy(0.4, 0.1)
    np.testing.assert_allclose(math.hypot(forward, yaw), 0.5, atol=1e-12)
    np.testing.assert_allclose(forward / yaw, 4.0, atol=1e-12)


def test_sharp_turn_preserves_normalized_input_direction():
    forward, yaw = velocity_to_policy(0.2, -0.4)
    np.testing.assert_allclose(math.hypot(forward, yaw), 0.5, atol=1e-12)
    np.testing.assert_allclose(yaw / forward, -2.0, atol=1e-12)


def test_non_finite_velocity_is_safely_rejected():
    assert velocity_to_policy(float("nan"), 0.0) == (0.0, 0.0)
    assert velocity_to_policy(0.0, float("inf")) == (0.0, 0.0)


def test_quaternion_is_normalized():
    quaternion = np.asarray(matrix_to_quaternion(rotation_z(0.8)))
    np.testing.assert_allclose(np.linalg.norm(quaternion), 1.0, atol=1e-12)
