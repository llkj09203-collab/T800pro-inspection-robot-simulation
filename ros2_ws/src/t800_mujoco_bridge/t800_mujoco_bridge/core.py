"""ROS-independent geometry and command helpers for the MuJoCo bridge."""

import math

import numpy as np


def matrix_to_quaternion(matrix):
    """Convert a finite right-handed 3x3 rotation matrix to x,y,z,w."""
    matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
    if not np.isfinite(matrix).all():
        raise ValueError("rotation matrix contains non-finite values")
    trace = np.trace(matrix)
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        return ((matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale, 0.25 * scale)
    index = int(np.argmax(np.diag(matrix)))
    if index == 0:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
        return (0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale)
    if index == 1:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
        return ((matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale)
    scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
    return ((matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
            (matrix[1, 0] - matrix[0, 1]) / scale)


def relative_transform(parent_origin, parent_matrix, child_origin, child_matrix):
    """Express a child's world pose in its parent's coordinate frame."""
    parent_origin = np.asarray(parent_origin, dtype=float).reshape(3)
    child_origin = np.asarray(child_origin, dtype=float).reshape(3)
    parent_matrix = np.asarray(parent_matrix, dtype=float).reshape(3, 3)
    child_matrix = np.asarray(child_matrix, dtype=float).reshape(3, 3)
    return (parent_matrix.T @ (child_origin - parent_origin),
            parent_matrix.T @ child_matrix)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def planar_velocity(previous_pose, pose, dt):
    if previous_pose is None or dt <= 0.0:
        return 0.0, 0.0, 0.0
    dx, dy = pose[0] - previous_pose[0], pose[1] - previous_pose[1]
    yaw = pose[2]
    return ((math.cos(yaw) * dx + math.sin(yaw) * dy) / dt,
            (-math.sin(yaw) * dx + math.cos(yaw) * dy) / dt,
            normalize_angle(pose[2] - previous_pose[2]) / dt)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def velocity_to_policy(linear_x, angular_z, forward_scale=1.0, yaw_scale=1.0):
    forward = float(np.clip(linear_x * forward_scale, -0.5, 0.5))
    yaw = float(np.clip(angular_z * yaw_scale, -0.5, 0.5))
    return forward, yaw
