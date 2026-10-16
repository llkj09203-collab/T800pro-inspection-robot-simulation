"""Small ROS-independent helpers used by navigation clients."""
import math

def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
