import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("t800_stage3_navigation")
    bridge_share = get_package_share_directory("t800_mujoco_bridge")
    params = os.path.join(share, "config", "traversability.yaml")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "bridge.launch.py"))),
        Node(package="t800_stage3_navigation", executable="elevation_mapper",
             parameters=[params], output="screen"),
    ])
