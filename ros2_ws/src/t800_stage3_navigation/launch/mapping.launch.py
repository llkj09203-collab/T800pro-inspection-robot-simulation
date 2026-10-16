import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("t800_stage3_navigation")
    bridge_share = get_package_share_directory("t800_mujoco_bridge")
    slam_config = os.path.join(share, "config", "slam.yaml")
    scan_config = os.path.join(share, "config", "pointcloud_to_laserscan.yaml")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "bridge.launch.py"))),
        Node(package="pointcloud_to_laserscan",
             executable="pointcloud_to_laserscan_node",
             name="pointcloud_to_laserscan", output="screen",
             parameters=[scan_config],
             remappings=[("cloud_in", "/lidar/points_raw"), ("scan", "/scan")]),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", output="screen",
             parameters=[slam_config]),
    ])
