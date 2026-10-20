import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("t800_navigation")
    bridge_share = get_package_share_directory("t800_mujoco_bridge")
    glim_config = os.path.join(
        share, "config", "elevation", "glim_cpu")
    params = os.path.join(
        share, "config", "elevation", "traversability.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_glim", default_value="true",
            description="Start glim_rosnode as part of the mapping pipeline"),
        DeclareLaunchArgument(
            "glim_config", default_value=glim_config,
            description="Absolute path to the GLIM configuration directory"),
        LogInfo(msg=(
            "Default mapping pipeline: GLIM -> /glim_ros/map -> "
            "elevation_mapper.")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "bridge.launch.py"))),
        Node(
            package="glim_ros",
            executable="glim_rosnode",
            name="glim_ros",
            output="screen",
            parameters=[{"config_path": LaunchConfiguration("glim_config")}],
            condition=IfCondition(LaunchConfiguration("start_glim"))),
        Node(package="t800_navigation", executable="elevation_mapper",
             parameters=[params], output="screen"),
    ])
