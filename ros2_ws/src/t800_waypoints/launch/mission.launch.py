from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_route = os.path.expanduser(
        "~/.ros/t800_routes/mixed_inspection.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("route_file", default_value=default_route),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("loop", default_value="false"),
        Node(
            package="t800_waypoints",
            executable="mission_manager",
            name="t800_mission_manager",
            output="screen",
            parameters=[{
                "route_file": LaunchConfiguration("route_file"),
                "autostart": LaunchConfiguration("autostart"),
                "loop": LaunchConfiguration("loop"),
            }],
        ),
    ])
