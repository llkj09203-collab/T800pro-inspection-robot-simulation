import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_route = os.path.expanduser(
        "~/.ros/t800_routes/mixed_inspection.yaml")
    route_file = LaunchConfiguration("route_file")
    return LaunchDescription([
        DeclareLaunchArgument("route_file", default_value=default_route),
        DeclareLaunchArgument("map_name", default_value="mixed_map"),
        DeclareLaunchArgument("autosave", default_value="false"),
        DeclareLaunchArgument("loop", default_value="false"),
        Node(
            package="t800_waypoints",
            executable="waypoint_editor",
            name="t800_waypoint_editor",
            output="screen",
            parameters=[{
                "route_file": route_file,
                "map_name": LaunchConfiguration("map_name"),
                "autosave": LaunchConfiguration("autosave"),
            }],
        ),
        Node(
            package="t800_waypoints",
            executable="mission_manager",
            name="t800_mission_manager",
            output="screen",
            parameters=[{
                "route_file": route_file,
                "autostart": False,
                "loop": LaunchConfiguration("loop"),
            }],
        ),
    ])
