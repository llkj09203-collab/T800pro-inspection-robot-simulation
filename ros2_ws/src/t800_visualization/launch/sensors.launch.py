import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("t800_visualization")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "rviz.launch.py")),
            launch_arguments={
                "config": os.path.join(share, "rviz", "sensors.rviz"),
            }.items(),
        ),
    ])
