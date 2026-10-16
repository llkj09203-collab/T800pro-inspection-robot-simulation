from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cyclone_config = (
        '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>')
    default_config = str(
        Path(get_package_share_directory("t800_mujoco_bridge")) / "config" / "bridge.yaml"
    )
    return LaunchDescription([
        SetEnvironmentVariable("CYCLONEDDS_URI", cyclone_config),
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="t800_mujoco_bridge",
            executable="sim_bridge",
            name="t800_mujoco_bridge",
            output="screen",
            parameters=[LaunchConfiguration("config")],
        ),
    ])
