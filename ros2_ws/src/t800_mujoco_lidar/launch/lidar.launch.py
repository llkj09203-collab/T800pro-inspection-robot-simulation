from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cyclone_config = (
        '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>')
    default_config = str(
        Path(get_package_share_directory("t800_mujoco_lidar")) / "config" / "lidar.yaml"
    )
    return LaunchDescription([
        SetEnvironmentVariable("CYCLONEDDS_URI", cyclone_config),
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("visualize_rays_in_mujoco", default_value="true"),
        Node(
            package="t800_mujoco_lidar",
            executable="lidar_node",
            name="t800_mujoco_lidar",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "model_path": LaunchConfiguration("model_path"),
                    "visualize_rays_in_mujoco": ParameterValue(
                        LaunchConfiguration("visualize_rays_in_mujoco"), value_type=bool),
                },
            ],
        ),
    ])
