import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from t800_navigation.scene_selection import resolve_scene_map


def generate_launch_description():
    cyclone_config = (
        '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>')
    navigation_share = get_package_share_directory("t800_navigation")
    active_scene, selected_map = resolve_scene_map(navigation_share)
    bridge_share = get_package_share_directory("t800_mujoco_bridge")
    nav2_share = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(
        navigation_share, "config", "navigation", "nav2_params.yaml")
    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Complete Nav2 parameter file")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": selected_map,
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "False",
        }.items())
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "bridge.launch.py")))
    scan_params = os.path.join(
        navigation_share, "config", "mapping_2d",
        "pointcloud_to_laserscan.yaml")
    scan_converter = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[scan_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cloud_in", "/lidar/points_raw"),
            ("scan", "/scan"),
        ],
    )
    return LaunchDescription([
        SetEnvironmentVariable("CYCLONEDDS_URI", cyclone_config),
        LogInfo(msg=(
            f'Using navigation map for MuJoCo scene "{active_scene["scene"]}": '
            f"{selected_map}")),
        params_arg,
        use_sim_time_arg,
        bridge,
        scan_converter,
        bringup,
    ])
