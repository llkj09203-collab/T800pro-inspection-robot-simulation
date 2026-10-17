import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.actions import Node

from t800_stage3_navigation.scene_selection import resolve_scene_map


def generate_launch_description():
    cyclone_config = (
        '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>')
    navigation_share = get_package_share_directory("t800_stage3_navigation")
    active_scene, selected_map = resolve_scene_map(navigation_share)
    bridge_share = get_package_share_directory("t800_mujoco_bridge")
    nav2_share = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(nav2_share, "params", "nav2_params.yaml")
    overrides_file = os.path.join(
        navigation_share, "config", "nav2_overrides.yaml")
    with open(overrides_file, "r", encoding="utf-8") as stream:
        navigation_overrides = yaml.safe_load(stream)
    parameter_rewrites = {
        name: str(value)
        for name, value in navigation_overrides["parameter_rewrites"].items()
    }
    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    velocity_params = ReplaceString(
        source_file=LaunchConfiguration("params_file"),
        replacements=navigation_overrides["text_replacements"])
    configured_params = RewrittenYaml(
        source_file=velocity_params,
        root_key="",
        param_rewrites=parameter_rewrites,
        convert_types=True)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": selected_map,
            "params_file": configured_params,
            "use_sim_time": "false",
            "autostart": "true",
            "use_composition": "False",
        }.items())
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "bridge.launch.py")))
    scan_params = os.path.join(
        navigation_share, "config", "pointcloud_to_laserscan.yaml")
    scan_converter = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[scan_params],
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
        bridge,
        scan_converter,
        bringup,
    ])
