import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    cyclone_config = (
        '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>')
    navigation_share = get_package_share_directory("t800_stage3_navigation")
    bridge_share = get_package_share_directory("t800_mujoco_bridge")
    nav2_share = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(nav2_share, "params", "nav2_params.yaml")
    map_arg = DeclareLaunchArgument(
        "map",
        description=(
            "Absolute path to the YAML map matching the active MuJoCo scene; "
            "no map is selected implicitly"),
    )
    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    velocity_params = ReplaceString(
        source_file=LaunchConfiguration("params_file"),
        replacements={
            "    alpha1: 0.2": (
                "    set_initial_pose: true\n"
                "    initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}\n"
                "    alpha1: 0.2"),
            "max_velocity: [0.26, 0.0, 1.0]": "max_velocity: [0.5, 0.0, 0.5]",
            "min_velocity: [-0.26, 0.0, -1.0]": "min_velocity: [-0.5, 0.0, -0.5]",
        })
    configured_params = RewrittenYaml(
        source_file=velocity_params,
        root_key="",
        param_rewrites={
            "base_frame_id": "base_link",
            "robot_base_frame": "base_link",
            "bt_loop_duration": "100",
            "default_server_timeout": "1000",
            "controller_frequency": "10.0",
            "transform_tolerance": "0.5",
            "z_resolution": "0.15",
            "max_vel_x": "0.5",
            "min_vel_x": "0.0",
            "max_vel_theta": "0.5",
            "max_speed_xy": "0.5",
            "xy_goal_tolerance": "0.20",
            "yaw_goal_tolerance": "3.14",
            "vx_samples": "10",
            "vtheta_samples": "10",
            "expected_planner_frequency": "1.0",
        },
        convert_types=True)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
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
        map_arg,
        params_arg,
        bridge,
        scan_converter,
        bringup,
    ])
