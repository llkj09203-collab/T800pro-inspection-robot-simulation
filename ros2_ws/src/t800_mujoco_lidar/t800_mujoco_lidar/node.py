"""ROS 2 node exposing the T800 MuJoCo ray lidar as PointCloud2."""

import os
from pathlib import Path
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from .lcm_source import LcmStateSource
from .pointcloud import make_pointcloud
from .raycaster import Raycaster


class MujocoLidarNode(Node):
    def __init__(self):
        super().__init__("t800_mujoco_lidar")
        defaults = {
            "lcm_url": "udpm://239.255.76.67:7667?ttl=1",
            "state_channel": "lidar/state",
            "robot_model": "t800pro",
            "sdk_root": os.environ.get(
                "ENGINEAI_ROBOTICS_DIR", "/home/llkj/engineai_robotics_native_sdk"),
            "model_path": "",
            "mount_camera": "depth",
            "frame_id": "lidar_link",
            "topic": "/lidar/points_raw",
            "horizontal_rays": 240,
            "vertical_rays": 180,
            "horizontal_fov_deg": 120.0,
            "vertical_up_deg": 33.1,
            "vertical_down_deg": 56.9,
            "min_range_m": 0.2,
            "max_range_m": 30.0,
            "update_rate_hz": 10.0,
            "state_timeout_s": 0.8,
            "angular_stride": 1,
            "visualize_rays_in_mujoco": True,
            "max_visualized_rays": 500,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        if not 0.2 <= values["state_timeout_s"] <= 5.0:
            raise ValueError("state_timeout_s must be in [0.2, 5.0]")
        if not 0.2 <= values["update_rate_hz"] <= 10.0:
            raise ValueError("update_rate_hz must be in [0.2, 10.0]")
        if not 1 <= values["max_visualized_rays"] <= 2000:
            raise ValueError("max_visualized_rays must be in [1, 2000]")
        self.values = values
        self.raycaster = None
        self.active_scenario = None
        self.visualized_ray_mask = None
        self.source = LcmStateSource(
            values["lcm_url"], values["state_channel"], values["state_timeout_s"])
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(PointCloud2, values["topic"], qos)
        self.frame_id = values["frame_id"]
        self.timeout = values["state_timeout_s"]
        self.last_stamp = 0.0
        self.last_wait_log = 0.0
        self.frames = 0
        self.timer = self.create_timer(1.0 / values["update_rate_hz"], self._capture)
        self.get_logger().info(
            f'3D lidar ready: {values["topic"]} [{self.frame_id}]; '
            'waiting to discover the active MuJoCo scene')

    def _configure_scene(self, scenario):
        model_path = self.values["model_path"]
        if not model_path:
            filename = (f'{self.values["robot_model"]}.xml' if scenario == "default"
                        else f'{self.values["robot_model"]}_lidar_{scenario}.xml')
            model_path = str(
                Path(self.values["sdk_root"]) / "assets" / "resource" / filename)
        if not Path(model_path).is_file():
            raise FileNotFoundError(
                f'No lidar model for MuJoCo scenario "{scenario}": {model_path}')
        self.raycaster = Raycaster(
            model_path=model_path,
            mount_camera=self.values["mount_camera"],
            horizontal_rays=self.values["horizontal_rays"],
            vertical_rays=self.values["vertical_rays"],
            horizontal_fov_deg=self.values["horizontal_fov_deg"],
            vertical_up_deg=self.values["vertical_up_deg"],
            vertical_down_deg=self.values["vertical_down_deg"],
            min_range_m=self.values["min_range_m"],
            max_range_m=self.values["max_range_m"],
            angular_stride=self.values["angular_stride"],
        )
        self.active_scenario = scenario
        self.last_stamp = 0.0
        self.get_logger().info(
            f'Automatically selected MuJoCo scenario "{scenario}": {model_path}')

    def _capture(self):
        state = self.source.get()
        now = time.time()
        if state is None:
            if now - self.last_wait_log >= 2.0:
                self.get_logger().warning(
                    "Waiting for a fresh lidar/state snapshot from MuJoCo")
                self.last_wait_log = now
            return
        age = now - float(state["stamp"])
        if not -0.1 <= age <= self.timeout:
            if now - self.last_wait_log >= 2.0:
                self.get_logger().warning(
                    f"Latest lidar/state snapshot is stale ({age:.3f}s old)")
                self.last_wait_log = now
            return
        if state["stamp"] <= self.last_stamp:
            return
        try:
            scenario = state["scenario"]
            if self.raycaster is None or scenario != self.active_scenario:
                self._configure_scene(scenario)
            points, rings, ray_ids = self.raycaster.capture(state)
            message = make_pointcloud(points, rings, state["stamp"], self.frame_id)
            self.publisher.publish(message)
            if self.values["visualize_rays_in_mujoco"]:
                maximum = int(self.values["max_visualized_rays"])
                if self.visualized_ray_mask is None:
                    width = (int(self.values["horizontal_rays"])
                             // int(self.values["angular_stride"]))
                    height = (int(self.values["vertical_rays"])
                              // int(self.values["angular_stride"]))
                    columns = max(1, round(np.sqrt(maximum * width / height)))
                    rows = max(1, maximum // columns)
                    column_ids = np.linspace(0, width - 1, columns, dtype=np.int64)
                    row_ids = np.linspace(0, height - 1, rows, dtype=np.int64)
                    selected = (row_ids[:, None] * width + column_ids[None, :]).ravel()
                    self.visualized_ray_mask = np.zeros(width * height, dtype=bool)
                    self.visualized_ray_mask[selected] = True
                    self.get_logger().info(
                        f'Visualizing a fixed {columns} x {rows} angular grid '
                        f'({len(selected)} rays) in MuJoCo')
                # The grid is fixed in both azimuth and elevation. Rays without
                # a return are omitted until they hit a surface.
                sampled = points[self.visualized_ray_mask[ray_ids]]
                payload = struct.pack("<IdI", 0x3156524c, state["stamp"], len(sampled))
                payload += sampled.astype("<f4", copy=False).tobytes()
                self.source.publish_debug_rays(payload)
            self.last_stamp = float(state["stamp"])
            self.frames += 1
            if self.frames == 1 or self.frames % 100 == 0:
                self.get_logger().info(
                    f"Published frame {self.frames}: {len(points)} points, "
                    f"sample_stamp={state['stamp']:.6f}")
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(f"Rejected lidar frame: {exc}")

    def destroy_node(self):
        self.source.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoLidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"t800_mujoco_lidar: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
