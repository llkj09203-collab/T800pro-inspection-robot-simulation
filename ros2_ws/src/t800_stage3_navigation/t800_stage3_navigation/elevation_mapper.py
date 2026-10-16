"""Convert GLIM's global 3D cloud into robot-aware 2.5D map layers."""

from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from .traversability import TraversabilityConfig, build_traversability_grid


class ElevationMapper(Node):
    def __init__(self):
        super().__init__("glim_elevation_mapper")
        defaults = TraversabilityConfig()
        for name in (
            "resolution", "max_step_height", "max_slope_deg", "max_roughness",
            "robot_clearance_height", "ground_band", "min_points_per_cell",
            "padding_cells", "ground_quantile", "ground_seed_bin",
            "ground_seed_tolerance", "inflation_radius",
        ):
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter("input_topic", "/glim_ros/map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("elevation_topic", "/elevation_map/points")
        self.declare_parameter("save_path", "/tmp/t800_25d_map")
        self.declare_parameter("minimum_update_period", 1.0)
        self.last_update_ns = 0
        self.latest = None
        self.latest_frame = "glim_map"

        input_qos = QoSProfile(depth=1)
        input_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        input_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("map_topic").value, output_qos)
        self.elevation_pub = self.create_publisher(
            PointCloud2, self.get_parameter("elevation_topic").value, output_qos)
        self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value,
            self._cloud_callback, input_qos)
        self.create_service(Trigger, "save_elevation_map", self._save)
        self.get_logger().info(
            f"GLIM 2.5D mapper ready: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('map_topic').value}")

    def _config(self):
        return TraversabilityConfig(**{
            name: self.get_parameter(name).value for name in (
                "resolution", "max_step_height", "max_slope_deg", "max_roughness",
                "robot_clearance_height", "ground_band", "min_points_per_cell",
                "padding_cells", "ground_quantile", "ground_seed_bin",
                "ground_seed_tolerance", "inflation_radius",
            )
        })

    def _cloud_callback(self, message):
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + message.header.stamp.nanosec
        period_ns = int(float(self.get_parameter("minimum_update_period").value) * 1e9)
        if self.last_update_ns and stamp_ns - self.last_update_ns < period_ns:
            return
        try:
            points = point_cloud2.read_points_numpy(
                message, field_names=("x", "y", "z"), skip_nans=True)
            points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
            grid = build_traversability_grid(points, self._config())
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"Rejected GLIM map cloud: {exc}")
            return
        if grid is None:
            return
        self.last_update_ns = stamp_ns
        self.latest = grid
        self.latest_frame = message.header.frame_id or "glim_map"
        self._publish_grid(grid, message.header.stamp)

    def _publish_grid(self, grid, stamp):
        occupancy = OccupancyGrid()
        occupancy.header.stamp = stamp
        occupancy.header.frame_id = self.latest_frame
        occupancy.info.map_load_time = stamp
        occupancy.info.resolution = grid.resolution
        occupancy.info.width = grid.width
        occupancy.info.height = grid.height
        occupancy.info.origin.position.x = grid.origin_x
        occupancy.info.origin.position.y = grid.origin_y
        occupancy.info.origin.orientation.w = 1.0
        occupancy.data = grid.cost.reshape(-1).astype(np.int8).tolist()
        self.map_pub.publish(occupancy)

        known_y, known_x = np.where(grid.cost >= 0)
        rows = np.column_stack((
            grid.origin_x + (known_x + 0.5) * grid.resolution,
            grid.origin_y + (known_y + 0.5) * grid.resolution,
            grid.ground[known_y, known_x],
            grid.ground[known_y, known_x],
            grid.ceiling[known_y, known_x],
            grid.roughness[known_y, known_x],
            grid.slope_deg[known_y, known_x],
            grid.cost[known_y, known_x].astype(np.float32),
        )).astype(np.float32)
        header = Header(stamp=stamp, frame_id=self.latest_frame)
        fields = [
            PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate((
                "x", "y", "z", "ground_z", "ceiling_z", "roughness", "slope_deg",
                "traversability_cost",
            ))
        ]
        self.elevation_pub.publish(point_cloud2.create_cloud(header, fields, rows.tolist()))

    def _save(self, _request, response):
        if self.latest is None:
            response.success = False
            response.message = "no GLIM elevation map has been received"
            return response
        target = Path(self.get_parameter("save_path").value).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        grid = self.latest
        np.savez_compressed(
            str(target) + ".npz", frame_id=self.latest_frame,
            origin=np.array([grid.origin_x, grid.origin_y]), resolution=grid.resolution,
            ground=grid.ground, ceiling=grid.ceiling, mean=grid.mean,
            roughness=grid.roughness, slope_deg=grid.slope_deg,
            count=grid.count, cost=grid.cost,
        )
        response.success = True
        response.message = str(target) + ".npz"
        return response


def main():
    rclpy.init()
    node = ElevationMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
