"""RViz-backed waypoint editor using draggable interactive markers."""

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from .routes import Route, Waypoint, load_route, save_route, yaw_from_quaternion


class WaypointEditor(Node):
    def __init__(self):
        super().__init__("t800_waypoint_editor")
        self.declare_parameter("route_file", str(
            Path.home() / ".ros" / "t800_routes" / "mixed_inspection.yaml"))
        self.declare_parameter("map_name", "mixed_map")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("add_topic", "/t800/waypoints/add_pose")
        self.declare_parameter("autosave", False)
        self.route_path = Path(str(self.get_parameter("route_file").value)).expanduser()
        self.route = load_route(self.route_path)
        if not self.route.map:
            self.route.map = str(self.get_parameter("map_name").value)
        if not self.route.frame_id:
            self.route.frame_id = str(self.get_parameter("frame_id").value)
        self.server = InteractiveMarkerServer(self, "t800_waypoints")
        self.create_subscription(
            PoseStamped, str(self.get_parameter("add_topic").value), self._add_pose, 10)
        self.create_service(Trigger, "/t800/waypoints/save", self._save)
        self.create_service(Trigger, "/t800/waypoints/reload", self._reload)
        self.create_service(Trigger, "/t800/waypoints/delete_last", self._delete_last)
        self.create_service(Trigger, "/t800/waypoints/clear", self._clear)
        self._rebuild_markers()
        self.get_logger().info(
            f"Waypoint editor ready: {len(self.route.waypoints)} points, file={self.route_path}")

    def _next_name(self):
        used = {waypoint.name for waypoint in self.route.waypoints}
        index = 1
        while f"P{index:02d}" in used:
            index += 1
        return f"P{index:02d}"

    def _add_pose(self, message):
        if message.header.frame_id and message.header.frame_id != self.route.frame_id:
            self.get_logger().error(
                f"Cannot add waypoint in {message.header.frame_id}; expected {self.route.frame_id}")
            return
        waypoint = Waypoint(
            self._next_name(), message.pose.position.x, message.pose.position.y,
            yaw_from_quaternion(message.pose.orientation))
        self.route.waypoints.append(waypoint)
        self._insert_marker(waypoint)
        self.server.applyChanges()
        self._autosave()
        self.get_logger().info(f"Added {waypoint.name} at ({waypoint.x:.2f}, {waypoint.y:.2f})")

    def _insert_marker(self, waypoint):
        marker = InteractiveMarker()
        marker.header.frame_id = self.route.frame_id
        marker.name = waypoint.name
        marker.description = waypoint.name
        marker.scale = 0.8
        marker.pose.position.x = waypoint.x
        marker.pose.position.y = waypoint.y
        marker.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        marker.pose.orientation.w = math.cos(waypoint.yaw / 2.0)

        visual = InteractiveMarkerControl()
        visual.always_visible = True
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = 0.65
        arrow.scale.y = 0.16
        arrow.scale.z = 0.16
        arrow.color.r = 0.1
        arrow.color.g = 0.8
        arrow.color.b = 0.25
        arrow.color.a = 1.0
        visual.markers.append(arrow)
        marker.controls.append(visual)

        # Match wp_map_tools: two explicit translation handles in the map
        # plane, plus a separate yaw ring. Explicit handles are easier to grab
        # reliably in RViz than a hidden MOVE_PLANE control.
        move_x = InteractiveMarkerControl()
        move_x.name = "move_x"
        move_x.orientation.w = 1.0
        move_x.orientation.x = 1.0
        move_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move_x)

        move_y = InteractiveMarkerControl()
        move_y.name = "move_y"
        move_y.orientation.w = 1.0
        move_y.orientation.z = 1.0
        move_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move_y)

        rotate = InteractiveMarkerControl()
        rotate.name = "rotate_yaw"
        rotate.orientation.w = 1.0
        rotate.orientation.y = 1.0
        rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(rotate)
        self.server.insert(marker, feedback_callback=self._feedback)

    def _feedback(self, feedback):
        waypoint = next((item for item in self.route.waypoints
                         if item.name == feedback.marker_name), None)
        if waypoint is None:
            return
        waypoint.x = feedback.pose.position.x
        waypoint.y = feedback.pose.position.y
        waypoint.yaw = yaw_from_quaternion(feedback.pose.orientation)
        self._autosave()

    def _rebuild_markers(self):
        self.server.clear()
        for waypoint in self.route.waypoints:
            self._insert_marker(waypoint)
        self.server.applyChanges()

    def _autosave(self):
        if bool(self.get_parameter("autosave").value):
            save_route(self.route_path, self.route)

    def _save(self, _request, response):
        save_route(self.route_path, self.route)
        response.success = True
        response.message = f"saved {len(self.route.waypoints)} waypoints to {self.route_path}"
        return response

    def _reload(self, _request, response):
        try:
            self.route = load_route(self.route_path)
            self._rebuild_markers()
            response.success = True
            response.message = f"loaded {len(self.route.waypoints)} waypoints"
        except (OSError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _delete_last(self, _request, response):
        if not self.route.waypoints:
            response.success = False
            response.message = "route is already empty"
            return response
        removed = self.route.waypoints.pop()
        self.server.erase(removed.name)
        self.server.applyChanges()
        self._autosave()
        response.success = True
        response.message = f"deleted {removed.name}"
        return response

    def _clear(self, _request, response):
        count = len(self.route.waypoints)
        self.route.waypoints.clear()
        self._rebuild_markers()
        self._autosave()
        response.success = True
        response.message = f"cleared {count} waypoints"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointEditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
