"""Execute a saved waypoint route one Nav2 goal at a time."""

import json
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .routes import load_route, quaternion_from_yaw


class MissionManager(Node):
    def __init__(self):
        super().__init__("t800_mission_manager")
        self.declare_parameter("route_file", str(
            Path.home() / ".ros" / "t800_routes" / "mixed_inspection.yaml"))
        self.declare_parameter("autostart", True)
        self.declare_parameter("loop", False)
        self.declare_parameter("goal_timeout", 180.0)
        self.declare_parameter("start_index", 0)
        self.route_path = Path(str(self.get_parameter("route_file").value)).expanduser()
        self.route = load_route(self.route_path)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.events = self.create_publisher(String, "/t800/mission/events", 10)
        self.create_service(Trigger, "/t800/mission/start", self._start_service)
        self.create_service(Trigger, "/t800/mission/cancel", self._cancel_service)
        self.index = int(self.get_parameter("start_index").value)
        self.goal_handle = None
        self.goal_started = None
        self.wait_until = None
        self.active = False
        self.finished = False
        self.start_requested = bool(self.get_parameter("autostart").value)
        self.timer = self.create_timer(0.2, self._tick)
        self.get_logger().warning(
            "真机运行前确认急停可用并安排现场监护；任务取消不能替代硬件急停")

    def _event(self, event, waypoint=None, detail=""):
        payload = {"event": event, "index": self.index, "detail": detail}
        if waypoint is not None:
            payload["waypoint"] = waypoint.name
            payload["action"] = waypoint.action
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.events.publish(message)
        self.get_logger().info(message.data)

    def _start_service(self, _request, response):
        if self.active:
            response.success = False
            response.message = "mission is already active"
        else:
            try:
                self.route = load_route(self.route_path)
            except (OSError, ValueError) as exc:
                response.success = False
                response.message = f"cannot load route: {exc}"
                return response
            if not self.route.waypoints:
                response.success = False
                response.message = "route has no waypoints"
                return response
            if self.finished or self.index >= len(self.route.waypoints):
                self.index = 0
            self.finished = False
            self.start_requested = True
            response.success = True
            response.message = "mission start requested"
        return response

    def _cancel_service(self, _request, response):
        self.start_requested = False
        self.active = False
        self.wait_until = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            response.message = "mission cancellation requested"
        else:
            response.message = "mission stopped"
        response.success = True
        self._event("mission_canceled")
        return response

    def _tick(self):
        if self.goal_handle is not None and self.goal_started is not None:
            if time.monotonic() - self.goal_started > float(
                    self.get_parameter("goal_timeout").value):
                self._event("goal_timeout", self.route.waypoints[self.index])
                self.goal_handle.cancel_goal_async()
                self.active = False
                self.start_requested = False
            return
        if self.wait_until is not None:
            if time.monotonic() < self.wait_until:
                return
            self.wait_until = None
            self._advance()
            return
        if not self.start_requested or self.active or self.finished:
            return
        if not self.route.waypoints:
            self._event("mission_failed", detail="route has no waypoints")
            self.finished = True
            return
        if not self.client.wait_for_server(timeout_sec=0.0):
            return
        self.start_requested = False
        self.active = True
        self._send_current()

    def _send_current(self):
        waypoint = self.route.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.route.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        quaternion = quaternion_from_yaw(waypoint.yaw)
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = quaternion
        self.goal_started = time.monotonic()
        self._event("goal_sent", waypoint)
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.goal_started = None
            self.active = False
            self.finished = True
            self._event("mission_failed", self.route.waypoints[self.index], "goal rejected")
            return
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self._goal_result)

    def _goal_result(self, future):
        status = future.result().status
        waypoint = self.route.waypoints[self.index]
        self.goal_handle = None
        self.goal_started = None
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.active = False
            self.finished = True
            self._event("mission_failed", waypoint, f"Nav2 status {status}")
            return
        self._event("waypoint_arrived", waypoint)
        # Every arrival is published above. External inspection/camera nodes can
        # bind to that event; the built-in executor provides deterministic wait.
        wait = waypoint.stay_seconds
        if waypoint.action not in {"none", "wait"}:
            self._event("action_requested", waypoint)
        if wait > 0.0:
            self.wait_until = time.monotonic() + wait
        else:
            self._advance()

    def _advance(self):
        self.index += 1
        if self.index >= len(self.route.waypoints):
            if bool(self.get_parameter("loop").value):
                self.index = 0
            else:
                self.active = False
                self.finished = True
                self._event("mission_completed")
                return
        self._send_current()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.shutdown()
