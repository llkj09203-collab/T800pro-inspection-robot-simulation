"""Send one Nav2 goal and record success, failure, cancellation or timeout."""
import csv
from pathlib import Path
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from .core import yaw_to_quaternion


class SingleGoal(Node):
    def __init__(self):
        super().__init__("t800_single_goal")
        for name, default in (("x", 2.0), ("y", 0.0), ("yaw", 0.0), ("timeout", 120.0)):
            self.declare_parameter(name, default)
        self.declare_parameter("goal_name", "P01")
        self.declare_parameter("log_file", "p00_p01.csv")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.started = None
        self.finished = False
        self.get_logger().warning("真机运行前确认急停可用并安排现场监护；软件取消不能替代硬件急停")
        self.create_timer(0.2, self._check_timeout)

    def start(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self._finish("SERVER_UNAVAILABLE", "Nav2 action server unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter("x").value)
        goal.pose.pose.position.y = float(self.get_parameter("y").value)
        q = yaw_to_quaternion(float(self.get_parameter("yaw").value))
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = q
        self.started = time.monotonic()
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self._finish("REJECTED", "goal rejected")
            return
        self.get_logger().info(f"{self.get_parameter('goal_name').value} accepted")
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self._result)

    def _feedback(self, feedback):
        remaining = feedback.feedback.distance_remaining
        self.get_logger().info(f"remaining {remaining:.2f} m", throttle_duration_sec=2.0)

    def _result(self, future):
        status = future.result().status
        names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        self._finish(names.get(status, f"STATUS_{status}"), "navigation finished")

    def _check_timeout(self):
        if self.finished or self.started is None:
            return
        if time.monotonic() - self.started <= float(self.get_parameter("timeout").value):
            return
        if self.goal_handle is None:
            self._finish("TIMEOUT", "timed out before goal acceptance")
        else:
            self.get_logger().error("Navigation timeout; cancelling goal")
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(lambda _future: self._finish("TIMEOUT_CANCELED", "timeout cancellation sent"))

    def _finish(self, result, detail):
        if self.finished:
            return
        self.finished = True
        elapsed = 0.0 if self.started is None else time.monotonic() - self.started
        path = Path(str(self.get_parameter("log_file").value)).expanduser()
        new_file = not path.exists()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            if new_file:
                writer.writerow(["timestamp", "goal", "x", "y", "yaw", "result", "elapsed_s", "detail"])
            writer.writerow([time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                             self.get_parameter("goal_name").value,
                             self.get_parameter("x").value, self.get_parameter("y").value,
                             self.get_parameter("yaw").value, result, f"{elapsed:.3f}", detail])
        self.get_logger().info(f"result={result}, elapsed={elapsed:.1f}s, log={path}")


def main(args=None):
    rclpy.init(args=args)
    node = SingleGoal()
    node.start()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node._finish("USER_CANCELED", "keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
