"""Bridge MuJoCo's native LCM state/control streams to standard ROS 2 topics."""

import json
import math
import threading
import time

import lcm
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .core import (
    matrix_to_quaternion,
    planar_velocity,
    relative_transform,
    velocity_to_policy,
    yaw_to_quaternion,
)


def make_transform(stamp, parent, child, translation, rotation):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    qx, qy, qz, qw = matrix_to_quaternion(rotation)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    return transform


def timestamp_message(timestamp):
    seconds = math.floor(float(timestamp))
    nanoseconds = int(round((float(timestamp) - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    from builtin_interfaces.msg import Time
    return Time(sec=int(seconds), nanosec=nanoseconds)


class SimBridge(Node):
    def __init__(self):
        super().__init__("t800_mujoco_bridge")
        self.declare_parameter("lcm_url", "udpm://239.255.76.67:7667?ttl=1")
        self.declare_parameter("state_channel", "lidar/state")
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("forward_scale", 1.0)
        self.declare_parameter("yaw_scale", 1.0)
        self.declare_parameter("imu_angular_velocity_noise_stddev", 0.0)
        self.declare_parameter("imu_linear_acceleration_noise_stddev", 0.0)
        self.declare_parameter("imu_dropout_probability", 0.0)

        self.lc = lcm.LCM(self.get_parameter("lcm_url").value)
        self.lc.subscribe(
            self.get_parameter("state_channel").value,
            self._state_callback).set_queue_capacity(1)
        self.lc.subscribe("lidar/reset_ack", self._reset_ack_callback).set_queue_capacity(1)
        self.lc.subscribe("lidar/imu", self._imu_callback).set_queue_capacity(1000)

        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.collision_pub = self.create_publisher(Bool, "simulation_collision", 10)
        self.imu_pub = self.create_publisher(Imu, "lidar/imu", 50)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.static_transforms_sent = False
        self.create_subscription(Twist, "cmd_vel", self._velocity_callback, 10)
        self.create_service(Trigger, "reset_simulation", self._reset_simulation)

        self.last_command = 0.0
        self.previous_pose = None
        self.previous_sim_time = None
        self.forward = self.yaw = 0.0
        self.stop = True
        self.reset_sequence = 0
        self.reset_ack = 0
        self.reset_event = threading.Event()
        self.running = True
        self.random = np.random.default_rng()
        self.thread = threading.Thread(target=self._lcm_loop, daemon=True)
        self.thread.start()
        self.create_timer(0.02, self._publish_command)
        self.get_logger().info(
            "MuJoCo bridge ready: odom, IMU, TF and guarded cmd_vel")

    def _lcm_loop(self):
        while self.running:
            self.lc.handle_timeout(100)

    def _velocity_callback(self, message):
        self.forward, self.yaw = velocity_to_policy(
            message.linear.x, message.angular.z,
            self.get_parameter("forward_scale").value,
            self.get_parameter("yaw_scale").value)
        self.last_command = time.monotonic()
        self.stop = False

    def _publish_command(self):
        timed_out = time.monotonic() - self.last_command > self.get_parameter(
            "command_timeout").value
        forward, yaw = ((0.0, 0.0) if self.stop or timed_out
                        else (self.forward, self.yaw))
        packet = f"LIDAR1 {time.time():.6f} {forward:.6f} {yaw:.6f} {int(self.stop or timed_out)}"
        self.lc.publish("lidar/control", packet.encode("ascii"))

    def _reset_ack_callback(self, _channel, payload):
        try:
            tag, sequence = payload.decode("ascii").split()
            if tag == "RESET_ACK1":
                self.reset_ack = int(sequence)
                self.reset_event.set()
        except (UnicodeDecodeError, ValueError):
            return

    def _reset_simulation(self, _request, response):
        self.stop = True
        self.forward = self.yaw = 0.0
        self._publish_command()
        time.sleep(0.6)
        self.reset_sequence += 1
        sequence = self.reset_sequence
        self.reset_event.clear()
        self.lc.publish(
            "lidar/reset", f"RESET1 {time.time():.6f} {sequence}".encode("ascii"))
        acknowledged = self.reset_event.wait(timeout=2.0) and self.reset_ack == sequence
        if acknowledged:
            self.previous_pose = None
            self.previous_sim_time = None
        response.success = acknowledged
        response.message = ("simulation reset acknowledged" if acknowledged
                            else "simulation reset timed out")
        return response

    def _state_callback(self, _channel, payload):
        try:
            data = json.loads(payload)
            if data.get("version") != 3:
                raise ValueError("unsupported state protocol")
            self._publish_state(data)
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f"Rejected malformed MuJoCo state: {error}", throttle_duration_sec=2.0)

    def _publish_state(self, data):
        stamp = timestamp_message(data["stamp"])
        x, y, yaw = map(float, data["pose"])
        pose = (x, y, yaw)
        sim_time = float(data["sim_time"])
        dt = 0.0 if self.previous_sim_time is None else sim_time - self.previous_sim_time
        linear_x, linear_y, angular_z = planar_velocity(self.previous_pose, pose, dt)
        self.previous_pose, self.previous_sim_time = pose, sim_time
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom)

        base_origin = data["base_origin"]
        base_world = np.asarray(data["base_matrix"], dtype=float).reshape(3, 3)
        pitch_origin = data["head_pitch_origin"]
        pitch_world = np.asarray(data["head_pitch_matrix"], dtype=float).reshape(3, 3)
        yaw_origin = data["head_yaw_origin"]
        yaw_world = np.asarray(data["head_yaw_matrix"], dtype=float).reshape(3, 3)
        imu_origin = data["head_imu_origin"]
        imu_world = np.asarray(data["head_imu_matrix"], dtype=float).reshape(3, 3)
        lidar_origin = data["origin"]
        camera_world = np.asarray(data["sensor_matrix"], dtype=float).reshape(3, 3)
        camera_from_lidar = np.array([[0.0, -1.0, 0.0],
                                      [0.0, 0.0, 1.0],
                                      [-1.0, 0.0, 0.0]])

        pitch_xyz, pitch_rotation = relative_transform(
            base_origin, base_world, pitch_origin, pitch_world)
        yaw_xyz, yaw_rotation = relative_transform(
            pitch_origin, pitch_world, yaw_origin, yaw_world)
        imu_xyz, imu_rotation = relative_transform(
            yaw_origin, yaw_world, imu_origin, imu_world)
        lidar_xyz, lidar_rotation = relative_transform(
            imu_origin, imu_world, lidar_origin, camera_world @ camera_from_lidar)
        transforms = [
            make_transform(stamp, "odom", "base_link", base_origin, base_world),
            make_transform(stamp, "base_link", "head_pitch_link", pitch_xyz, pitch_rotation),
            make_transform(stamp, "head_pitch_link", "head_yaw_link", yaw_xyz, yaw_rotation),
            make_transform(stamp, "head_yaw_link", "sensor_imu_link", imu_xyz, imu_rotation),
            make_transform(stamp, "sensor_imu_link", "lidar_link", lidar_xyz, lidar_rotation),
        ]
        self.tf_pub.sendTransform(transforms)
        if not self.static_transforms_sent:
            footprint = TransformStamped()
            footprint.header.stamp = stamp
            footprint.header.frame_id = "base_link"
            footprint.child_frame_id = "base_footprint"
            footprint.transform.rotation.w = 1.0
            self.static_tf_pub.sendTransform(footprint)
            self.static_transforms_sent = True

        collision = Bool()
        collision.data = bool(data.get("collision", False))
        self.collision_pub.publish(collision)

    def _imu_callback(self, _channel, payload):
        try:
            data = json.loads(payload)
            if data.get("version") != 1:
                raise ValueError("unsupported IMU protocol")
            dropout = float(self.get_parameter("imu_dropout_probability").value)
            if not 0.0 <= dropout <= 1.0:
                raise ValueError("imu_dropout_probability must be in [0, 1]")
            if self.random.random() < dropout:
                return
            quaternion = np.asarray(data["quaternion_wxyz"], dtype=float)
            acceleration = np.asarray(data["linear_acceleration"], dtype=float)
            angular_velocity = np.asarray(data["angular_velocity"], dtype=float)
            if (quaternion.shape != (4,) or acceleration.shape != (3,) or
                    angular_velocity.shape != (3,) or not np.isfinite(np.concatenate(
                        (quaternion, acceleration, angular_velocity))).all()):
                raise ValueError("invalid IMU values")
            angular_std = float(self.get_parameter(
                "imu_angular_velocity_noise_stddev").value)
            acceleration_std = float(self.get_parameter(
                "imu_linear_acceleration_noise_stddev").value)
            if angular_std < 0.0 or acceleration_std < 0.0:
                raise ValueError("IMU noise standard deviations must be non-negative")
            angular_velocity += self.random.normal(0.0, angular_std, 3)
            acceleration += self.random.normal(0.0, acceleration_std, 3)

            message = Imu()
            message.header.stamp = timestamp_message(data["stamp"])
            message.header.frame_id = "sensor_imu_link"
            message.orientation.w, message.orientation.x, message.orientation.y, \
                message.orientation.z = map(float, quaternion)
            message.angular_velocity.x, message.angular_velocity.y, \
                message.angular_velocity.z = map(float, angular_velocity)
            message.linear_acceleration.x, message.linear_acceleration.y, \
                message.linear_acceleration.z = map(float, acceleration)
            for index in (0, 4, 8):
                message.angular_velocity_covariance[index] = angular_std ** 2
                message.linear_acceleration_covariance[index] = acceleration_std ** 2
            self.imu_pub.publish(message)
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f"Rejected malformed head IMU packet: {error}", throttle_duration_sec=2.0)

    def destroy_node(self):
        self.stop = True
        self._publish_command()
        self.running = False
        self.thread.join(timeout=0.3)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
