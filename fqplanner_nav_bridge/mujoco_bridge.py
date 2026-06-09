#!/usr/bin/env python3
"""Bridge an FQPlanner MuJoCo HTTP backend to ROS2 navigation topics."""

from __future__ import annotations

import json
import math
import threading
import urllib.error
import urllib.parse
import urllib.request

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("fqplanner_mujoco_bridge")
        self.declare_parameter("backend_url", "http://127.0.0.1:5001")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("laser_z", 0.25)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("scan_rate", 5.0)
        self.declare_parameter("http_timeout", 0.5)
        self.declare_parameter("cmd_vel_duration", 0.25)
        self.declare_parameter("cmd_vel_linear_scale", 1.0)
        self.declare_parameter("cmd_vel_angular_scale", 1.0)
        self.declare_parameter("scan_angle_min", -math.pi)
        self.declare_parameter("scan_angle_max", math.pi)
        self.declare_parameter("scan_angle_increment", math.radians(1.0))
        self.declare_parameter("scan_range_min", 0.05)
        self.declare_parameter("scan_range_max", 5.0)
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("map_x_min", -1.0)
        self.declare_parameter("map_x_max", 8.0)
        self.declare_parameter("map_y_min", -6.0)
        self.declare_parameter("map_y_max", 1.0)
        self.declare_parameter("publish_initial_pose", False)
        self.declare_parameter("initial_pose_repeats", 10)
        self.declare_parameter("initial_pose_period", 1.0)
        self.declare_parameter("fake_localization", False)

        self.backend_url = self.get_parameter("backend_url").value.rstrip("/")
        self.http_timeout = float(self.get_parameter("http_timeout").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.laser_frame = str(self.get_parameter("laser_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.cmd_lock = threading.Lock()
        self.initial_pose_sent = 0
        self.initial_pose_done = False

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.on_amcl_pose, 10)

        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        scan_rate = max(0.5, float(self.get_parameter("scan_rate").value))
        self.create_timer(1.0 / publish_rate, self.publish_odom)
        self.create_timer(1.0 / scan_rate, self.publish_scan)
        if bool(self.get_parameter("publish_initial_pose").value):
            period = max(0.2, float(self.get_parameter("initial_pose_period").value))
            self.create_timer(period, self.publish_initial_pose)
        self.publish_static_tf()
        self.get_logger().info(f"FQPlanner MuJoCo bridge connected to {self.backend_url}")

    def request_json(self, method: str, endpoint: str, payload=None):
        endpoint = endpoint if endpoint.startswith("/") else f"/{endpoint}"
        url = self.backend_url + endpoint
        if method == "POST":
            req = urllib.request.Request(
                url,
                data=json.dumps(payload or {}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
        else:
            req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=self.http_timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def publish_static_tf(self):
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.laser_frame
        tf.transform.translation.z = float(self.get_parameter("laser_z").value)
        tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf)

    def publish_odom(self):
        try:
            data = self.request_json("GET", "/base_status")
        except Exception as exc:
            self.get_logger().debug(f"base_status failed: {exc}")
            return

        pos = data.get("pos") or [0.0, 0.0, 0.0]
        yaw = float(data.get("yaw_rad", math.radians(float(data.get("yaw_deg", 0.0)))))
        qx, qy, qz, qw = yaw_to_quat(yaw)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2]) if len(pos) > 2 else 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        qvel = data.get("qvel") or [0.0, 0.0, 0.0]
        if len(qvel) >= 3:
            odom.twist.twist.linear.x = float(qvel[0])
            odom.twist.twist.linear.y = float(qvel[1])
            odom.twist.twist.angular.z = float(qvel[2])
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        # Fake localization: publish identity map->odom so Nav2 skips AMCL
        if bool(self.get_parameter("fake_localization").value):
            map_tf = TransformStamped()
            map_tf.header.stamp = stamp
            map_tf.header.frame_id = self.map_frame
            map_tf.child_frame_id = self.odom_frame
            map_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(map_tf)

    def publish_scan(self):
        params = {
            "angle_min": self.get_parameter("scan_angle_min").value,
            "angle_max": self.get_parameter("scan_angle_max").value,
            "angle_increment": self.get_parameter("scan_angle_increment").value,
            "range_min": self.get_parameter("scan_range_min").value,
            "range_max": self.get_parameter("scan_range_max").value,
            "resolution": self.get_parameter("map_resolution").value,
            "x_min": self.get_parameter("map_x_min").value,
            "x_max": self.get_parameter("map_x_max").value,
            "y_min": self.get_parameter("map_y_min").value,
            "y_max": self.get_parameter("map_y_max").value,
        }
        endpoint = "/scan?" + urllib.parse.urlencode(params)
        try:
            data = self.request_json("GET", endpoint)
        except Exception as exc:
            self.get_logger().debug(f"scan failed: {exc}")
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(data.get("frame_id") or self.laser_frame)
        msg.angle_min = float(data.get("angle_min", params["angle_min"]))
        msg.angle_max = float(data.get("angle_max", params["angle_max"]))
        msg.angle_increment = float(data.get("angle_increment", params["angle_increment"]))
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / max(0.5, float(self.get_parameter("scan_rate").value))
        msg.range_min = float(data.get("range_min", params["range_min"]))
        msg.range_max = float(data.get("range_max", params["range_max"]))
        msg.ranges = [
            math.inf if value is None else float(value)
            for value in data.get("ranges", [])
        ]
        self.scan_pub.publish(msg)

    def publish_initial_pose(self):
        max_repeats = int(self.get_parameter("initial_pose_repeats").value)
        if self.initial_pose_done or self.initial_pose_sent >= max_repeats:
            return
        try:
            data = self.request_json("GET", "/base_status")
        except Exception as exc:
            self.get_logger().debug(f"initialpose source failed: {exc}")
            return

        pos = data.get("pos") or [0.0, 0.0, 0.0]
        yaw = float(data.get("yaw_rad", math.radians(float(data.get("yaw_deg", 0.0)))))
        qx, qy, qz, qw = yaw_to_quat(yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(msg)
        self.initial_pose_sent += 1
        self.get_logger().info(
            f"Published initial pose {self.initial_pose_sent}/{max_repeats}: "
            f"x={pos[0]:.2f}, y={pos[1]:.2f}, yaw={math.degrees(yaw):.1f} deg"
        )

    def on_amcl_pose(self, _msg: PoseWithCovarianceStamped):
        if not self.initial_pose_done:
            self.initial_pose_done = True
            self.get_logger().info("AMCL pose received; initial pose publishing stopped")

    def on_cmd_vel(self, msg: Twist):
        payload = {
            "vx": float(msg.linear.x) * float(self.get_parameter("cmd_vel_linear_scale").value),
            "vy": float(msg.linear.y) * float(self.get_parameter("cmd_vel_linear_scale").value),
            "vw": float(msg.angular.z) * float(self.get_parameter("cmd_vel_angular_scale").value),
            "duration": float(self.get_parameter("cmd_vel_duration").value),
        }
        with self.cmd_lock:
            try:
                self.request_json("POST", "/cmd_vel", payload)
            except (urllib.error.URLError, TimeoutError, OSError) as exc:
                self.get_logger().debug(f"cmd_vel forward failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
