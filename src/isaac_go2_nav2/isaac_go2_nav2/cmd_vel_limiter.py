#!/usr/bin/env python3
"""Clamp keyboard Twist commands before they reach the Go2 policy controller."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def clamp(value, limit):
    if not math.isfinite(float(value)):
        return 0.0
    if limit <= 0.0:
        return float(value)
    return max(-limit, min(limit, float(value)))


class CmdVelLimiter(Node):
    def __init__(self):
        super().__init__("cmd_vel_limiter")
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        # Conservative public demonstration limits, not calibrated hardware values.
        self.declare_parameter("max_linear_x", 0.05)
        self.declare_parameter("max_linear_y", 0.0)
        self.declare_parameter("max_angular_z", 0.15)

        self.last_warn = 0.0
        self.pub = self.create_publisher(
            Twist,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("input_topic").value,
            self.on_cmd,
            10,
        )

        self.get_logger().info(
            "Limiting %s -> %s to x=%.2f y=%.2f yaw=%.2f"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
                float(self.get_parameter("max_linear_x").value),
                float(self.get_parameter("max_linear_y").value),
                float(self.get_parameter("max_angular_z").value),
            )
        )

    def on_cmd(self, msg):
        max_x = float(self.get_parameter("max_linear_x").value)
        max_y = float(self.get_parameter("max_linear_y").value)
        max_yaw = float(self.get_parameter("max_angular_z").value)

        out = Twist()
        out.linear.x = clamp(msg.linear.x, max_x)
        out.linear.y = clamp(msg.linear.y, max_y)
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = clamp(msg.angular.z, max_yaw)
        self.pub.publish(out)

        raw = (msg.linear.x, msg.linear.y, msg.angular.z)
        limited = (out.linear.x, out.linear.y, out.angular.z)
        changed = any(
            math.isfinite(raw_value)
            and abs(raw_value - limited_value) > 1e-6
            for raw_value, limited_value in zip(raw, limited)
        )
        if changed and time.monotonic() >= self.last_warn:
            self.get_logger().warn(
                "Clamped cmd_vel_raw from (%.2f %.2f %.2f) to (%.2f %.2f %.2f)"
                % (*raw, *limited)
            )
            self.last_warn = time.monotonic() + 2.0


def main():
    rclpy.init()
    node = CmdVelLimiter()
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
