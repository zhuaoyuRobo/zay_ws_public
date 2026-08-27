#!/usr/bin/env python3
"""Safety bridge from Nav2 Twist commands to a real Go2W command topic."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def finite(value, default=0.0):
    value = float(value)
    if not math.isfinite(value):
        return default
    return value


def clamp(value, low, high):
    value = finite(value)
    return max(low, min(high, value))


class Go2WCmdVelBridge(Node):
    def __init__(self):
        super().__init__("go2w_cmd_vel_bridge")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/go2w/cmd_vel")
        self.declare_parameter("enable_output", False)
        # Public defaults intentionally keep hardware output disabled and slow.
        self.declare_parameter("command_timeout", 0.50)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("max_linear_x", 0.05)
        self.declare_parameter("max_linear_y", 0.0)
        self.declare_parameter("max_angular_z", 0.15)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("deadband_linear", 0.02)
        self.declare_parameter("deadband_angular", 0.05)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        if self.input_topic == self.output_topic:
            raise RuntimeError("input_topic and output_topic must be different")

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_timeout_warn = 0.0
        self.sent_stop = False

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.on_cmd, 10)

        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            "Go2W cmd bridge %s -> %s enable_output=%s max=(%.2f, %.2f, %.2f)"
            % (
                self.input_topic,
                self.output_topic,
                bool(self.get_parameter("enable_output").value),
                float(self.get_parameter("max_linear_x").value),
                float(self.get_parameter("max_linear_y").value),
                float(self.get_parameter("max_angular_z").value),
            )
        )

    def on_cmd(self, msg):
        out = Twist()
        max_x = abs(float(self.get_parameter("max_linear_x").value))
        max_y = abs(float(self.get_parameter("max_linear_y").value))
        max_yaw = abs(float(self.get_parameter("max_angular_z").value))
        allow_reverse = bool(self.get_parameter("allow_reverse").value)

        min_x = -max_x if allow_reverse else 0.0
        out.linear.x = clamp(msg.linear.x, min_x, max_x)
        out.linear.y = clamp(msg.linear.y, -max_y, max_y)
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = clamp(msg.angular.z, -max_yaw, max_yaw)

        linear_deadband = abs(float(self.get_parameter("deadband_linear").value))
        angular_deadband = abs(float(self.get_parameter("deadband_angular").value))
        if abs(out.linear.x) < linear_deadband:
            out.linear.x = 0.0
        if abs(out.linear.y) < linear_deadband:
            out.linear.y = 0.0
        if abs(out.angular.z) < angular_deadband:
            out.angular.z = 0.0

        self.last_cmd = out
        self.last_cmd_time = time.monotonic()
        self.sent_stop = False

    def on_timer(self):
        if not bool(self.get_parameter("enable_output").value):
            return

        now = time.monotonic()
        timeout = max(0.05, float(self.get_parameter("command_timeout").value))
        if now - self.last_cmd_time > timeout:
            if not self.sent_stop:
                self.pub.publish(Twist())
                self.sent_stop = True
            if now - self.last_timeout_warn > 2.0:
                self.get_logger().warn(
                    "No fresh %s for %.2fs; publishing stop on %s"
                    % (self.input_topic, timeout, self.output_topic)
                )
                self.last_timeout_warn = now
            return

        self.pub.publish(self.last_cmd)


def main():
    rclpy.init()
    node = Go2WCmdVelBridge()
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
