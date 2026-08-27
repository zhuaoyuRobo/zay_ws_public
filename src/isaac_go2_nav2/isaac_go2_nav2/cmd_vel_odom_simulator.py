#!/usr/bin/env python3
"""Publish dead-reckoned odometry from /cmd_vel for low-config bring-up."""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class CmdVelOdomSimulator(Node):
    def __init__(self):
        super().__init__("cmd_vel_odom_simulator")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("cmd_timeout", 0.6)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("pause_when_external_odom", True)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = None
        self.last_step_time = self.get_clock().now()
        self.external_odom_active = False

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, self.get_parameter("cmd_vel_topic").value, self.on_cmd, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.on_timer)
        tf_note = " with TF" if bool(self.get_parameter("publish_tf").value) else ""
        self.get_logger().info(
            "Publishing fallback dead-reckoned %s%s from %s"
            % (
                self.odom_topic,
                tf_note,
                self.get_parameter("cmd_vel_topic").value,
            )
        )

    def on_cmd(self, msg):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def external_odom_present(self):
        if not bool(self.get_parameter("pause_when_external_odom").value):
            return False
        publishers = self.get_publishers_info_by_topic(self.odom_topic)
        for publisher in publishers:
            if (
                publisher.node_name != self.get_name()
                or publisher.node_namespace != self.get_namespace()
            ):
                return True
        return False

    def on_timer(self):
        if self.external_odom_present():
            if not self.external_odom_active:
                self.get_logger().info(
                    "External %s detected; pausing fallback odom publisher"
                    % self.odom_topic
                )
            self.external_odom_active = True
            self.last_step_time = self.get_clock().now()
            return
        if self.external_odom_active:
            self.get_logger().warn(
                "External %s disappeared; resuming fallback odom publisher"
                % self.odom_topic
            )
            self.external_odom_active = False

        now = self.get_clock().now()
        dt = (now - self.last_step_time).nanoseconds * 1e-9
        self.last_step_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / float(self.get_parameter("rate").value)

        if self.last_cmd_time is None:
            vx = vy = wz = 0.0
        else:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if age > float(self.get_parameter("cmd_timeout").value):
                vx = vy = wz = 0.0
            else:
                vx, vy, wz = self.vx, self.vy, self.wz

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (cos_yaw * vx - sin_yaw * vy) * dt
        self.y += (sin_yaw * vx + cos_yaw * vy) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        quat = quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if bool(self.get_parameter("publish_tf").value):
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = quat
            self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = CmdVelOdomSimulator()
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
