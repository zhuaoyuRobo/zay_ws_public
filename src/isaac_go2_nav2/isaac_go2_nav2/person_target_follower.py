#!/usr/bin/env python3
"""Send Nav2 goals from a moving person/target pose."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def fill_yaw_quat(orientation, yaw):
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw * 0.5)
    orientation.w = math.cos(yaw * 0.5)


class PersonTargetFollower(Node):
    """Convert a person pose topic or TF frame into NavigateToPose goals."""

    def __init__(self):
        super().__init__("person_target_follower")
        self.declare_parameter("target_topic", "/person_pose")
        self.declare_parameter("target_frame", "")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("follow_distance", 2.0)
        self.declare_parameter("update_rate", 0.5)
        self.declare_parameter("target_timeout", 1.0)
        self.declare_parameter("resend_period", 4.0)
        self.declare_parameter("min_goal_update_distance", 0.5)
        self.declare_parameter("goal_yaw_mode", "target")

        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)

        self.latest_pose = None
        self.latest_pose_time = 0.0
        self.last_goal_xy = None
        self.last_goal_time = 0.0
        self.last_server_log = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        if self.target_topic:
            self.create_subscription(PoseStamped, self.target_topic, self.on_target_pose, 10)
        self.create_timer(1.0 / float(self.get_parameter("update_rate").value), self.on_timer)

        self.get_logger().info(
            "Following target topic='%s' frame='%s' into Nav2 goals in frame '%s'"
            % (self.target_topic, self.target_frame, self.goal_frame)
        )

    def on_target_pose(self, msg):
        self.latest_pose = msg
        self.latest_pose_time = time.monotonic()

    def pose_to_goal_frame(self, pose_msg):
        source_frame = pose_msg.header.frame_id or self.goal_frame
        x = float(pose_msg.pose.position.x)
        y = float(pose_msg.pose.position.y)
        yaw = yaw_from_quat(pose_msg.pose.orientation)

        if source_frame == self.goal_frame:
            return x, y, yaw

        transform = self.tf_buffer.lookup_transform(
            self.goal_frame,
            source_frame,
            Time(),
        )
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tyaw = yaw_from_quat(transform.transform.rotation)
        c = math.cos(tyaw)
        s = math.sin(tyaw)
        return tx + c * x - s * y, ty + s * x + c * y, normalize_angle(tyaw + yaw)

    def target_from_tf(self):
        if not self.target_frame:
            return None
        transform = self.tf_buffer.lookup_transform(
            self.goal_frame,
            self.target_frame,
            Time(),
        )
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw_from_quat(transform.transform.rotation),
        )

    def current_target(self):
        now = time.monotonic()
        timeout = float(self.get_parameter("target_timeout").value)
        if self.latest_pose is not None and now - self.latest_pose_time <= timeout:
            return self.pose_to_goal_frame(self.latest_pose)
        return self.target_from_tf()

    def make_follow_goal(self, target):
        target_x, target_y, target_yaw = target
        follow_distance = float(self.get_parameter("follow_distance").value)
        goal_x = target_x - follow_distance * math.cos(target_yaw)
        goal_y = target_y - follow_distance * math.sin(target_yaw)

        mode = str(self.get_parameter("goal_yaw_mode").value)
        if mode == "zero":
            goal_yaw = 0.0
        elif mode == "face_target":
            goal_yaw = math.atan2(target_y - goal_y, target_x - goal_x)
        else:
            goal_yaw = target_yaw
        return goal_x, goal_y, normalize_angle(goal_yaw)

    def should_send(self, goal):
        now = time.monotonic()
        resend_period = float(self.get_parameter("resend_period").value)
        min_dist = float(self.get_parameter("min_goal_update_distance").value)
        if self.last_goal_xy is None:
            return True
        if now - self.last_goal_time >= resend_period:
            return True
        dx = goal[0] - self.last_goal_xy[0]
        dy = goal[1] - self.last_goal_xy[1]
        return math.hypot(dx, dy) >= min_dist

    def send_nav_goal(self, goal):
        now = time.monotonic()
        if not self.client.wait_for_server(timeout_sec=0.0):
            if now >= self.last_server_log:
                self.get_logger().warn("Nav2 action server navigate_to_pose is not ready yet.")
                self.last_server_log = now + 5.0
            return

        goal_x, goal_y, goal_yaw = goal
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self.goal_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal_x)
        msg.pose.pose.position.y = float(goal_y)
        fill_yaw_quat(msg.pose.pose.orientation, goal_yaw)

        self.client.send_goal_async(msg)
        self.last_goal_xy = (goal_x, goal_y)
        self.last_goal_time = now
        self.get_logger().info(
            "Sent follow goal x=%.2f y=%.2f yaw=%.2f" % (goal_x, goal_y, goal_yaw)
        )

    def on_timer(self):
        try:
            target = self.current_target()
        except TransformException as exc:
            self.get_logger().warn("Waiting for target transform: %s" % exc)
            return

        if target is None:
            return
        goal = self.make_follow_goal(target)
        if self.should_send(goal):
            self.send_nav_goal(goal)


def main():
    rclpy.init()
    node = PersonTargetFollower()
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
