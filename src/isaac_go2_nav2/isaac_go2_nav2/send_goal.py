#!/usr/bin/env python3
"""Send a single NavigateToPose goal to Nav2."""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalSender(Node):
    def __init__(self):
        super().__init__("isaac_go2_goal_sender")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send_goal(self, x, y, yaw, frame_id):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(frame_id)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
        goal.pose.pose.orientation.w = math.cos(float(yaw) * 0.5)

        self.get_logger().info(
            "Sending goal frame=%s x=%.2f y=%.2f yaw=%.2f"
            % (goal.pose.header.frame_id, float(x), float(y), float(yaw))
        )
        self.get_logger().info("Waiting for Nav2 action server...")
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False
        self.get_logger().info("Goal accepted")
        return True


def main():
    x, y, yaw = 6.0, 4.0, 0.0
    frame_id = "map"
    if len(sys.argv) >= 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    if len(sys.argv) >= 4:
        yaw = float(sys.argv[3])
    if len(sys.argv) >= 5:
        frame_id = sys.argv[4]

    rclpy.init()
    node = GoalSender()
    try:
        node.send_goal(x, y, yaw, frame_id)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
