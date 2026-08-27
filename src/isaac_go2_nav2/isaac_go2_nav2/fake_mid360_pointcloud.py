#!/usr/bin/env python3
"""Publish a lightweight MID360-like PointCloud2 from the static Nav2 map."""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from isaac_go2_nav2.map_scan_simulator import read_map_yaml, read_pgm, yaw_from_quat


class FakeMID360PointCloud(Node):
    """Raycast the known map and publish hits as a fake MID360 point cloud."""

    def __init__(self):
        super().__init__("fake_mid360_pointcloud")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("initial_x", -6.0)
        self.declare_parameter("initial_y", -4.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("range_min", 0.25)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("ray_step", 0.04)
        self.declare_parameter("z_layers", [-0.03, 0.0, 0.03])

        map_yaml = str(self.get_parameter("map_yaml").value)
        if not map_yaml:
            raise ValueError("map_yaml must point to a user-supplied ASCII P2 map")
        map_info = read_map_yaml(map_yaml)
        self.width, self.height, self.max_value, self.pixels = read_pgm(map_info["image"])
        normalized = self.pixels / float(self.max_value)
        occupancy = normalized if map_info["negate"] else 1.0 - normalized
        self.occupied = occupancy >= map_info["occupied_thresh"]
        self.resolution = map_info["resolution"]
        self.origin_x = float(map_info["origin"][0])
        self.origin_y = float(map_info["origin"][1])

        self.latest_odom = None
        self.last_publish_wall_time = 0.0
        self.publish_period = 1.0 / max(float(self.get_parameter("rate").value), 0.001)
        self.cloud_pub = self.create_publisher(
            point_cloud2.PointCloud2,
            self.get_parameter("cloud_topic").value,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.on_odom,
            20,
        )
        self.get_logger().info(
            "Publishing fake MID360 PointCloud2 on %s from map %s"
            % (
                self.get_parameter("cloud_topic").value,
                self.get_parameter("map_yaml").value,
            )
        )

    def on_odom(self, msg):
        self.latest_odom = msg
        now = time.monotonic()
        if now - self.last_publish_wall_time >= self.publish_period:
            self.last_publish_wall_time = now
            self.publish_cloud()

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row_from_bottom = int((y - self.origin_y) / self.resolution)
        row = self.height - 1 - row_from_bottom
        return row, col

    def is_occupied_world(self, x, y):
        row, col = self.world_to_grid(x, y)
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return True
        return bool(self.occupied[row, col])

    def raycast(self, x, y, yaw, rel_angle):
        range_min = float(self.get_parameter("range_min").value)
        range_max = float(self.get_parameter("range_max").value)
        step = float(self.get_parameter("ray_step").value)
        angle = yaw + rel_angle
        c = math.cos(angle)
        s = math.sin(angle)
        distance = range_min
        while distance <= range_max:
            if self.is_occupied_world(x + c * distance, y + s * distance):
                return distance
            distance += step
        return None

    def odom_as_map_pose(self):
        msg = self.latest_odom
        x0 = float(self.get_parameter("initial_x").value)
        y0 = float(self.get_parameter("initial_y").value)
        yaw0 = float(self.get_parameter("initial_yaw").value)
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oyaw = yaw_from_quat(msg.pose.pose.orientation)
        c = math.cos(yaw0)
        s = math.sin(yaw0)
        return x0 + c * ox - s * oy, y0 + s * ox + c * oy, yaw0 + oyaw

    def publish_cloud(self):
        if self.latest_odom is None:
            return

        angle_min = float(self.get_parameter("angle_min").value)
        angle_max = float(self.get_parameter("angle_max").value)
        angle_increment = float(self.get_parameter("angle_increment").value)
        count = int(round((angle_max - angle_min) / angle_increment)) + 1
        x, y, yaw = self.odom_as_map_pose()
        z_layers = [float(value) for value in self.get_parameter("z_layers").value]

        points = []
        for index in range(count):
            rel_angle = angle_min + index * angle_increment
            distance = self.raycast(x, y, yaw, rel_angle)
            if distance is None:
                continue
            point_x = distance * math.cos(rel_angle)
            point_y = distance * math.sin(rel_angle)
            for point_z in z_layers:
                points.append((point_x, point_y, point_z))

        header = Header()
        header.stamp = self.latest_odom.header.stamp
        header.frame_id = str(self.get_parameter("frame_id").value)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))


def main():
    rclpy.init()
    node = FakeMID360PointCloud()
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
