#!/usr/bin/env python3
"""Generate a lightweight LaserScan from the static Nav2 map and /odom."""

import ast
import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_map_yaml(path):
    values = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()

    image = Path(values.get("image", "maze.pgm"))
    if not image.is_absolute():
        image = Path(path).parent / image
    return {
        "image": image,
        "resolution": float(values.get("resolution", "0.05")),
        "origin": ast.literal_eval(values.get("origin", "[0.0, 0.0, 0.0]")),
        "occupied_thresh": float(values.get("occupied_thresh", "0.65")),
        "negate": int(values.get("negate", "0")),
    }


def read_pgm(path):
    tokens = []
    for raw_line in Path(path).read_text(encoding="ascii").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            tokens.extend(line.split())
    if tokens[0] != "P2":
        raise ValueError("Only ASCII P2 PGM maps are supported.")
    width = int(tokens[1])
    height = int(tokens[2])
    max_value = int(tokens[3])
    pixels = np.asarray([int(value) for value in tokens[4:]], dtype=np.float32)
    if pixels.size != width * height:
        raise ValueError("PGM pixel count mismatch.")
    return width, height, max_value, pixels.reshape((height, width))


class MapScanSimulator(Node):
    """Publish map raycasts as a LaserScan for low-load Isaac demos."""

    def __init__(self):
        super().__init__("map_scan_simulator")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("frame_id", "base_laser")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("initial_x", -6.0)
        self.declare_parameter("initial_y", -4.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(1.0))
        self.declare_parameter("range_min", 0.08)
        self.declare_parameter("range_max", 6.0)
        self.declare_parameter("ray_step", 0.04)

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
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.get_parameter("scan_topic").value,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.on_odom,
            20,
        )
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.on_timer)
        self.get_logger().info(
            "Generating %s from %s using map %s"
            % (
                self.get_parameter("scan_topic").value,
                self.get_parameter("odom_topic").value,
                self.get_parameter("map_yaml").value,
            )
        )

    def on_odom(self, msg):
        self.latest_odom = msg

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
        return range_max

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

    def on_timer(self):
        if self.latest_odom is None:
            return

        angle_min = float(self.get_parameter("angle_min").value)
        angle_max = float(self.get_parameter("angle_max").value)
        angle_increment = float(self.get_parameter("angle_increment").value)
        count = int(round((angle_max - angle_min) / angle_increment)) + 1
        x, y, yaw = self.odom_as_map_pose()
        ranges = [
            self.raycast(x, y, yaw, angle_min + index * angle_increment)
            for index in range(count)
        ]

        msg = LaserScan()
        msg.header.stamp = self.latest_odom.header.stamp
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / float(self.get_parameter("rate").value)
        msg.range_min = float(self.get_parameter("range_min").value)
        msg.range_max = float(self.get_parameter("range_max").value)
        msg.ranges = [float(value) for value in ranges]
        msg.intensities = []
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = MapScanSimulator()
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
