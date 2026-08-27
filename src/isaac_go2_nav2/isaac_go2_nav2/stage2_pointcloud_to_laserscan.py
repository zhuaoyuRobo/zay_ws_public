"""Stage 2 point cloud projection for the FAST_LIO OS0 route."""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class Stage2PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__("stage2_pointcloud_to_laserscan")

        self.declare_parameter("target_frame", "body")
        self.declare_parameter("transform_tolerance", 0.10)
        self.declare_parameter("use_tf", True)
        self.declare_parameter("stamp_mode", "cloud")
        self.declare_parameter("min_height", 0.05)
        self.declare_parameter("max_height", 0.85)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("scan_time", 0.10)
        self.declare_parameter("range_min", 0.30)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("use_inf", True)
        self.declare_parameter("inf_epsilon", 1.0)
        self.declare_parameter("self_filter_enabled", True)
        self.declare_parameter("self_filter_min_x", -0.45)
        self.declare_parameter("self_filter_max_x", 0.45)
        self.declare_parameter("self_filter_min_y", -0.32)
        self.declare_parameter("self_filter_max_y", 0.32)

        self._warned_frame_mismatch = False
        self._warned_bad_stamp_mode = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(LaserScan, "scan", 10)
        self._sub = self.create_subscription(PointCloud2, "cloud_in", self._on_cloud, 10)

    def _scan_stamp(self, msg: PointCloud2):
        stamp_mode = str(self.get_parameter("stamp_mode").value).lower()
        if stamp_mode == "cloud":
            return msg.header.stamp
        if stamp_mode == "now":
            return self.get_clock().now().to_msg()
        if not self._warned_bad_stamp_mode:
            self.get_logger().warning(
                "Unknown stamp_mode '%s'; falling back to current ROS time" % stamp_mode
            )
            self._warned_bad_stamp_mode = True
        return self.get_clock().now().to_msg()

    def _transform_xyz(self, points, transform):
        x = points["x"].astype(np.float32, copy=False)
        y = points["y"].astype(np.float32, copy=False)
        z = points["z"].astype(np.float32, copy=False)

        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = q.x
        qy = q.y
        qz = q.z
        qw = q.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            raise ValueError("transform quaternion has zero norm")
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        out_x = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z + tx
        out_y = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z + ty
        out_z = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z + tz
        return out_x.astype(np.float32), out_y.astype(np.float32), out_z.astype(np.float32)

    def _xyz_in_target_frame(self, msg: PointCloud2, target_frame: str):
        points = point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        source_frame = msg.header.frame_id
        if not target_frame or not source_frame or source_frame == target_frame:
            return msg.header, points["x"], points["y"], points["z"]

        if not bool(self.get_parameter("use_tf").value):
            if not self._warned_frame_mismatch:
                self.get_logger().warning(
                    "Publishing scan in target_frame without TF transform; "
                    "use a cloud already expressed in that frame."
                )
                self._warned_frame_mismatch = True
            return msg.header, points["x"], points["y"], points["z"]

        timeout = Duration(seconds=max(float(self.get_parameter("transform_tolerance").value), 0.0))
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=timeout,
            )
        except TransformException as exc:
            self.get_logger().warning(
                "Skipping cloud: no TF %s <- %s: %s"
                % (target_frame, source_frame, exc),
                throttle_duration_sec=1.0,
            )
            return None

        return transform.header, *self._transform_xyz(points, transform)

    def _on_cloud(self, msg: PointCloud2) -> None:
        target_frame = str(self.get_parameter("target_frame").value)
        min_height = float(self.get_parameter("min_height").value)
        max_height = float(self.get_parameter("max_height").value)
        angle_min = float(self.get_parameter("angle_min").value)
        angle_max = float(self.get_parameter("angle_max").value)
        angle_increment = float(self.get_parameter("angle_increment").value)
        scan_time = float(self.get_parameter("scan_time").value)
        range_min = float(self.get_parameter("range_min").value)
        range_max = float(self.get_parameter("range_max").value)
        use_inf = bool(self.get_parameter("use_inf").value)
        inf_epsilon = float(self.get_parameter("inf_epsilon").value)
        self_filter_enabled = bool(self.get_parameter("self_filter_enabled").value)
        self_filter_min_x = float(self.get_parameter("self_filter_min_x").value)
        self_filter_max_x = float(self.get_parameter("self_filter_max_x").value)
        self_filter_min_y = float(self.get_parameter("self_filter_min_y").value)
        self_filter_max_y = float(self.get_parameter("self_filter_max_y").value)

        if angle_increment <= 0.0 or angle_max <= angle_min:
            self.get_logger().error("Invalid scan angular bounds")
            return

        beam_count = int(math.floor((angle_max - angle_min) / angle_increment)) + 1
        if beam_count <= 0:
            return

        transformed = self._xyz_in_target_frame(msg, target_frame)
        if transformed is None:
            return
        scan_header, x, y, z = transformed

        fill_value = math.inf if use_inf else range_max + inf_epsilon
        scan_ranges = np.full(beam_count, fill_value, dtype=np.float32)

        if x.size:
            ranges = np.hypot(x, y)
            angles = np.arctan2(y, x)

            valid = (
                (z >= min_height)
                & (z <= max_height)
                & (ranges >= range_min)
                & (ranges <= range_max)
                & (angles >= angle_min)
                & (angles <= angle_max)
            )
            if self_filter_enabled:
                inside_self = (
                    (x >= self_filter_min_x)
                    & (x <= self_filter_max_x)
                    & (y >= self_filter_min_y)
                    & (y <= self_filter_max_y)
                )
                valid = valid & ~inside_self
            if np.any(valid):
                bins = np.floor((angles[valid] - angle_min) / angle_increment).astype(np.int32)
                bins = np.clip(bins, 0, beam_count - 1)
                np.minimum.at(scan_ranges, bins, ranges[valid])

        scan = LaserScan()
        scan.header = scan_header
        scan.header.stamp = self._scan_stamp(msg)
        if target_frame:
            scan.header.frame_id = target_frame
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = scan_time
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = scan_ranges.tolist()
        self._pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = Stage2PointCloudToLaserScan()
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
