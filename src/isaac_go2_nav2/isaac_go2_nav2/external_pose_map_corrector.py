#!/usr/bin/env python3
"""Publish map -> odom_frame from an external global pose observation.

The local odometry still owns odom_frame -> base_frame.  This node computes the
upper correction:

    map -> odom_frame = map -> base_frame * inverse(odom_frame -> base_frame)

For Isaac simulation, /odom can be used as the external observation in
"relative" mode with an external origin equal to the robot spawn pose in map.
For a real AprilTag/localization pipeline, publish the robot pose in map and
use "map" mode.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def lerp_angle(current, target, alpha):
    return yaw_wrap(current + alpha * yaw_wrap(target - current))


def quat_from_yaw(yaw):
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def rpy_from_quat(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def pose_in_origin(x, y, yaw, x0, y0, yaw0):
    c = math.cos(yaw0)
    s = math.sin(yaw0)
    return x0 + c * x - s * y, y0 + s * x + c * y, yaw_wrap(yaw0 + yaw)


class ExternalPoseMapCorrector(Node):
    def __init__(self):
        super().__init__("external_pose_map_corrector")

        self.declare_parameter("local_odom_topic", "/Odometry_nav2")
        self.declare_parameter("external_pose_topic", "/odom")
        self.declare_parameter("external_pose_type", "odometry")
        self.declare_parameter("external_pose_mode", "relative")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "camera_init")
        self.declare_parameter("base_frame", "body_nav")
        self.declare_parameter("external_origin_x", -6.0)
        self.declare_parameter("external_origin_y", -4.0)
        self.declare_parameter("external_origin_yaw", 0.0)
        self.declare_parameter("correction_alpha", 0.2)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("max_abs_roll_pitch", 0.8)
        self.declare_parameter("max_correction_step", 0.5)
        self.declare_parameter("max_yaw_correction_step", 0.6)
        self.declare_parameter("log_period", 2.0)

        self.local_pose = None
        self.external_pose = None
        self.filtered_correction = None
        self.last_log_time = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.local_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("local_odom_topic").value),
            self.on_local_odom,
            20,
        )

        external_pose_type = str(self.get_parameter("external_pose_type").value).lower()
        external_pose_topic = str(self.get_parameter("external_pose_topic").value)
        if external_pose_type in ("pose", "pose_stamped", "posestamped"):
            self.external_sub = self.create_subscription(
                PoseStamped,
                external_pose_topic,
                self.on_external_pose_stamped,
                20,
            )
        elif external_pose_type in ("odom", "odometry"):
            self.external_sub = self.create_subscription(
                Odometry,
                external_pose_topic,
                self.on_external_odom,
                20,
            )
        else:
            self.get_logger().warning(
                "Unknown external_pose_type '%s'; using nav_msgs/Odometry" % external_pose_type
            )
            self.external_sub = self.create_subscription(
                Odometry,
                external_pose_topic,
                self.on_external_odom,
                20,
            )

        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            "Correcting %s -> %s from external %s (%s) and local %s"
            % (
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("odom_frame").value),
                external_pose_topic,
                str(self.get_parameter("external_pose_mode").value),
                str(self.get_parameter("local_odom_topic").value),
            )
        )

    def _pose_from_msg(self, pose_msg):
        roll, pitch, yaw = rpy_from_quat(pose_msg.orientation)
        max_abs_roll_pitch = float(self.get_parameter("max_abs_roll_pitch").value)
        if max_abs_roll_pitch >= 0.0 and max(abs(roll), abs(pitch)) > max_abs_roll_pitch:
            return None
        p = pose_msg.position
        values = [p.x, p.y, p.z, roll, pitch, yaw]
        if not all(math.isfinite(float(value)) for value in values):
            return None
        return float(p.x), float(p.y), yaw

    def _external_pose_in_map(self, pose_msg):
        pose = self._pose_from_msg(pose_msg)
        if pose is None:
            return None

        mode = str(self.get_parameter("external_pose_mode").value).lower()
        if mode == "map":
            return pose
        if mode != "relative":
            self.get_logger().warning(
                "Unknown external_pose_mode '%s'; treating as relative" % mode,
                throttle_duration_sec=2.0,
            )

        return pose_in_origin(
            pose[0],
            pose[1],
            pose[2],
            float(self.get_parameter("external_origin_x").value),
            float(self.get_parameter("external_origin_y").value),
            float(self.get_parameter("external_origin_yaw").value),
        )

    def on_local_odom(self, msg):
        pose = self._pose_from_msg(msg.pose.pose)
        if pose is None:
            self.get_logger().warning("Ignoring invalid local odometry", throttle_duration_sec=2.0)
            return
        self.local_pose = pose

    def on_external_odom(self, msg):
        pose = self._external_pose_in_map(msg.pose.pose)
        if pose is None:
            self.get_logger().warning("Ignoring invalid external odometry", throttle_duration_sec=2.0)
            return
        self.external_pose = pose

    def on_external_pose_stamped(self, msg):
        pose = self._external_pose_in_map(msg.pose)
        if pose is None:
            self.get_logger().warning("Ignoring invalid external PoseStamped", throttle_duration_sec=2.0)
            return
        self.external_pose = pose

    def _target_correction(self):
        if self.local_pose is None or self.external_pose is None:
            return None

        local_x, local_y, local_yaw = self.local_pose
        external_x, external_y, external_yaw = self.external_pose
        correction_yaw = yaw_wrap(external_yaw - local_yaw)
        c = math.cos(correction_yaw)
        s = math.sin(correction_yaw)
        correction_x = external_x - (c * local_x - s * local_y)
        correction_y = external_y - (s * local_x + c * local_y)
        return correction_x, correction_y, correction_yaw

    def _filtered_target(self, target):
        alpha = clamp(float(self.get_parameter("correction_alpha").value), 0.0, 1.0)
        if self.filtered_correction is None or alpha >= 1.0:
            return target

        current_x, current_y, current_yaw = self.filtered_correction
        target_x, target_y, target_yaw = target
        return (
            current_x + alpha * (target_x - current_x),
            current_y + alpha * (target_y - current_y),
            lerp_angle(current_yaw, target_yaw, alpha),
        )

    def _step_is_valid(self, target):
        max_step = float(self.get_parameter("max_correction_step").value)
        max_yaw_step = float(self.get_parameter("max_yaw_correction_step").value)
        if self.filtered_correction is None:
            return True
        current_x, current_y, current_yaw = self.filtered_correction
        dx = target[0] - current_x
        dy = target[1] - current_y
        yaw_step = abs(yaw_wrap(target[2] - current_yaw))
        if max_step >= 0.0 and math.hypot(dx, dy) > max_step:
            return False
        if max_yaw_step >= 0.0 and yaw_step > max_yaw_step:
            return False
        return True

    def on_timer(self):
        target = self._target_correction()
        if target is None:
            return
        if not self._step_is_valid(target):
            self.get_logger().warning(
                "Skipping external correction jump to map->%s: x=%.2f y=%.2f yaw=%.1fdeg"
                % (
                    str(self.get_parameter("odom_frame").value),
                    target[0],
                    target[1],
                    math.degrees(target[2]),
                ),
                throttle_duration_sec=2.0,
            )
            return

        self.filtered_correction = self._filtered_target(target)
        self._publish_correction(self.filtered_correction)
        self._maybe_log(target)

    def _publish_correction(self, correction):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter("map_frame").value)
        transform.child_frame_id = str(self.get_parameter("odom_frame").value)
        transform.transform.translation.x = correction[0]
        transform.transform.translation.y = correction[1]
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quat_from_yaw(correction[2])
        self.tf_broadcaster.sendTransform(transform)

    def _maybe_log(self, target):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        log_period = float(self.get_parameter("log_period").value)
        if self.last_log_time is not None and now - self.last_log_time < log_period:
            return
        self.last_log_time = now
        correction = self.filtered_correction or target
        self.get_logger().info(
            "map->%s correction x=%.2f y=%.2f yaw=%.1fdeg target=(%.2f, %.2f, %.1fdeg)"
            % (
                str(self.get_parameter("odom_frame").value),
                correction[0],
                correction[1],
                math.degrees(correction[2]),
                target[0],
                target[1],
                math.degrees(target[2]),
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExternalPoseMapCorrector()
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
