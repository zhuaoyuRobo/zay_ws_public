import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class OdomToImu(Node):
    """Publish a lightweight IMU signal from Isaac odometry for the Go2 policy."""

    def __init__(self):
        super().__init__("odom_to_imu")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("imu_frame", "imu_link")

        self.last_stamp = None
        self.last_linear = None

        self.pub = self.create_publisher(
            Imu,
            self.get_parameter("imu_topic").value,
            20,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.on_odom,
            20,
        )
        self.get_logger().info(
            "Publishing synthetic IMU %s from %s"
            % (
                self.get_parameter("imu_topic").value,
                self.get_parameter("odom_topic").value,
            )
        )

    def on_odom(self, msg):
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self.get_parameter("imu_frame").value

        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity = msg.twist.twist.angular

        now = stamp_to_seconds(msg.header.stamp)
        linear = msg.twist.twist.linear
        if self.last_stamp is not None and self.last_linear is not None:
            dt = now - self.last_stamp
            if math.isfinite(dt) and dt > 1.0e-4:
                imu.linear_acceleration.x = (linear.x - self.last_linear[0]) / dt
                imu.linear_acceleration.y = (linear.y - self.last_linear[1]) / dt
                imu.linear_acceleration.z = (linear.z - self.last_linear[2]) / dt

        imu.orientation_covariance[0] = 0.02
        imu.orientation_covariance[4] = 0.02
        imu.orientation_covariance[8] = 0.05
        imu.angular_velocity_covariance[0] = 0.05
        imu.angular_velocity_covariance[4] = 0.05
        imu.angular_velocity_covariance[8] = 0.08
        imu.linear_acceleration_covariance[0] = 0.1
        imu.linear_acceleration_covariance[4] = 0.1
        imu.linear_acceleration_covariance[8] = 0.1

        self.last_stamp = now
        self.last_linear = (linear.x, linear.y, linear.z)
        self.pub.publish(imu)


def main():
    rclpy.init()
    node = OdomToImu()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
