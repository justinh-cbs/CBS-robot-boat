#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from threading import Lock
import math
import time


class RobotOdometryNode(Node):
    def __init__(self):
        super().__init__("robot_odometry")

        self.declare_parameter("use_gps", True)
        self.declare_parameter("use_imu", True)
        self.use_gps = self.get_parameter("use_gps").value
        self.use_imu = self.get_parameter("use_imu").value

        self.initial_lat = None
        self.initial_lon = None
        self.last_gps_time = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.mutex = Lock()

        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.gps_vel_pub = self.create_publisher(
            TwistWithCovarianceStamped, "gps/vel", 10
        )

        # Subscribers
        if self.use_gps:
            self.gps_sub = self.create_subscription(
                NavSatFix, "gps/fix", self.gps_callback, 10
            )

        if self.use_imu:
            self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)

        # timer for regular odometry updates (50 Hz)
        self.create_timer(0.02, self.publish_odometry)

        self.latest_orientation = None
        self.latest_angular_velocity = None
        self.latest_linear_acceleration = None

        self.get_logger().info("Robot odometry node initialized")

    def gps_callback(self, msg):
        """Process GPS data and update position estimate"""
        with self.mutex:
            if self.initial_lat is None:
                self.initial_lat = msg.latitude
                self.initial_lon = msg.longitude
                self.last_gps_time = time.time()
                self.get_logger().info(
                    f"Set initial GPS position: {self.initial_lat}, {self.initial_lon}"
                )
                return

            earth_radius = 6371000.0  # meters
            d_lat = msg.latitude - self.initial_lat
            d_lon = msg.longitude - self.initial_lon
            lat_rad = math.radians(self.initial_lat)

            x = earth_radius * d_lon * math.cos(lat_rad)
            y = earth_radius * d_lat

            current_time = time.time()
            if self.last_gps_time is not None:
                dt = current_time - self.last_gps_time
                if dt > 0:
                    self.velocity_x = (x - self.last_x) / dt
                    self.velocity_y = (y - self.last_y) / dt

                    vel_msg = TwistWithCovarianceStamped()
                    vel_msg.header.stamp = self.get_clock().now().to_msg()
                    vel_msg.header.frame_id = "base_link"
                    vel_msg.twist.twist.linear.x = self.velocity_x
                    vel_msg.twist.twist.linear.y = self.velocity_y

                    covariance = np.zeros(36)
                    covariance[0] = 0.1  # x velocity variance
                    covariance[7] = 0.1  # y velocity variance
                    vel_msg.twist.covariance = covariance.tolist()

                    self.gps_vel_pub.publish(vel_msg)

            self.last_x = x
            self.last_y = y
            self.last_gps_time = current_time

    def imu_callback(self, msg):
        """Store latest IMU data"""
        with self.mutex:
            self.latest_orientation = msg.orientation
            self.latest_angular_velocity = msg.angular_velocity
            self.latest_linear_acceleration = msg.linear_acceleration

    def publish_odometry(self):
        """Publish odometry message combining available sensor data"""
        with self.mutex:
            if self.initial_lat is None:
                return

            current_time = self.get_clock().now().to_msg()

            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.last_x
            t.transform.translation.y = self.last_y
            t.transform.translation.z = 0.0

            if self.latest_orientation is not None:
                t.transform.rotation = self.latest_orientation
            else:
                t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.last_x
            odom.pose.pose.position.y = self.last_y
            odom.pose.pose.position.z = 0.0

            if self.latest_orientation is not None:
                odom.pose.pose.orientation = self.latest_orientation
            else:
                odom.pose.pose.orientation.w = 1.0

            odom.twist.twist.linear.x = self.velocity_x
            odom.twist.twist.linear.y = self.velocity_y
            if self.latest_angular_velocity is not None:
                odom.twist.twist.angular = self.latest_angular_velocity

            pose_covariance = np.zeros(36)
            twist_covariance = np.zeros(36)

            pose_covariance[0] = 0.1  # x
            pose_covariance[7] = 0.1  # y
            pose_covariance[35] = 0.1  # yaw

            twist_covariance[0] = 0.1  # x velocity
            twist_covariance[7] = 0.1  # y velocity
            twist_covariance[35] = 0.1  # yaw velocity

            odom.pose.covariance = pose_covariance.tolist()
            odom.twist.covariance = twist_covariance.tolist()

            self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    node = RobotOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
