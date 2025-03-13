#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
from rclpy.time import Time


class RobotVisualizer(Node):
    def __init__(self):
        super().__init__("robot_visualizer")

        self.path = Path()
        self.path.header.frame_id = "map"

        self.path_pub = self.create_publisher(Path, "gps_path", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, "gps/fix", self.gps_callback, 10
        )

        self.initial_lat = None
        self.initial_lon = None

        self.current_orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        self.publish_static_transforms()

        self.get_logger().info("Robot visualizer node initialized")

    def publish_static_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "base_imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def gps_callback(self, msg):
        if self.initial_lat is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude
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

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.current_orientation

        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose = pose.pose
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.current_orientation
        self.tf_broadcaster.sendTransform(t)

    def imu_callback(self, msg):
        self.current_orientation = msg.orientation


def main(args=None):
    rclpy.init(args=args)
    visualizer = RobotVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
