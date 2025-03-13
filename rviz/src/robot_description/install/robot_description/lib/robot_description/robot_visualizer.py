#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')

        # Initialize path for GPS tracking
        self.path = Path()
        self.path.header.frame_id = 'map'

        # Publishers
        self.path_pub = self.create_publisher(Path, 'gps_path', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store initial GPS position for relative positioning
        self.initial_lat = None
        self.initial_lon = None

        self.get_logger().info('Robot visualizer node initialized')

    def gps_callback(self, msg):
        if self.initial_lat is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude
            return

        # Convert GPS to local coordinates (very simple approximation)
        # For more accuracy, use proper GPS to local coordinate transformation
        earth_radius = 6371000.0  # meters
        d_lat = msg.latitude - self.initial_lat
        d_lon = msg.longitude - self.initial_lon

        x = earth_radius * d_lon * math.cos(math.radians(self.initial_lat))
        y = earth_radius * d_lat

        # Add point to path
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

        # Publish odometry
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose = pose.pose
        self.odom_pub.publish(odom)

        # Publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        self.tf_broadcaster.sendTransform(t)

    def imu_callback(self, msg):
        # IMU data is already in the correct format for RViz
        # The IMU display will use it directly
        pass

def main(args=None):
    rclpy.init(args=args)
    visualizer = RobotVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()