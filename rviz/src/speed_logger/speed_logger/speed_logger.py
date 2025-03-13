#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class SpeedLoggerNode(Node):
    def __init__(self):
        super().__init__('speed_logger_node')

        self.declare_parameter('log_interval', 1.0)  # seconds

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10)

        self.speed_pub = self.create_publisher(Float32, 'speed', 10)

        self.last_gps_pos = None
        self.last_gps_time = None
        self.current_cmd_vel = None
        self.current_odom_vel = None
        self.current_gps_speed = None

        self.create_timer(
            self.get_parameter('log_interval').value,
            self.log_data)

        self.get_logger().info('Speed logger initialized')

    def cmd_vel_callback(self, msg):
        """Handle commanded velocity updates"""
        self.current_cmd_vel = msg

    def odom_callback(self, msg):
        """Handle odometry updates"""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_odom_vel = math.sqrt(vx*vx + vy*vy)

        speed_msg = Float32()
        speed_msg.data = self.current_odom_vel
        self.speed_pub.publish(speed_msg)

    def gps_callback(self, msg):
        """Handle GPS updates"""
        current_time = self.get_clock().now()

        if self.last_gps_pos is not None and self.last_gps_time is not None:
            dt = (current_time - self.last_gps_time).nanoseconds / 1e9

            if dt > 0:
                #calculate distance between GPS points
                dist = self.calculate_gps_distance(
                    self.last_gps_pos.latitude, self.last_gps_pos.longitude,
                    msg.latitude, msg.longitude
                )

                self.current_gps_speed = dist / dt

        self.last_gps_pos = msg
        self.last_gps_time = current_time

    def calculate_gps_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between GPS coordinates using Haversine formula"""
        R = 6371000

        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = (math.sin(dlat/2)**2 + math.cos(lat1) * 
             math.cos(lat2) * math.sin(dlon/2)**2)
        c = 2 * math.asin(math.sqrt(a))

        return R * c

    def log_data(self):
        """Log current speed data"""
        cmd_vel_lin = self.current_cmd_vel.linear.x if self.current_cmd_vel else 0.0
        cmd_vel_ang = self.current_cmd_vel.angular.z if self.current_cmd_vel else 0.0
        odom_speed = self.current_odom_vel if self.current_odom_vel is not None else 0.0
        gps_speed = self.current_gps_speed if self.current_gps_speed is not None else 0.0
        self.get_logger().info(
            f'Speeds (m/s) - Command: linear={cmd_vel_lin:.2f}, angular={cmd_vel_ang:.2f}, '
            f'Odometry: {odom_speed:.2f}, GPS: {gps_speed:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SpeedLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
