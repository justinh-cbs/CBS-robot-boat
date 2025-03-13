#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, MagneticField, Imu
from geometry_msgs.msg import Twist
from waypoint_nav_interfaces.srv import AddWaypoint, ClearWaypoints
from waypoint_nav_interfaces.msg import WaypointArray
import math
from math import sin, cos, atan2, radians, degrees, pi


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, "gps/fix", self.gps_callback, 10
        )
        self.mag_sub = self.create_subscription(
            MagneticField, "imu/mag", self.mag_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)

        # Services
        self.add_waypoint_service = self.create_service(
            AddWaypoint, "add_waypoint", self.add_waypoint_callback
        )
        self.clear_waypoints_service = self.create_service(
            ClearWaypoints, "clear_waypoints", self.clear_waypoints_callback
        )

        # Current position and heading
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.magnetic_bearing = None
        self.angular_velocity = None

        # Waypoint list
        self.waypoints = []
        self.current_waypoint_index = 0

        # Navigation settings
        self.distance_threshold = 2.0  # meters
        self.heading_threshold = 5.0  # degrees
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 0.5  # rad/s

        # Control loop
        self.create_timer(0.1, self.navigation_loop)  # 10 Hz

        self.get_logger().info("Waypoint navigator initialized")

    def add_waypoint_callback(self, request, response):
        """Handle add waypoint service requests"""
        waypoint = [request.latitude, request.longitude]
        self.waypoints.append(waypoint)
        self.get_logger().info(f"Added waypoint: {waypoint}")
        response.success = True
        return response

    def clear_waypoints_callback(self, request, response):
        """Handle clear waypoints service requests"""
        self.waypoints.clear()
        self.current_waypoint_index = 0
        self.get_logger().info("Cleared all waypoints")
        response.success = True
        return response

    def gps_callback(self, msg):
        """Handle GPS updates"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def mag_callback(self, msg):
        """Handle magnetometer updates"""
        # Calculate bearing from magnetometer data
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y
        self.magnetic_bearing = math.atan2(mag_y, mag_x) * 180.0 / math.pi
        if self.magnetic_bearing < 0:
            self.magnetic_bearing += 360.0

    def imu_callback(self, msg):
        """Handle IMU updates"""
        self.angular_velocity = msg.angular_velocity.z

    def get_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two points"""
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        d_lon = lon2 - lon1

        y = sin(d_lon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(d_lon)
        bearing = atan2(y, x)

        return degrees(bearing)

    def get_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points using Haversine formula"""
        R = 6371000  # Earth's radius in meters
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

        d_lat = lat2 - lat1
        d_lon = lon2 - lon1

        a = sin(d_lat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(d_lon / 2) ** 2
        c = 2 * atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def get_heading_error(self, target_bearing):
        """Calculate the heading error considering the shortest turn direction"""
        if self.magnetic_bearing is None:
            return 0.0

        error = target_bearing - self.magnetic_bearing

        # Normalize to [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        return error

    def navigation_loop(self):
        """Main navigation control loop"""
        if (
            self.current_lat is None
            or self.current_lon is None
            or self.magnetic_bearing is None
            or len(self.waypoints) == 0
        ):
            return

        # Get current waypoint
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            self.stop_robot()
            return

        target = self.waypoints[self.current_waypoint_index]
        target_lat = target[0]
        target_lon = target[1]

        # Calculate distance and bearing to target
        distance = self.get_distance(
            self.current_lat, self.current_lon, target_lat, target_lon
        )
        target_bearing = self.get_bearing(
            self.current_lat, self.current_lon, target_lat, target_lon
        )

        # Calculate heading error
        heading_error = self.get_heading_error(target_bearing)

        # Log navigation info
        self.get_logger().debug(
            f"Distance to waypoint: {distance:.2f}m, Heading error: {heading_error:.2f}°"
        )

        # Check if waypoint reached
        if distance < self.distance_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
            return

        # Create and send movement command
        cmd = Twist()

        # Angular velocity control with feedback from IMU
        if abs(heading_error) > self.heading_threshold:
            # Use proportional control for rotation
            Kp_angular = 0.01
            cmd.angular.z = Kp_angular * heading_error
            cmd.angular.z = max(
                min(cmd.angular.z, self.max_angular_speed), -self.max_angular_speed
            )
            cmd.linear.x = 0.0  # Don't move forward while rotating
        else:
            # Move forward when heading is roughly correct
            cmd.linear.x = self.max_linear_speed * (
                1 - abs(heading_error) / 90.0
            )  # Reduce speed when not perfectly aligned
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.stop_robot()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
