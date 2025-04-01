#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Joy, MagneticField, Imu
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
from waypoint_nav_interfaces.srv import AddWaypoint, ClearWaypoints
from waypoint_nav_interfaces.msg import WaypointArray
from enum import Enum
import math
import numpy as np


class NavigationState(Enum):
    MANUAL = 1
    AUTONOMOUS = 2


class PIDController:
    def __init__(self, kp, ki, kd, min_out, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True

    def compute(self, error, dt):
        if self.first_run:
            self.previous_error = error
            self.first_run = False
            return self.kp * error

        self.integral += error * dt
        if self.integral * self.ki > self.max_out:
            self.integral = self.max_out / self.ki
        elif self.integral * self.ki < self.min_out:
            self.integral = self.min_out / self.ki

        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        output = max(min(output, self.max_out), self.min_out)

        self.previous_error = error
        return output


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")

        # Parameters - will be overwritten by launch file, modify
        # params there instead of here!
        self.declare_parameters(
            namespace="",
            parameters=[
                ("switch_button", 5),
                ("waypoint_radius", 2.0),
                ("max_linear_speed", 0.5),
                ("max_angular_speed", 0.5),
                ("pid_linear_kp", 0.5),
                ("pid_linear_ki", 0.0),
                ("pid_linear_kd", 0.1),
                ("pid_angular_kp", 1.0),
                ("pid_angular_ki", 0.0),
                ("pid_angular_kd", 0.1),
            ],
        )

        self.state = NavigationState.MANUAL
        self.current_waypoint_index = 0
        self.waypoints = []  # List of (lat, lon) tuples
        self.heading_threshold = 5.0  # degrees
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 0.5  # rad/s

        # init current pos
        self.current_lat = None
        self.current_lon = None

        self.linear_pid = PIDController(
            self.get_parameter("pid_linear_kp").value,
            self.get_parameter("pid_linear_ki").value,
            self.get_parameter("pid_linear_kd").value,
            0.0,  # min speed
            self.get_parameter("max_linear_speed").value,
        )

        self.angular_pid = PIDController(
            self.get_parameter("pid_angular_kp").value,
            self.get_parameter("pid_angular_ki").value,
            self.get_parameter("pid_angular_kd").value,
            -self.get_parameter("max_angular_speed").value,
            self.get_parameter("max_angular_speed").value,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "waypoint_markers", 10)
        self.current_goal_pub = self.create_publisher(Marker, "current_goal", 10)

        self.create_subscription(NavSatFix, "gps/fix", self.gps_callback, 10)
        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, "imu/mag", self.mag_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)

        self.add_waypoint_srv = self.create_service(
            AddWaypoint, "add_waypoint", self.add_waypoint_callback
        )
        self.clear_waypoints_service = self.create_service(
            ClearWaypoints, "clear_waypoints", self.clear_waypoints_callback
        )

        self.last_update = self.get_clock().now()
        self.create_timer(0.1, self.navigation_callback)

        self.create_timer(1.0, self.publish_markers)

        self.get_logger().info("Waypoint navigator initialized")

    def add_waypoint_callback(self, request, response):
        """Service callback to add a new waypoint"""
        self.add_waypoint(request.latitude, request.longitude)
        response.success = True
        response.message = f"Added waypoint: {request.latitude}, {request.longitude}"
        return response

    def clear_waypoints_callback(self, request, response):
        """Service callback to clear all waypoints"""
        self.waypoints = []
        self.current_waypoint_index = 0
        if self.state == NavigationState.AUTONOMOUS:
            self.state = NavigationState.MANUAL
            self.publish_cmd_vel(0.0, 0.0)
        return response

    def add_waypoint(self, lat, lon):
        """Add a new waypoint to the list"""
        self.waypoints.append((lat, lon))
        self.get_logger().info(f"Added waypoint: {lat}, {lon}")
        self.publish_markers()

    def gps_callback(self, msg: NavSatFix):
        """Handle incoming GPS data"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def mag_callback(self, msg):
        """Handle magnetometer updates and calculate bearing"""
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y
        self.magnetic_bearing = math.atan2(mag_y, mag_x) * 180.0 / math.pi
        if self.magnetic_bearing < 0:
            self.magnetic_bearing += 360.0

    def imu_callback(self, msg):
        """Handle IMU updates"""
        self.angular_velocity = msg.angular_velocity.z

    def joy_callback(self, msg: Joy):
        """Handle joystick input"""
        switch_button = self.get_parameter("switch_button").value

        if len(msg.buttons) > switch_button and msg.buttons[switch_button]:
            if self.state == NavigationState.MANUAL:
                if len(self.waypoints) > 0 and self.current_lat is not None:
                    self.state = NavigationState.AUTONOMOUS
                    self.linear_pid.reset()
                    self.angular_pid.reset()
                    self.get_logger().info("Switching to AUTONOMOUS mode")
                else:
                    self.get_logger().warn(
                        "Cannot switch to autonomous: No waypoints or GPS fix"
                    )
            else:
                self.state = NavigationState.MANUAL
                self.get_logger().info("Switching to MANUAL mode")
                self.publish_cmd_vel(0.0, 0.0)

    def navigation_callback(self):
        """Main navigation control loop"""
        if self.state != NavigationState.AUTONOMOUS:
            return

        if (
            self.current_lat is None
            or self.current_lon is None
            or self.magnetic_bearing is None
            or len(self.waypoints) == 0
        ):
            return

        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now

        target_lat, target_lon = self.waypoints[self.current_waypoint_index]

        distance = self.calculate_distance(
            self.current_lat, self.current_lon, target_lat, target_lon
        )

        bearing = self.calculate_bearing(
            self.current_lat, self.current_lon, target_lat, target_lon
        )

        heading_error = self.get_heading_error(bearing)

        self.get_logger().info(
            f"Distance to waypoint: {distance:.2f}m, Heading error: {heading_error:.2f}°, robot mag bearing: {self.magnetic_bearing}"
        )

        # check if we've reached the waypoint
        if distance < self.get_parameter("waypoint_radius").value:
            self.current_waypoint_index += 1
            self.linear_pid.reset()
            self.angular_pid.reset()

            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info("Reached final waypoint")
                self.state = NavigationState.MANUAL
                self.publish_cmd_vel(0.0, 0.0)
                return
            return

        # PID control for linear and angular velocity
        # linear_speed = self.linear_pid.compute(distance, dt)
        # angular_speed = self.angular_pid.compute(bearing, dt)

        # self.publish_cmd_vel(linear_speed, angular_speed)

        cmd = Twist()

        if abs(heading_error) > self.heading_threshold:
            Kp_angular = 0.01
            cmd.angular.z = Kp_angular * heading_error
            cmd.angular.z = max(
                min(cmd.angular.z, self.max_angular_speed), -self.max_angular_speed
            )
            cmd.linear.x = 0.0  # don't move forward while rotating
            # TODO: check this, it may be causing control problems compounded by
            # motor shaft friction

        else:
            # move forward when heading is roughly correct
            cmd.linear.x = self.max_linear_speed * (
                1 - abs(heading_error) / 90.0
            )
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def publish_cmd_vel(self, linear_speed: float, angular_speed: float):
        """Publish velocity commands"""
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(msg)

    def publish_markers(self):
        """Publish visualization markers for RViz"""
        if self.current_lat is None:
            return

        marker_array = MarkerArray()

        for i, (lat, lon) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            x, y = self.latlon_to_local(lat, lon)

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2

            if i < self.current_waypoint_index:
                marker.color.r = 0.5  # gray for visited
                marker.color.g = 0.5
                marker.color.b = 0.5
            elif i == self.current_waypoint_index:
                marker.color.r = 0.0  # green for current
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0  # blue for future
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        # current goal marker
        # TODO: connect trajectory to current goal for display
        if (
            self.state == NavigationState.AUTONOMOUS
            and len(self.waypoints) > self.current_waypoint_index
        ):
            goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
            x, y = self.latlon_to_local(goal_lat, goal_lon)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "current_goal"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 1.0

            marker.scale.x = 1.5
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.current_goal_pub.publish(marker)

    def latlon_to_local(self, lat: float, lon: float) -> tuple:
        """Convert lat/lon to local coordinates relative to current position"""
        if self.current_lat is None:
            return (0.0, 0.0)

        R = 6371000  # Earth's radius in meters as if it wasn't flat
        x = (
            R
            * math.radians(lon - self.current_lon)
            * math.cos(math.radians(self.current_lat))
        )
        y = R * math.radians(lat - self.current_lat)

        return (x, y)

    def get_heading_error(self, target_bearing):
        """Calculate the heading error considering the shortest turn direction"""
        if self.magnetic_bearing is None:
            return 0.0

        error = target_bearing - self.magnetic_bearing

        # normalize to [-180, 180] until they add more degrees to a circle
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        return error

    @staticmethod
    def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate great circle distance between two points in meters"""
        R = 6371000

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat_rad = math.radians(lat2 - lat1)
        dlon_rad = math.radians(lon2 - lon1)

        a = math.sin(dlat_rad / 2) * math.sin(dlat_rad / 2) + math.cos(
            lat1_rad
        ) * math.cos(lat2_rad) * math.sin(dlon_rad / 2) * math.sin(dlon_rad / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    @staticmethod
    def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate bearing between two points in radians"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon_rad = math.radians(lon2 - lon1)

        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
            lat2_rad
        ) * math.cos(dlon_rad)

        return math.atan2(y, x)


def main():
    rclpy.init()
    node = WaypointNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
