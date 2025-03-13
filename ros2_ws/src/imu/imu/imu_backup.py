"""
Main ROS2 node for IMU, Magnetometer, and Barometer
"""

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3

from imu.lsm6ds33 import LSM6DS33
from imu.lis3mdl import LIS3MDL
from imu.lps25h import LPS25H


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        # Create publishers
        self.imu_publisher = self.create_publisher(Imu, "imu/data", 10)
        self.mag_publisher = self.create_publisher(MagneticField, "imu/mag", 10)

        # Create timer for publishing data
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

        # Get frame_id parameter
        self.frame_id = self.declare_parameter("frame_id", "base_imu_link").value

        # Initialize sensors
        self.imu = LSM6DS33()
        self.magnetometer = LIS3MDL()
        self.barometer = LPS25H()

        # IMU and magnetometer setup
        self.get_logger().info("Calibrating IMU...")
        self.imu.enable()
        self.get_logger().info("IMU calibration complete.")
        self.magnetometer.enable()

    def get_bearing(self, mag_x, mag_y):
        """Calculate bearing from magnetometer readings"""
        bearing = math.atan2(mag_y, mag_x) * 180.0 / math.pi
        if bearing < 0:
            bearing += 360.0
        return bearing

    def publish_sensor_data(self):
        """
        Publish IMU and Magnetometer data
        """
        current_time = self.get_clock().now().to_msg()

        # Create and publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = self.frame_id

        # Get gyro data
        gyro = Vector3()
        gyro_data = self.imu.get_gyro_angular_velocity()
        gyro.x = gyro_data[0]
        gyro.y = gyro_data[1]
        gyro.z = gyro_data[2]

        # Get accelerometer data
        accel = Vector3()
        accel_data = self.imu.get_accelerometer_g_forces()
        accel.x = accel_data[0]
        accel.y = accel_data[1]
        accel.z = accel_data[2]

        # Set IMU message fields
        imu_msg.angular_velocity = gyro
        imu_msg.linear_acceleration = accel

        # Set covariance matrices (if known, otherwise use defaults)
        imu_msg.angular_velocity_covariance[0] = 0.00001
        imu_msg.angular_velocity_covariance[4] = 0.00001
        imu_msg.angular_velocity_covariance[8] = 0.00001

        imu_msg.linear_acceleration_covariance[0] = 0.00001
        imu_msg.linear_acceleration_covariance[4] = 0.00001
        imu_msg.linear_acceleration_covariance[8] = 0.00001

        # Create and publish magnetometer message
        mag_msg = MagneticField()
        mag_msg.header.stamp = current_time
        mag_msg.header.frame_id = self.frame_id

        # Get magnetometer data
        mag_data = self.magnetometer.get_magnetometer_raw()
        mag_msg.magnetic_field.x = float(mag_data[0])
        mag_msg.magnetic_field.y = float(mag_data[1])
        mag_msg.magnetic_field.z = float(mag_data[2])

        # Calculate bearing
        bearing = self.get_bearing(mag_data[0], mag_data[1])

        # Log data periodically
        self.get_logger().debug(f"Bearing: {bearing:.1f}°")
        self.get_logger().debug(f"Gyro (xyz): {gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f}")

        # Publish messages
        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
