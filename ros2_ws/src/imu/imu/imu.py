"""
Main ROS2 node for IMU, Magnetometer, and Barometer with magnetometer calibration support
"""

import rclpy
from rclpy.node import Node
import numpy as np
import json
import os
from pathlib import Path

from sensor_msgs.msg import Imu, MagneticField, FluidPressure
from geometry_msgs.msg import Quaternion, Vector3

from imu.lsm6ds33 import LSM6DS33
from imu.lis3mdl import LIS3MDL
from imu.lps25h import LPS25H


class CalibratedImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        self.mag_publisher = self.create_publisher(MagneticField, "imu/mag", 10)

        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        self.frame_id = self.declare_parameter("frame_id", "base_imu_link").value

        self.declare_parameter("use_mag_calibration", True)
        self.declare_parameter("calibration_file", "mag_calibration.json")

        # Initialize sensors
        self.imu = LSM6DS33()
        self.magnetometer = LIS3MDL()
        self.barometer = LPS25H()

        # Load magnetometer calibration
        self.mag_offset = np.zeros(3)
        self.mag_scale = np.ones(3)
        if self.get_parameter("use_mag_calibration").value:
            self.load_magnetometer_calibration()

        self.get_logger().info("Calibrating IMU...")
        self.imu.enable()
        self.get_logger().info("IMU calibration complete.")
        self.magnetometer.enable()

    def load_magnetometer_calibration(self):
        """Load magnetometer calibration from file"""
        calibration_file = self.get_parameter("calibration_file").value
        file_path = Path(Path.home(), "ros2_ws/src/imu/calibration", calibration_file)

        try:
            if os.path.exists(file_path):
                with open(file_path, "r") as f:
                    calibration_data = json.load(f)
                    self.mag_offset = np.array(calibration_data["offset"])
                    self.mag_scale = np.array(calibration_data["scale"])
                    self.get_logger().info("Loaded magnetometer calibration")
            else:
                self.get_logger().warn(f"No calibration file found at {file_path}")
        except Exception as e:
            self.get_logger().error(f"Error loading calibration: {str(e)}")

    def save_magnetometer_calibration(self, offset, scale):
        """Save magnetometer calibration to file"""
        calibration_file = self.get_parameter("calibration_file").value
        file_path = Path(Path.home(), "ros2_ws/src/imu/calibration", calibration_file)

        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        calibration_data = {"offset": offset.tolist(), "scale": scale.tolist()}

        try:
            with open(file_path, "w") as f:
                json.dump(calibration_data, f, indent=4)
            self.get_logger().info("Saved magnetometer calibration")
        except Exception as e:
            self.get_logger().error(f"Error saving calibration: {str(e)}")

    def get_calibrated_mag_reading(self):
        """Get calibrated magnetometer reading"""
        raw_data = np.array(self.magnetometer.get_magnetometer_raw())
        if self.get_parameter("use_mag_calibration").value:
            calibrated_data = (raw_data - self.mag_offset) / self.mag_scale
            return calibrated_data
        return raw_data

    def publish_sensor_data(self):
        """Publish IMU and magnetometer data"""

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        gyro = Vector3()
        gyro_data = self.imu.get_gyro_angular_velocity()
        gyro.x = gyro_data[0]
        gyro.y = gyro_data[1]
        gyro.z = gyro_data[2]

        accel = Vector3()
        accel_data = self.imu.get_accelerometer_g_forces()
        accel.x = accel_data[0]
        accel.y = accel_data[1]
        accel.z = accel_data[2]

        imu_msg.angular_velocity = gyro
        imu_msg.angular_velocity_covariance[0] = 0.00001
        imu_msg.angular_velocity_covariance[4] = 0.00001
        imu_msg.angular_velocity_covariance[8] = 0.00001

        imu_msg.linear_acceleration = accel
        imu_msg.linear_acceleration_covariance[0] = 0.00001
        imu_msg.linear_acceleration_covariance[4] = 0.00001
        imu_msg.linear_acceleration_covariance[8] = 0.00001

        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.frame_id

        mag_data = self.get_calibrated_mag_reading()
        mag_msg.magnetic_field.x = mag_data[0] * 1e-6  # Convert to Tesla
        mag_msg.magnetic_field.y = mag_data[1] * 1e-6
        mag_msg.magnetic_field.z = mag_data[2] * 1e-6

        mag_msg.magnetic_field_covariance[0] = 0.000001
        mag_msg.magnetic_field_covariance[4] = 0.000001
        mag_msg.magnetic_field_covariance[8] = 0.000001

        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalibratedImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
