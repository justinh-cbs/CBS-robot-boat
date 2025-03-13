#!/usr/bin/env python3
"""
ROS2 node for calibrating the LIS3MDL magnetometer.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import json
import time
from pathlib import Path
import os

from imu.lis3mdl import LIS3MDL


class MagnetometerCalibrationNode(Node):
    def __init__(self):
        super().__init__("magnetometer_calibration_node")

        # initialize magnetometer
        self.mag = LIS3MDL()
        self.mag.enable()

        # initialize calibration variables
        self.running_min = np.array([32767, 32767, 32767])
        self.running_max = np.array([-32768, -32768, -32768])
        self.mag_offset = np.zeros(3)
        self.mag_scale = np.ones(3)

        self.declare_parameter("calibration_duration", 30.0)
        self.declare_parameter("calibration_file", "mag_calibration.json")

        self.create_timer(1.0, self.start_calibration_callback)
        self.calibration_started = False
        self.calibration_complete = False

    def start_calibration_callback(self):
        if not self.calibration_started and not self.calibration_complete:
            self.get_logger().info("Starting magnetometer calibration...")
            self.get_logger().info("Rotate the sensor through all orientations")
            self.calibration_started = True
            self.start_time = time.time()
            self.last_update = self.start_time

            # change timer period to higher frequency for data collection
            self.create_timer(0.1, self.calibration_loop)

    def calibration_loop(self):
        if not self.calibration_complete:
            duration = self.get_parameter("calibration_duration").value
            current_time = time.time()

            if (current_time - self.last_update) < duration:
                # read magnetometer values
                raw_data = self.mag.get_magnetometer_raw()
                raw_data = np.array(raw_data)

                # update min/max values
                for i in range(3):
                    if raw_data[i] < self.running_min[i]:
                        self.running_min[i] = raw_data[i]
                        self.last_update = current_time
                    if raw_data[i] > self.running_max[i]:
                        self.running_max[i] = raw_data[i]
                        self.last_update = current_time

                # log progress
                time_remaining = int(duration - (current_time - self.last_update))
                self.get_logger().info(
                    f"Raw: {raw_data}, Time remaining: {time_remaining}s"
                )
            else:
                self.complete_calibration()

    def complete_calibration(self):
        """Calculate calibration values and save them"""
        self.mag_offset = (self.running_max + self.running_min) / 2
        self.mag_scale = (self.running_max - self.running_min) / 2

        self.mag_scale = np.where(self.mag_scale == 0, 1, self.mag_scale)

        self.get_logger().info("\nCalibration Results:")
        self.get_logger().info(f"Min values: {self.running_min}")
        self.get_logger().info(f"Max values: {self.running_max}")
        self.get_logger().info(f"Offsets: {self.mag_offset}")
        self.get_logger().info(f"Scale factors: {self.mag_scale}")

        self.save_calibration()

        self.get_logger().info("\nStarting verification...")
        self.create_timer(0.2, self.verify_calibration)
        self.verification_start = time.time()
        self.calibration_complete = True

    def save_calibration(self):
        """Save calibration data to file"""
        calibration_file = self.get_parameter("calibration_file").value
        file_path = Path(Path.home(), "ros2_ws/src/imu/calibration", calibration_file)

        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        calibration_data = {
            "offset": self.mag_offset.tolist(),
            "scale": self.mag_scale.tolist(),
        }

        try:
            with open(file_path, "w") as f:
                json.dump(calibration_data, f, indent=4)
            self.get_logger().info(f"Saved calibration to {file_path}")
        except Exception as e:
            self.get_logger().error(f"Error saving calibration: {str(e)}")

    def verify_calibration(self):
        """Verify calibration by showing calibrated values"""
        if time.time() - self.verification_start < 10.0:
            raw_data = np.array(self.mag.get_magnetometer_raw())
            calibrated_data = (raw_data - self.mag_offset) / self.mag_scale
            self.get_logger().info(f"Calibrated values: {calibrated_data}")
        else:
            self.get_logger().info("Calibration and verification complete!")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerCalibrationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
