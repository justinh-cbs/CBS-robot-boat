"""
IMU fusion node with dynamic magnetometer weighting and GPS preparation
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class IMUFusionNode(Node):
    def __init__(self):
        super().__init__("imu_fusion_node")

        self.angles = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.last_time = None
        self.gyro_bias = np.zeros(3)
        self.calibration_samples = 0
        self.is_calibrating = True

        # Magnetometer variables
        self.mag_bias = np.zeros(3)
        self.mag_scale = np.ones(3)
        self.mag_samples = []
        self.MAG_CAL_SAMPLES = 100
        self.last_mag_values = np.zeros(3)
        self.mag_variance = 0.0

        # dynamic alpha parameters
        self.alpha_min = 0.01  # minimum magnetometer influence (1%)
        self.alpha_max = 0.15  # maximum magnetometer influence (15%)
        self.mag_stability_threshold = 0.05  # Lower = more stable

        # moving average window for magnetometer stability
        self.mag_history = []
        self.MAG_HISTORY_SIZE = 20

        self.imu_publisher = self.create_publisher(Imu, "imu/data", 10)
        self.raw_imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, "imu/mag", self.mag_callback, 10
        )

        # gyro scaling factor
        self.gyro_scale = np.array([0.01, 0.01, 0.01])
        self.tf_broadcaster = TransformBroadcaster(self)

    def calculate_mag_stability(self, mag_data):
        """Calculate magnetometer stability based on recent readings"""
        current_mag = np.array([mag_data.x, mag_data.y, mag_data.z])

        self.mag_history.append(current_mag)
        if len(self.mag_history) > self.MAG_HISTORY_SIZE:
            self.mag_history.pop(0)

        if len(self.mag_history) < 2:
            return 0.0

        magnitudes = [np.linalg.norm(m) for m in self.mag_history]
        variance = np.var(magnitudes)

        mag_delta = np.linalg.norm(current_mag - self.last_mag_values)
        self.last_mag_values = current_mag

        # combine metrics (lower = more stable)
        stability = 1.0 / (1.0 + variance + mag_delta)
        return stability

    def get_dynamic_alpha(self, stability):
        """Calculate dynamic alpha based on magnetometer stability"""
        alpha = self.alpha_min + (self.alpha_max - self.alpha_min) * stability

        alpha = (
            1 / (1 + math.exp(-10 * (alpha - 0.5))) * (self.alpha_max - self.alpha_min)
            + self.alpha_min
        )

        return alpha

    def calibrate_gyro(self, angular_velocity):
        if self.calibration_samples < 100:
            self.gyro_bias += np.array(
                [angular_velocity.x, angular_velocity.y, angular_velocity.z]
            )
            self.calibration_samples += 1
        elif self.is_calibrating:
            self.gyro_bias /= 100
            self.is_calibrating = False

    def calibrate_magnetometer(self, mag_data):
        """Collect magnetometer samples for calibration"""
        if len(self.mag_samples) < self.MAG_CAL_SAMPLES:
            self.mag_samples.append([mag_data.x, mag_data.y, mag_data.z])
            return True

        if len(self.mag_samples) == self.MAG_CAL_SAMPLES:
            mag_data = np.array(self.mag_samples)
            mag_min = np.min(mag_data, axis=0)
            mag_max = np.max(mag_data, axis=0)
            self.mag_bias = (mag_min + mag_max) / 2
            mag_span = (mag_max - mag_min) / 2
            self.mag_scale = np.mean(mag_span) / mag_span
            self.mag_samples.append([0, 0, 0])
        return False

    def euler_to_quaternion(self, roll, pitch, yaw):
        cr, cp, cy = math.cos(roll / 2), math.cos(pitch / 2), math.cos(yaw / 2)
        sr, sp, sy = math.sin(roll / 2), math.sin(pitch / 2), math.sin(yaw / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [w, x, y, z]

    def update_angles(self, angular_velocity, dt):
        # TODO: add in GPS to fusion, calculate heading from GPS when speed is above a threshold
        # Use GPS heading as a third input to the fusion
        # Weight GPS heading based on speed (more weight at higher speeds)
        if self.is_calibrating:
            return

        gyro = np.array(
            [
                angular_velocity.x - self.gyro_bias[0],
                angular_velocity.y - self.gyro_bias[1],
                angular_velocity.z - self.gyro_bias[2],
            ]
        )

        gyro = gyro * self.gyro_scale

        self.angles += gyro * dt
        self.angles = np.array(
            [math.atan2(math.sin(angle), math.cos(angle)) for angle in self.angles]
        )

    def calculate_mag_heading(self, mag_x, mag_y, mag_z, roll, pitch):
        """Calculate yaw angle from magnetometer data with tilt compensation"""
        mx = (mag_x - self.mag_bias[0]) * self.mag_scale[0]
        my = (mag_y - self.mag_bias[1]) * self.mag_scale[1]
        mz = (mag_z - self.mag_bias[2]) * self.mag_scale[2]

        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)

        Xh = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch
        Yh = my * cos_roll - mz * sin_roll

        heading = math.atan2(-Yh, Xh)
        return heading

    def imu_callback(self, msg):
        current_time = self.get_clock().now()

        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
        else:
            dt = 0.0
        self.last_time = current_time

        if self.is_calibrating:
            self.calibrate_gyro(msg.angular_velocity)
            return

        self.update_angles(msg.angular_velocity, dt)

        fused_msg = Imu()
        fused_msg.header = msg.header

        q = self.euler_to_quaternion(self.angles[0], self.angles[1], self.angles[2])
        fused_msg.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])

        fused_msg.angular_velocity = msg.angular_velocity
        fused_msg.linear_acceleration = msg.linear_acceleration

        fused_msg.orientation_covariance = [0.001] * 9
        fused_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        fused_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.imu_publisher.publish(fused_msg)

    def mag_callback(self, msg):
        """Process magnetometer data with dynamic weighting"""
        if self.is_calibrating:
            return

        if self.calibrate_magnetometer(msg.magnetic_field):
            return

        stability = self.calculate_mag_stability(msg.magnetic_field)

        alpha = self.get_dynamic_alpha(stability)

        mag_heading = self.calculate_mag_heading(
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
            self.angles[0],
            self.angles[1],
        )

        gyro_yaw = self.angles[2]

        diff = mag_heading - gyro_yaw
        if diff > math.pi:
            mag_heading -= 2 * math.pi
        elif diff < -math.pi:
            mag_heading += 2 * math.pi

        self.angles[2] = (1 - alpha) * gyro_yaw + alpha * mag_heading

        if stability > self.mag_stability_threshold:
            self.get_logger().debug(
                f"High mag stability: {stability:.3f}, alpha: {alpha:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = IMUFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
