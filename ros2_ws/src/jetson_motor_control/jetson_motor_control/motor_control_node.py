#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
import time


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")

        # GPIO Setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Define pins
        self.LEFT_PWM_PIN = 32
        self.LEFT_DIR_PIN = 35
        self.RIGHT_PWM_PIN = 33
        self.RIGHT_DIR_PIN = 36

        # Set up GPIO pins
        GPIO.setup(self.LEFT_PWM_PIN, GPIO.OUT)
        GPIO.setup(self.LEFT_DIR_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_PWM_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_DIR_PIN, GPIO.OUT)

        # Create PWM objects
        self.left_pwm = GPIO.PWM(self.LEFT_PWM_PIN, 1000)
        self.right_pwm = GPIO.PWM(self.RIGHT_PWM_PIN, 1000)

        # Start PWM with 0% duty cycle
        self.left_pwm.start(0)
        self.right_pwm.start(0)

        # Create subscription to cmd_vel
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        self.get_logger().info("Motor control node initialized")

        # Test motors on startup
        self.test_motors()

    def test_motors(self):
        """Run a quick motor test at startup"""
        self.get_logger().info("Testing motors...")

        # Forward pulse
        GPIO.output(self.LEFT_DIR_PIN, True)
        GPIO.output(self.RIGHT_DIR_PIN, True)
        self.left_pwm.ChangeDutyCycle(30)
        self.right_pwm.ChangeDutyCycle(30)
        time.sleep(0.1)

        # Stop
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)

        self.get_logger().info("Motor test complete")

    def cmd_vel_callback(self, msg):
        # Log incoming command
        # self.get_logger().info(f'Received cmd_vel - linear: {msg.linear.x:.2f}, angular: {msg.angular.z:.2f}')

        # Extract linear and angular velocities
        speed_lin = msg.linear.x
        speed_ang = msg.angular.z

        # Convert to left and right motor speeds (-1 to 1)
        w_r = (speed_lin + speed_ang) / 2
        w_l = (speed_lin - speed_ang) / 2

        # Convert to PWM values (0-100)
        left_pwm_value = abs(w_l) * 100.0
        right_pwm_value = abs(w_r) * 100.0

        # Set directions
        GPIO.output(self.LEFT_DIR_PIN, w_l > 0)
        GPIO.output(self.RIGHT_DIR_PIN, w_r > 0)

        # Set PWM duty cycles
        self.left_pwm.ChangeDutyCycle(min(left_pwm_value, 100.0))
        self.right_pwm.ChangeDutyCycle(min(right_pwm_value, 100.0))

        # Log motor commands
        # self.get_logger().info(
        #     f'Motor outputs - Left: PWM={left_pwm_value:.1f}% DIR={w_l > 0}, '
        #     f'Right: PWM={right_pwm_value:.1f}% DIR={w_r > 0}'
        # )

    def cleanup(self):
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")


def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorControlNode()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.cleanup()
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
