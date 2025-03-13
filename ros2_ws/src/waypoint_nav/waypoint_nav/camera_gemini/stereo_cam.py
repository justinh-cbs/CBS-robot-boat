#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Lock  # Import Lock for thread safety
import time

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        self.declare_parameters(
            namespace="",
            parameters=[
                ("resolution_width", 1280),
                ("resolution_height", 720),
                ("framerate", 30.0),
                ("flip_method", 0),  # 0 = none, 1 = clockwise, 2 = counter-clockwise
                ("use_arducam_stereo_hat", True), # set to true if using Arducam stereo hat
                ("camera_left_port", 0), # port for left camera
                ("camera_right_port", 1), # port for right camera
            ],
        )

        self.resolution_width = self.get_parameter("resolution_width").value
        self.resolution_height = self.get_parameter("resolution_height").value
        self.framerate = self.get_parameter("framerate").value
        self.flip_method = self.get_parameter("flip_method").value
        self.use_arducam_stereo_hat = self.get_parameter("use_arducam_stereo_hat").value
        self.camera_left_port = self.get_parameter("camera_left_port").value
        self.camera_right_port = self.get_parameter("camera_right_port").value


        self.bridge = CvBridge()
        self.lock = Lock()  # Initialize a lock for thread safety

        # Publishers
        self.left_image_pub = self.create_publisher(Image, 'camera/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, 'camera/right/image_raw', 10)
        self.left_camera_info_pub = self.create_publisher(CameraInfo, 'camera/left/camera_info', 10)
        self.right_camera_info_pub = self.create_publisher(CameraInfo, 'camera/right/camera_info', 10)

        # Initialize camera streams
        if self.use_arducam_stereo_hat:
            self.cap = cv2.VideoCapture(f"arducam_stereo:0,1-{self.resolution_width}x{self.resolution_height}@30") # Use Arducam driver
            if not self.cap.isOpened():
                self.get_logger().error("Cannot open Arducam Stereo Cameras")
                rclpy.shutdown()
                return
        else:
            #left cam
            self.cap_left = cv2.VideoCapture(self.camera_left_port)
            if not self.cap_left.isOpened():
                self.get_logger().error(f"Cannot open Camera {self.camera_left_port}")
                rclpy.shutdown()
                return
            #right cam
            self.cap_right = cv2.VideoCapture(self.camera_right_port)
            if not self.cap_right.isOpened():
                self.get_logger().error(f"Cannot open Camera {self.camera_right_port}")
                rclpy.shutdown()
                return
            
            self.cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_width)
            self.cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_height)
            self.cap_left.set(cv2.CAP_PROP_FPS, self.framerate)

            self.cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_width)
            self.cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_height)
            self.cap_right.set(cv2.CAP_PROP_FPS, self.framerate)
        

        # Timer for capturing and publishing frames
        self.timer = self.create_timer(1.0 / self.framerate, self.capture_and_publish)

        self.get_logger().info(f"Stereo Camera Node initialized with resolution: {self.resolution_width}x{self.resolution_height} and framerate: {self.framerate}fps")

    def capture_and_publish(self):
        """Captures frames from the cameras and publishes them."""
        with self.lock:
            if self.use_arducam_stereo_hat:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("Failed to grab frame from Arducam Stereo Cameras")
                    return

                # Split stereo frame into left and right
                frame_width = frame.shape[1]
                left_frame = frame[:, : frame_width // 2, :]
                right_frame = frame[:, frame_width // 2 :, :]

                #flip
                if self.flip_method == 1:
                    left_frame = cv2.rotate(left_frame, cv2.ROTATE_90_CLOCKWISE)
                    right_frame = cv2.rotate(right_frame, cv2.ROTATE_90_CLOCKWISE)
                elif self.flip_method == 2:
                    left_frame = cv2.rotate(left_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    right_frame = cv2.rotate(right_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

                # Publish frames
                self.publish_frame(left_frame, self.left_image_pub)
                self.publish_frame(right_frame, self.right_image_pub)
                self.publish_camera_info(self.left_camera_info_pub)  # Placeholder for now
                self.publish_camera_info(self.right_camera_info_pub)  # Placeholder for now
            else:
                ret_left, left_frame = self.cap_left.read()
                ret_right, right_frame = self.cap_right.read()
                if not ret_left:
                    self.get_logger().error("Failed to grab frame from left Camera")
                    return
                if not ret_right:
                    self.get_logger().error("Failed to grab frame from right Camera")
                    return
                
                #flip
                if self.flip_method == 1:
                    left_frame = cv2.rotate(left_frame, cv2.ROTATE_90_CLOCKWISE)
                    right_frame = cv2.rotate(right_frame, cv2.ROTATE_90_CLOCKWISE)
                elif self.flip_method == 2:
                    left_frame = cv2.rotate(left_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    right_frame = cv2.rotate(right_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

                # Publish frames
                self.publish_frame(left_frame, self.left_image_pub)
                self.publish_frame(right_frame, self.right_image_pub)
                self.publish_camera_info(self.left_camera_info_pub)  # Placeholder for now
                self.publish_camera_info(self.right_camera_info_pub)  # Placeholder for now


    def publish_frame(self, frame, publisher):
        """Publishes a single frame."""
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")

    def publish_camera_info(self, publisher):
        """Publishes camera information (placeholder for now)."""
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        # TODO: Set correct camera info data here.
        # You'll need to calibrate your cameras and use the resulting
        # camera matrix and distortion coefficients here.
        publisher.publish(camera_info_msg)

def main(args=None):
    rclpy.init(args=args)
    stereo_camera_node = StereoCameraNode()
    try:
        rclpy.spin(stereo_camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(stereo_camera_node, 'cap'):
            stereo_camera_node.cap.release()
        if hasattr(stereo_camera_node, 'cap_left'):
            stereo_camera_node.cap_left.release()
        if hasattr(stereo_camera_node, 'cap_right'):
            stereo_camera_node.cap_right.release()

        stereo_camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
