#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge


class ColorDetection(Node):

    def __init__(self):
        super().__init__('color_detection')

        print("ColorDetection node started. Waiting for color images...")

        # CvBridge converts ROS Image <-> OpenCV image
        self.bridge = CvBridge()

        # Subscribe to color image topic
        self.create_subscription(
            Image,
            '/realsense/color/image_raw',  # RealSense color stream
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV image (NumPy array)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        height, width, _ = cv_image.shape
        # Pick the center pixel
        center_pixel = cv_image[height // 2, width // 2]  # BGR
        b, g, r = center_pixel

        print(f"Center pixel RGB: R={r}, G={g}, B={b}")


def main():
    rclpy.init()
    node = ColorDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

