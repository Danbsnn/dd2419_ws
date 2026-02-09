#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge

class ColorDetection3D(Node):
    def __init__(self):
        super().__init__('color_detection_3d')
        self.get_logger().info("ColorDetection3D node started.")

        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(Image, '/realsense/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/realsense/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/realsense/color/camera_info', self.camera_info_callback, 10)

        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def color_callback(self, msg: Image):
        if self.fx is None or self.depth_image is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Color conversion failed: {e}")
            return

        # HSV mask for red (change ranges for other colors)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        # Clean mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1500:
                continue

            # Bounding box & center
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2

            # Depth
            Z = self.depth_image[cy, cx] / 1000.0  # meters
            X = (cx - self.cx) * Z / self.fx
            Y = (cy - self.cy) * Z / self.fy

            # Get average color inside contour
            mask_obj = np.zeros_like(mask)
            cv2.drawContours(mask_obj, [cnt], -1, 255, -1)
            mean_color = cv2.mean(frame, mask=mask_obj)[:3]  # BGR
            B, G, R = [int(c) for c in mean_color]

            self.get_logger().info(
                f"Cube 3D pos: X={X:.3f} Y={Y:.3f} Z={Z:.3f} | Color: R={R}, G={G}, B={B}"
            )

            # Visualization
            cv2.drawContours(frame, [cnt], -1, (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 5, (0,0,255), -1)
            cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0), 2)
            cv2.putText(frame, f"R={R} G={G} B={B}", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow("Red Cube Detection", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ColorDetection3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
