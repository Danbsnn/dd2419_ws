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
        self.create_subscription(Image,
                                 '/realsense/color/image_raw',
                                 self.color_callback, 10)
        self.create_subscription(Image,
                                 '/realsense/depth/image_rect_raw',
                                 self.depth_callback, 10)
        self.create_subscription(CameraInfo,
                                 '/realsense/color/camera_info',
                                 self.camera_info_callback, 10)

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        # Depth image
        self.depth_image = None

        # Target cube size (3 cm)
        self.target_size = 0.03        # meters
        self.size_tolerance = 0.12    # ±1.2 cm tolerance for robustness

        # HSV color ranges
        self.color_ranges = {
            'red': [
                ([0, 100, 100], [10, 255, 255]),
                ([160, 100, 100], [180, 255, 255])
            ],
            'green': [
                ([40, 50, 50], [90, 255, 255])
            ],
            'blue': [
                ([90, 50, 50], [140, 255, 255])
            ]
        }

        # Noise filtering
        self.min_contour_area = 1500
        self.kernel_open = np.ones((5, 5), np.uint8)
        self.kernel_close = np.ones((7, 7), np.uint8)

    # --------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # --------------------------------------------------------
    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    # --------------------------------------------------------
    def color_callback(self, msg: Image):
        if self.fx is None or self.depth_image is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Color conversion failed: {e}")
            return

        # Resize depth to match color frame
        if self.depth_image.shape[:2] != frame.shape[:2]:
            depth_resized = cv2.resize(
                self.depth_image,
                (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )
        else:
            depth_resized = self.depth_image

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_windows = {}

        for color_name, ranges in self.color_ranges.items():
            mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

            # Combine multiple HSV ranges (for red)
            for lower, upper in ranges:
                lower_np = np.array(lower, dtype=np.uint8)
                upper_np = np.array(upper, dtype=np.uint8)
                mask_total |= cv2.inRange(hsv, lower_np, upper_np)

            # Morphology and blur
            mask_total = cv2.morphologyEx(mask_total,
                                          cv2.MORPH_OPEN,
                                          self.kernel_open)
            mask_total = cv2.morphologyEx(mask_total,
                                          cv2.MORPH_CLOSE,
                                          self.kernel_close)
            mask_total = cv2.GaussianBlur(mask_total, (5, 5), 0)

            mask_windows[color_name] = mask_total.copy()

            # Contours
            contours, _ = cv2.findContours(mask_total,
                                           cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < self.min_contour_area:
                    continue

                # Use minAreaRect for accurate width/height
                rect = cv2.minAreaRect(cnt)
                (W_pixel, H_pixel) = rect[1]

                # Mask for depth calculation
                mask_obj = np.zeros_like(mask_total)
                cv2.drawContours(mask_obj, [cnt], -1, 255, -1)
                depth_values = depth_resized[mask_obj == 255]
                depth_values = depth_values[depth_values > 0]
                if len(depth_values) == 0:
                    continue
                Z = np.median(depth_values) / 1000.0
                if Z <= 0:
                    continue

                # Real-world size
                W_real = (W_pixel * Z) / self.fx
                H_real = (H_pixel * Z) / self.fy

                # Robust cube filter: use max dimension
                cube_size = max(W_real, H_real)
                if not (self.target_size - self.size_tolerance < cube_size < self.target_size + self.size_tolerance):
                    continue

                # 3D position
                cx_pixel = int(rect[0][0])
                cy_pixel = int(rect[0][1])
                X = (cx_pixel - self.cx) * Z / self.fx
                Y = (cy_pixel - self.cy) * Z / self.fy

                # Average BGR color
                B, G, R = [int(c) for c in cv2.mean(frame, mask=mask_obj)[:3]]

                self.get_logger().info(
                    f"{color_name.upper()} 3cm Cube → "
                    f"X={X:.3f}m Y={Y:.3f}m Z={Z:.3f}m | "
                    f"Size≈{cube_size:.3f}m | RGB=({R},{G},{B})"
                )

                # Visualization
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
                cv2.circle(frame, (cx_pixel, cy_pixel), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"{color_name} 3cm",
                            (cx_pixel - 20, cy_pixel - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1)

        # Show RGB frame
        cv2.imshow("3cm Cube Detection", frame)

        # Show individual masks
        for cname, mask in mask_windows.items():
            cv2.imshow(f"{cname.capitalize()} Mask", mask)

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

