#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf_transformations
from visualization_msgs.msg import Marker

import numpy as np
import cv2
from cv_bridge import CvBridge


class ColorDetectionCameraFrame(Node):

    def __init__(self):
        super().__init__('color_detection_camera_frame')
        self.get_logger().info("Cube Detection Started (camera frame)")

        self.bridge = CvBridge()

        # Static transform between camera frame and base_link

        self.static_broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'realsense_camera_color_optical_frame'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(t)

        # Subscribers
        self.create_subscription(
            Image, '/realsense/color/image_raw',
            self.color_callback, 10)

        self.create_subscription(
            Image, '/realsense/depth/image_rect_raw',
            self.depth_callback, 10)

        self.create_subscription(
            CameraInfo, '/realsense/color/camera_info',
            self.camera_info_callback, 10)

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, '/detected_object', 10)

        self.marker_pub = self.create_publisher(
            Marker, '/cube_marker', 10)

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_image = None

        # Parameters
        self.min_contour_area = 300      # Minimum contour area
        self.max_distance = 3.5          # meters

        # HSV color ranges (single broad green range)
        self.color_ranges = {
            'red': [
                ([0, 120, 70], [10, 255, 255]),
                ([170, 120, 70], [180, 255, 255])
            ],
            'green': [
                ([30, 30, 60], [90, 255, 255])  # broad range to detect bright & moderate green
            ],
            'blue': [
                ([90, 80, 50], [130, 255, 255])
            ],
            'skin': [
                ([0, 50, 130], [20, 130, 255])    # light skin cube
            ]
        }

    # -------------------------

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # -------------------------

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    # -------------------------

    def color_callback(self, msg):

        if self.fx is None or self.depth_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Resize depth to match color
        depth_resized = cv2.resize(
            self.depth_image,
            (frame.shape[1], frame.shape[0]),
            interpolation=cv2.INTER_NEAREST
        )

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height = frame.shape[0]

        # Only lower half (floor)
        floor_mask = np.zeros((height, frame.shape[1]), dtype=np.uint8)
        floor_mask[int(height * 0.4):, :] = 255

        mask_windows = {}

        for color_name, ranges in self.color_ranges.items():

            # Combine multiple ranges (for red)
            mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask_total |= cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Apply floor mask
            mask_total = cv2.bitwise_and(mask_total, floor_mask)

            # Morphology for clean mask
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))

            mask_windows[color_name] = mask_total.copy()

            # Find contours
            contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                continue

            # Only largest contour per color
            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) < self.min_contour_area:
                continue

            rect = cv2.minAreaRect(cnt)
            cx_pixel = int(rect[0][0])
            cy_pixel = int(rect[0][1])

            mask_obj = np.zeros_like(mask_total)
            cv2.drawContours(mask_obj, [cnt], -1, 255, -1)

            depth_vals = depth_resized[mask_obj == 255]
            depth_vals = depth_vals[depth_vals > 0]

            if len(depth_vals) == 0:
                continue

            Z = np.median(depth_vals) / 1000.0
            if Z > self.max_distance:
                continue

            X = (cx_pixel - self.cx) * Z / self.fx
            Y = (cy_pixel - self.cy) * Z / self.fy

            # Publish PoseStamped in camera frame
            pose = PoseStamped()
            pose.header.frame_id = "realsense_camera_color_optical_frame"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(X)
            pose.pose.position.y = float(Y)
            pose.pose.position.z = float(Z)
            pose.pose.orientation.w = 1.0

            self.goal_pub.publish(pose)
            self.publish_marker(pose, color_name)

            self.get_logger().info(
                f"{color_name.upper()} DETECTED: X={X:.2f} Y={Y:.2f} Z={Z:.2f}"
            )

            # Draw box and center
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            cv2.drawContours(frame, [box], 0, (255,0,0), 2)
            cv2.circle(frame, (cx_pixel, cy_pixel), 5, (0,0,255), -1)
            cv2.putText(frame, color_name,
                        (cx_pixel-20, cy_pixel-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255,255,255), 2)

        # Show images
        cv2.imshow("Cube Detection", frame)
        for cname, mask in mask_windows.items():
            cv2.imshow(f"{cname} Mask", mask)
        cv2.waitKey(1)

    # -------------------------

    def publish_marker(self, pose, color_name):
        marker = Marker()
        marker.header.frame_id = "realsense_camera_color_optical_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Colors
        if color_name == "red":
            marker.color.r = 1.0
        elif color_name == "green":
            marker.color.g = 1.0
        elif color_name == "blue":
            marker.color.b = 1.0
        elif color_name == "skin":
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.6

        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ColorDetectionCameraFrame()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
