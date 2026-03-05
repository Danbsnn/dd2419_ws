   #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs

import numpy as np
import cv2
from cv_bridge import CvBridge


class ColorDetectionNavigation(Node):

    def __init__(self):
        super().__init__('color_detection_navigation')
        self.get_logger().info("Ground-only Cube Detection Started")

        self.bridge = CvBridge()

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

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_image = None

        # Parameters
        self.min_contour_area = 700
        self.max_distance = 2.0
        self.ground_tolerance = 0.05

        self.color_ranges = {
            'red': [([0,100,100],[10,255,255]),
                    ([160,100,100],[180,255,255])],
            'green': [([40,50,50],[90,255,255])]
        }

    # --------------------------------------------------

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # --------------------------------------------------

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    # --------------------------------------------------

    def color_callback(self, msg):

        if self.fx is None or self.depth_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')

        depth_resized = cv2.resize(
            self.depth_image,
            (frame.shape[1], frame.shape[0]),
            interpolation=cv2.INTER_NEAREST)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        height = frame.shape[0]

        # Only lower half (floor)
        floor_mask = np.zeros((height, frame.shape[1]), dtype=np.uint8)
        floor_mask[int(height*0.4):, :] = 255

        mask_windows = {}

        for color_name, ranges in self.color_ranges.items():

            mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

            for lower, upper in ranges:
                mask_total |= cv2.inRange(
                    hsv, np.array(lower), np.array(upper))

            # Apply floor mask
            mask_total = cv2.bitwise_and(mask_total, floor_mask)

            mask_total = cv2.morphologyEx(
                mask_total, cv2.MORPH_OPEN,
                np.ones((5,5), np.uint8))

            mask_windows[color_name] = mask_total.copy()

            contours, _ = cv2.findContours(
                mask_total, cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:

                if cv2.contourArea(cnt) < self.min_contour_area:
                    continue

                rect = cv2.minAreaRect(cnt)
                (W_pixel, H_pixel) = rect[1]

                mask_obj = np.zeros_like(mask_total)
                cv2.drawContours(mask_obj, [cnt], -1, 255, -1)

                depth_vals = depth_resized[mask_obj == 255]
                depth_vals = depth_vals[depth_vals > 0]

                if len(depth_vals) == 0:
                    continue

                Z = np.median(depth_vals) / 1000.0

                if Z > self.max_distance:
                    continue

                cx_pixel = int(rect[0][0])
                cy_pixel = int(rect[0][1])

                X = (cx_pixel - self.cx) * Z / self.fx
                Y = (cy_pixel - self.cy) * Z / self.fy

                pose_cam = PoseStamped()
                pose_cam.header.stamp = self.get_clock().now().to_msg()
                pose_cam.header.frame_id = "camera_color_optical_frame"
                pose_cam.pose.position.x = float(X)
                pose_cam.pose.position.y = float(Y)
                pose_cam.pose.position.z = float(Z)
                pose_cam.pose.orientation.w = 1.0

                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map",
                        "camera_color_optical_frame",
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )

                    pose_map_pose = tf2_geometry_msgs.do_transform_pose(
                        pose_cam.pose, transform)

                    # Ground filter
                    if abs(pose_map_pose.position.z) > self.ground_tolerance:
                        continue

                    pose_map = PoseStamped()
                    pose_map.header.frame_id = "map"
                    pose_map.header.stamp = pose_cam.header.stamp
                    pose_map.pose = pose_map_pose

                    self.goal_pub.publish(pose_map)
                    self.publish_marker(pose_map, color_name)

                    self.get_logger().info(
                        f"{color_name.upper()} cube: "
                        f"X={pose_map.pose.position.x:.2f}, "
                        f"Y={pose_map.pose.position.y:.2f}")

                    # Draw bounding box
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(frame, [box], 0, (255,0,0), 2)

                    # Draw center
                    cv2.circle(frame, (cx_pixel, cy_pixel),
                               5, (0,0,255), -1)

                    cv2.putText(frame, color_name,
                                (cx_pixel-20, cy_pixel-10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255,255,255), 2)

                except Exception as e:
                    self.get_logger().warn(f"TF failed: {e}")

        # Show original image
        cv2.imshow("Ground Cube Detection", frame)

        # Show masks
        for cname, mask in mask_windows.items():
            cv2.imshow(f"{cname} Mask", mask)

        cv2.waitKey(1)

    # --------------------------------------------------

    def publish_marker(self, pose, color_name):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        if color_name == "red":
            marker.color.r = 1.0
        elif color_name == "green":
            marker.color.g = 1.0

        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ColorDetectionNavigation()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

       
