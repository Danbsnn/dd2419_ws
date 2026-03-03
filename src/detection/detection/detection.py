"""
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
import math


class ColorDetectionNavigation(Node):

    def __init__(self):
        super().__init__('color_detection_navigation')
        self.get_logger().info("ColorDetectionNavigation node started.")

        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            Image, '/realsense/color/image_raw', self.color_callback, 10)
        self.create_subscription(
            Image, '/realsense/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, '/realsense/color/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/detected_object', 10)
        self.marker_pub = self.create_publisher(Marker, '/cube_marker', 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
det
        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_image = None

        # Cube detection params
        self.target_size = 0.03         # 3cm
        self.size_tolerance = 0.02      # ±2cm
        self.min_contour_area = 700     # temporarily lowered
        
        # HSV color ranges
        self.color_ranges = {
            'red': [([0,100,100],[10,255,255]), ([160,100,100],[180,255,255])],
            'green': [([40,50,50],[90,255,255])],
           # 'blue': [([90,50,50],[140,255,255])]
        
        }

        # Navigation offset
        self.stop_distance = 0   # stop 40cm before cube

    # ------------------------------
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # ------------------------------
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # ------------------------------
    def color_callback(self, msg):
        if self.fx is None or self.depth_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        depth_resized = cv2.resize(self.depth_image, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_windows = {}

        for color_name, ranges in self.color_ranges.items():

            mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask_total |= cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
            mask_windows[color_name] = mask_total.copy()

            contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < self.min_contour_area:
                    continue

                rect = cv2.minAreaRect(cnt)
                (W_pixel, H_pixel) = rect[1]

                mask_obj = np.zeros_like(mask_total)
                cv2.drawContours(mask_obj, [cnt], -1, 255, -1)
                depth_vals = depth_resized[mask_obj==255]
                depth_vals = depth_vals[depth_vals>0]

                Z = np.median(depth_vals)/10000.0 if len(depth_vals)>0 else 0.3  # fallback 30cm

                W_real = (W_pixel * Z)/self.fx
                H_real = (H_pixel * Z)/self.fy
                cube_size = max(W_real, H_real)

                # Debug info
                self.get_logger().info(f"{color_name.upper()} cube: Z={Z:.3f} cube_size={cube_size:.3f}")

                # Temporarily comment size filtering
               # if not (self.target_size - self.size_tolerance < cube_size < self.target_size + self.size_tolerance):
                  # continue

                cx_pixel = int(rect[0][0])
                cy_pixel = int(rect[0][1])
                X = (cx_pixel - self.cx) * Z / self.fx
                Y = (cy_pixel - self.cy) * Z / self.fy

                # Draw detection
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(frame, [box], 0, (255,0,0),2)
                cv2.circle(frame, (cx_pixel,cy_pixel), 5, (0,0,255), -1)
                cv2.putText(frame, color_name, (cx_pixel-20, cy_pixel-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)

                # Create PoseStamped
                goal_cam = PoseStamped()
                goal_cam.header.stamp = self.get_clock().now().to_msg()
                goal_cam.header.frame_id = "camera_color_optical_frame"
                goal_cam.pose.position.x = float(X)
                goal_cam.pose.position.y = float(Y)
                goal_cam.pose.position.z = float(Z)
                goal_cam.pose.orientation.w = 1.0

                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map",
                        "camera_color_optical_frame",
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    goal_map_pose = tf2_geometry_msgs.do_transform_pose(goal_cam.pose, transform)
                    goal_map = PoseStamped()
                    goal_map.pose = goal_map_pose
                    goal_map.header.stamp = goal_cam.header.stamp
                    goal_map.header.frame_id = "map"

                    # Stop before cube
                    dx = goal_map.pose.position.x
                    dy = goal_map.pose.position.y
                    dist = math.sqrt(dx*dx + dy*dy)
                    if dist > 0.05:
                        goal_map.pose.position.x -= self.stop_distance * (dx/dist)
                        goal_map.pose.position.y -= self.stop_distance * (dy/dist)

                    # Face cube
                    yaw = math.atan2(dy, dx)
                    goal_map.pose.orientation.z = math.sin(yaw/2.0)
                    goal_map.pose.orientation.w = math.cos(yaw/2.0)

                    # Publish PoseStamped
                    self.goal_pub.publish(goal_map)
                    self.publish_marker(goal_map, color_name)
                    self.get_logger().info(f"{color_name.upper()} goal published to /detected_object")

                except Exception as e:
                    self.get_logger().warn(f"TF failed: {e}")

        # Show detection & masks
        cv2.imshow("Cube Detection", frame)
        for cname, mask in mask_windows.items():

           cv2.imshow(f"{cname} Mask", mask)
        cv2.waitKey(1)

    # ------------------------------
    def publish_marker(self, pose, color_name):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        if color_name=="red": marker.color.r = 1.0
        elif color_name=="green": marker.color.g = 1.0
       # elif color_name=="blue": marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ColorDetectionNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    """
     
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
import math


class ColorDetectionNavigation(Node):

    def __init__(self):
        super().__init__('color_detection_navigation')
        self.get_logger().info("ColorDetectionNavigation node started.")

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

        # Detection parameters
        self.target_size = 0.03
        self.size_tolerance = 0.02
        self.min_contour_area = 700

        # HSV ranges
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
        mask_windows = {}

        for color_name, ranges in self.color_ranges.items():

            mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

            for lower, upper in ranges:
                mask_total |= cv2.inRange(
                    hsv, np.array(lower), np.array(upper))

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

                Z = np.median(depth_vals) / 1000.0  # mm → m

                W_real = (W_pixel * Z) / self.fx
                H_real = (H_pixel * Z) / self.fy
                cube_size = max(W_real, H_real)

                self.get_logger().info(
                    f"{color_name.upper()} cube: "
                    f"Z={Z:.3f} size={cube_size:.3f}")

                cx_pixel = int(rect[0][0])
                cy_pixel = int(rect[0][1])

                X = (cx_pixel - self.cx) * Z / self.fx
                Y = (cy_pixel - self.cy) * Z / self.fy

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
                            0.5, (255,255,255), 1)

                # Create pose in camera frame
                pose_cam = PoseStamped()
                pose_cam.header.stamp = self.get_clock().now().to_msg()
                pose_cam.header.frame_id = "camera_color_optical_frame"
                pose_cam.pose.position.x = float(X)
                pose_cam.pose.position.y = float(Y)
                pose_cam.pose.position.z = float(Z)
                pose_cam.pose.orientation.w = 1.0
                pose_cam.pose.position.z = 0.0
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map",
                        "camera_color_optical_frame",
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )

                    pose_map_pose = tf2_geometry_msgs.do_transform_pose(
                        pose_cam.pose, transform)

                    pose_map = PoseStamped()
                    pose_map.header.frame_id = "map"
                    pose_map.header.stamp = pose_cam.header.stamp
                    pose_map.pose = pose_map_pose

                    self.goal_pub.publish(pose_map)
                    self.publish_marker(pose_map, color_name)

                except Exception as e:
                    self.get_logger().warn(f"TF failed: {e}")

        # Show original frame
        cv2.imshow("Cube Detection", frame)

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
