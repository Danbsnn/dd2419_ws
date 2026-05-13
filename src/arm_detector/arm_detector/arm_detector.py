#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point          # ← ADDED: Point
from cv_bridge import CvBridge
import cv2
import numpy as np

from tf2_ros import Buffer, TransformListener, TransformException

# ─────────────────────────────────────────
# CAMERA INTRINSICS
# ─────────────────────────────────────────
K = np.array([
    [404.23923175620627, 2.188762593807254, 319.5],
    [0.0,               402.195373124321,   239.5],
    [0.0,               0.0,                1.0]
], dtype=np.float64)

D = np.array([
    -0.4458703412524673,
     2.7816594156519177,
    -2.6362887625271108,
    -0.7973909844908826
], dtype=np.float64)

_map1, _map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), K, (640, 480), cv2.CV_16SC2
)

# ─────────────────────────────────────────
# COLOR SETTINGS
# ─────────────────────────────────────────
COLOR_HSV_RANGES = {
    "green": [
        (np.array([35, 70, 70], np.uint8), np.array([90, 255, 255], np.uint8)),
    ],
    "blue": [
        (np.array([90, 70, 70], np.uint8), np.array([130, 255, 255], np.uint8)),
    ],
}

_MARKER_RGBA = {
    "green": ColorRGBA(r=0.0, g=0.9, b=0.0, a=0.85),
    "blue":  ColorRGBA(r=0.0, g=0.4, b=1.0, a=0.85),
}

_MARKER_ID = {
    "green": (2, 3),
    "blue":  (4, 5),
}

CUBE_SIDE = 0.03

# ─────────────────────────────────────────
# ORIENTATION FUNCTION
# ─────────────────────────────────────────
def get_cube_yaw(cnt):
    rect = cv2.minAreaRect(cnt)
    angle = rect[2]

    w, h = rect[1]
    if w < h:
        angle = angle + 90

    return np.deg2rad(angle)

# ─────────────────────────────────────────
# NODE
# ─────────────────────────────────────────
class CubeDetector(Node):

    def __init__(self):
        super().__init__('cube_detector')

        self.bridge = CvBridge()
        self.debug = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_offset_x = 0.0
        self.base_offset_y = 0.0
        self.camera_height = 0.2

        self.pub = self.create_publisher(MarkerArray, 'cubes', 10)

        # ── ADDED: publisher for raw pixel centers ──────────────────────────
        self.center_raw_pub = self.create_publisher(Point, 'center_raw', 10)
        # ───────────────────────────────────────────────────────────────────

        self.sub = self.create_subscription(
            Image, '/arm/camera/image_raw', self.image_callback, 10
        )

        self.create_timer(0.01, self.update_camera_offsets)
        self.create_timer(0.1, self.publish_markers)

        self.latest = {}

    def update_camera_offsets(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.005)
            )

            now = self.get_clock().now()
            tf_time = rclpy.time.Time.from_msg(t.header.stamp)
            age = (now - tf_time).nanoseconds * 1e-9

            if age > 0.03:
                return

            self.base_offset_x = t.transform.translation.x
            self.base_offset_y = t.transform.translation.y
            self.camera_height = t.transform.translation.z

        except TransformException:
            return

    def pixel_to_camera_xy(self, u, v):
        pt = np.array([[[u, v]]], dtype=np.float32)
        undist = cv2.fisheye.undistortPoints(pt, K, D, P=K)
        u2, v2 = undist[0, 0]

        X_cam = (u2 - K[0, 2]) / K[0, 0] * self.camera_height
        Y_cam = (v2 - K[1, 2]) / K[1, 1] * self.camera_height

        return X_cam, Y_cam

    def cam_to_base(self, X_cam, Y_cam):
        return (
            self.base_offset_x - Y_cam,
            self.base_offset_y - X_cam,
            0.0
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.remap(frame, _map1, _map2, cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        vis = frame.copy() if self.debug else None
        h, w = frame.shape[:2]

        detections = {}

        if self.debug:
            cv2.drawMarker(vis, (w // 2, h // 2),
                           (200, 200, 200), cv2.MARKER_CROSS, 20, 1)

        edges = cv2.Canny(frame, 80, 160)

        for color, ranges in COLOR_HSV_RANGES.items():
            mask = np.zeros((h, w), dtype=np.uint8)

            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, lo, hi)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8), 2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8), 2)

            if self.debug:
                try:
                    cv2.imshow(f"{color}_mask", mask)
                except:
                    pass

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not cnts:
                continue

            cnt = max(cnts, key=cv2.contourArea)

            if cv2.contourArea(cnt) < 100:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)

            edge_mask = cv2.bitwise_and(edges, mask)

            combined = np.zeros_like(mask)
            cv2.drawContours(combined, [cnt], -1, 255, -1)
            combined = cv2.bitwise_or(combined, edge_mask)

            M = cv2.moments(combined, binaryImage=True)

            if M["m00"] == 0:
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

            u = int(M["m10"] / M["m00"])
            v = int(M["m01"] / M["m00"])

            X_cam, Y_cam = self.pixel_to_camera_xy(u, v)
            x_b, y_b, z_b = self.cam_to_base(X_cam, Y_cam)

            yaw = get_cube_yaw(cnt)

            detections[color] = (x_b, y_b, z_b, yaw)

            # ── ADDED: publish raw pixel center for this detection ──────────
            center_msg = Point()
            center_msg.x = float(u)
            center_msg.y = float(v)
            center_msg.z = 0.0
            self.center_raw_pub.publish(center_msg)
            # ─────────────────────────────────────────────────────────────────

            if self.debug:
                color_bgr = {
                    "red": (0, 0, 255),
                    "green": (0, 255, 0),
                    "blue": (255, 0, 0)
                }[color]

                cv2.rectangle(vis, (x, y), (x + bw, y + bh), color_bgr, 2)
                cv2.circle(vis, (u, v), 5, (255, 255, 255), -1)
                cv2.line(vis, (w//2, h//2), (u, v), color_bgr, 1)

                cv2.putText(
                    vis,
                    f"{color} ({x_b:+.3f}, {y_b:+.3f})",
                    (x, max(y - 10, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color_bgr,
                    2
                )

        self.latest = detections

        if self.debug:
            try:
                cv2.imshow("Detection", vis)
                cv2.waitKey(1)
            except:
                pass

    def publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for color in COLOR_HSV_RANGES:
            if color in self.latest:
                x, y, z, yaw = self.latest[color]

                m = Marker()
                m.header.frame_id = "base_link"
                m.header.stamp = now
                m.ns = "cubes"
                m.id = _MARKER_ID[color][0]
                m.type = Marker.CUBE
                m.action = Marker.ADD

                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = 0.0

                cy = np.cos(yaw * 0.5)
                sy = np.sin(yaw * 0.5)

                m.pose.orientation.x = 0.0
                m.pose.orientation.y = 0.0
                m.pose.orientation.z = sy
                m.pose.orientation.w = cy

                m.scale = Vector3(x=CUBE_SIDE, y=CUBE_SIDE, z=CUBE_SIDE)
                m.color = _MARKER_RGBA[color]
                m.lifetime = Duration(seconds=0.5).to_msg()

                ma.markers.append(m)

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()