#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
from scipy.spatial import cKDTree

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Pose


class Lidar(Node):
    def __init__(self):
        super().__init__('lidar')

        self.last_scan = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_map', 10)

        self.create_subscription(LaserScan, '/lidar/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Pose, '/initial_pose', self.init_pose_callback, 10)

        self.map_points = None  # Nx2 numpy
        self.last_update_pose = None

        self.T_map_to_odom = None  # 4x4

        # ICP params
        self.icp_max_corr_dist = 0.5
        self.local_map_radius = 3.0

        self.tf_timer = self.create_timer(0.05, self.publish_map_to_odom)

        self.get_logger().info("2D ICP Lidar node running...")

    # ================= ICP ================= #

    def icp_2d(self, source, target, init_T=np.eye(3)):
        T = init_T.copy()
        tree = cKDTree(target)

        prev_error = float('inf')

        for _ in range(20):
            src_h = np.hstack((source, np.ones((len(source), 1))))
            transformed = (T @ src_h.T).T[:, :2]

            dists, indices = tree.query(transformed)

            mask = dists < self.icp_max_corr_dist
            if np.sum(mask) < 10:
                break

            src_corr = transformed[mask]
            tgt_corr = target[indices[mask]]

            R, t = self.best_fit_transform(src_corr, tgt_corr)

            T_step = np.eye(3)
            T_step[:2, :2] = R
            T_step[:2, 2] = t

            T = T_step @ T

            mean_error = np.mean(dists[mask])
            if abs(prev_error - mean_error) < 1e-4:
                break
            prev_error = mean_error

        return T, prev_error

    def best_fit_transform(self, A, B):
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        AA = A - centroid_A
        BB = B - centroid_B

        H = AA.T @ BB
        U, _, Vt = np.linalg.svd(H)

        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_B - R @ centroid_A

        return R, t

    # ================= MAIN ================= #

    def scan_callback(self, msg):
        if self.T_map_to_odom is None:
            return

        if self.last_scan is None:
            self.last_scan = msg
            return

        scan = self.last_scan
        self.last_scan = msg

        # Get pose from TF
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom',
                'lidar_link',
                rclpy.time.Time.from_msg(scan.header.stamp),
                rclpy.duration.Duration(seconds=0.1)
            )
        except Exception:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        q = tf.transform.rotation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.should_run_icp(x, y, yaw):
            return

        # === Convert scan to points (NO motion distortion for stability) ===
        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        mask = (ranges > scan.range_min) & (ranges < scan.range_max)
        ranges = ranges[mask]
        angles = angles[mask]

        if len(ranges) < 20:
            return

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        gx = lx * np.cos(yaw) - ly * np.sin(yaw) + x
        gy = lx * np.sin(yaw) + ly * np.cos(yaw) + y

        source_pts = np.column_stack((gx, gy))[::2]  # downsample

        # === INIT MAP ===
        T_odom_to_map = np.linalg.inv(self.T_map_to_odom)

        if self.map_points is None:
            self.get_logger().info("Initializing map")
            src_h = np.hstack((source_pts, np.ones((len(source_pts), 1))))
            self.map_points = (T_odom_to_map @ src_h.T).T[:, :2]
            self.last_update_pose = (x, y, yaw)
            self.publish_map()
            return

        # === Local map extraction ===
        robot_map = (T_odom_to_map @ np.array([x, y, 0, 1]))[:2]

        dx = self.map_points[:, 0] - robot_map[0]
        dy = self.map_points[:, 1] - robot_map[1]
        mask = (dx*dx + dy*dy) < self.local_map_radius**2

        local_map = self.map_points[mask]

        if len(local_map) < 20:
            return

        # === Transform scan to map frame ===
        src_h = np.hstack((source_pts, np.ones((len(source_pts), 1))))
        source_map = (T_odom_to_map @ src_h.T).T[:, :2]

        # === ICP ===
        T_icp, error = self.icp_2d(source_map, local_map)

        if error > 0.3:
            self.get_logger().warn(f"ICP error too high: {error:.3f}")
            return

        # === Update transform (CORRECT WAY) ===
        T_odom_to_map_2d = np.eye(3)
        T_odom_to_map_2d[:2, :2] = T_odom_to_map[:2, :2]
        T_odom_to_map_2d[:2, 2] = T_odom_to_map[:2, 3]

        T_odom_to_map_2d = T_icp @ T_odom_to_map_2d

        # back to 4x4
        T_odom_to_map = np.eye(4)
        T_odom_to_map[:2, :2] = T_odom_to_map_2d[:2, :2]
        T_odom_to_map[:2, 3] = T_odom_to_map_2d[:2, 2]

        self.T_map_to_odom = np.linalg.inv(T_odom_to_map)

        # === Add to map ===
        source_map = (T_icp @ np.hstack((source_map, np.ones((len(source_map),1)))).T).T[:, :2]

        self.map_points = np.vstack((self.map_points, source_map))[::2]  # keep map small

        self.last_update_pose = (x, y, yaw)
        self.publish_map()

    # ================= UTILS ================= #

    def publish_map(self):
        if self.map_points is None:
            return

        pts3d = np.column_stack((self.map_points, np.zeros(len(self.map_points))))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        msg = point_cloud2.create_cloud_xyz32(header, pts3d.tolist())
        self.pc_pub.publish(msg)

    def should_run_icp(self, x, y, yaw):
        if self.last_update_pose is None:
            return True

        lx, ly, lyaw = self.last_update_pose
        return np.hypot(x-lx, y-ly) > 0.2 or abs(self.wrap_to_pi(yaw-lyaw)) > 0.1

    def wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi

    def init_pose_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        q = msg.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.T_map_to_odom = np.array([
            [np.cos(yaw), -np.sin(yaw), 0, x],
            [np.sin(yaw),  np.cos(yaw), 0, y],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.get_logger().info("Initial pose set")

    def publish_map_to_odom(self):
        if self.T_map_to_odom is None:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        T = self.T_map_to_odom

        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])

        yaw = np.arctan2(T[1, 0], T[0, 0])
        q = quaternion_from_euler(0, 0, yaw)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = Lidar()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
