#!/usr/bin/env python3

import struct
import ctypes
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from sklearn.cluster import DBSCAN

from tf_transformations import quaternion_from_euler

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


class Detection(Node):

    def __init__(self):

        super().__init__('detection')

        self.get_logger().info(
            "Cube + Box Detection Started"
        )

        # ==================================================
        # TF
        # ==================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ==================================================
        # Publishers
        # ==================================================

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/detected_object',
            10
        )

        self.box_pub = self.create_publisher(
            PoseStamped,
            '/box_detected',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/cube_marker',
            10
        )

        # ==================================================
        # Subscriber
        # ==================================================

        self.create_subscription(
            PointCloud2,
            '/realsense/depth/color/points',
            self.cloud_callback,
            10
        )

        # ==================================================
        # Parameters
        # ==================================================

        self.dbscan_eps = 0.025
        self.dbscan_min_samples = 8

    # ======================================================
    # CALLBACK
    # ======================================================

    def cloud_callback(self, msg):

        gen = pc2.read_points_numpy(
            msg,
            skip_nans=True
        )

        if len(gen) == 0:
            return

        points = gen[:, :3]

        # ==================================================
        # RGB decode
        # ==================================================

        colors_uint32 = np.empty(
            points.shape[0],
            dtype=np.uint32
        )

        for idx, x_ in enumerate(gen):

            c = x_[3]

            s = struct.pack('>f', c)

            i = struct.unpack('>l', s)[0]

            colors_uint32[idx] = (
                ctypes.c_uint32(i).value
            )

        r = (
            ((colors_uint32 >> 16) & 255)
            .astype(np.float32) / 255.0
        )

        g = (
            ((colors_uint32 >> 8) & 255)
            .astype(np.float32) / 255.0
        )

        b = (
            (colors_uint32 & 255)
            .astype(np.float32) / 255.0
        )

        # ==================================================
        # Coordinates
        # ==================================================

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # ==================================================
        # CUBE DETECTION
        # ==================================================

        spatial_mask = (
            (z > 0.20) &
            (z < 0.9) &
            (np.abs(x) < 0.25) &
            (y > -0.02) &
            (y < 0.08)
        )

        indices = np.where(spatial_mask)[0]

        if len(indices) > 0:

            candidates = np.column_stack([
                x[indices],
                y[indices],
                z[indices],
                r[indices],
                g[indices],
                b[indices]
            ]).astype(np.float32)

            pts_xyz = candidates[:, :3]

            clustering = DBSCAN(
                eps=self.dbscan_eps,
                min_samples=self.dbscan_min_samples
            ).fit(pts_xyz)

            labels = clustering.labels_

            unique_labels = set(labels) - {-1}

            for label in unique_labels:

                cluster_mask = (
                    labels == label
                )

                cluster = candidates[
                    cluster_mask
                ]

                if cluster.shape[0] < 8:
                    continue

                self.process_cluster(cluster)

        # ==================================================
        # BOX DETECTION
        # ==================================================

        grey_mask = self._is_grey_hsl_vectorized_from_rgb(
            r,
            g,
            b
        )

        box_mask = (
            grey_mask &
            (y > -0.02) &
            (y < 0.08) &
            (z > 0.20) &
            (z < 0.60) &
            (np.abs(x) < 0.5)
        )

        grey_points_xyz = np.column_stack([
            x[box_mask],
            y[box_mask],
            z[box_mask]
        ]).astype(np.float32)

        if len(grey_points_xyz) > 40:

            clustering = DBSCAN(
                eps=0.04,
                min_samples=20
            ).fit(grey_points_xyz)

            labels = clustering.labels_

            unique_labels = set(labels) - {-1}

            best_cluster = None
            best_size = 0

            for label in unique_labels:

                cluster = grey_points_xyz[
                    labels == label
                ]

                if len(cluster) < 40:
                    continue

                min_xyz = np.min(cluster, axis=0)
                max_xyz = np.max(cluster, axis=0)

                extent = max_xyz - min_xyz

                sx = float(extent[0])
                sy = float(extent[1])
                sz = float(extent[2])

                # ==========================================
                # Reject streaks / floor artifacts
                # ==========================================

                if sx > 0.5 or sy > 0.5:
                    continue

                # ==========================================
                # Reject thin clouds
                # ==========================================

                if sz < 0.04:
                    continue

                # ==========================================
                # Reject sparse noise
                # ==========================================

                density = len(cluster) / (
                    sx * sy * sz + 1e-6
                )

                if density < 15000:
                    continue

                # ==========================================
                # Reject cube-sized objects
                # ==========================================

                if (
                    sx < 0.10 and
                    sy < 0.10
                ):
                    continue

                # ==========================================
                # Expected box dimensions
                # ==========================================

                if not (
                    0.12 < sx < 0.35 and
                    0.08 < sy < 0.30
                ):
                    continue

                cluster_size = len(cluster)

                if cluster_size > best_size:

                    best_size = cluster_size
                    best_cluster = cluster

            if best_cluster is not None:

                grey_points = np.column_stack([
                    best_cluster[:, 2],
                    -best_cluster[:, 0]
                ]).astype(np.float32)

                center, yaw, axes = self.estimate_box_from_points(
                    grey_points,
                    box_size=(0.24, 0.16)
                )

                if center is not None:

                    pose = PoseStamped()

                    pose.header.frame_id = (
                        "realsense_camera_link"
                    )

                    pose.header.stamp = (
                        self.get_clock().now().to_msg()
                    )

                    pose.pose.position.x = float(center[0])
                    pose.pose.position.y = float(center[1])
                    pose.pose.position.z = 0.0

                    q = quaternion_from_euler(
                        0.0,
                        0.0,
                        yaw
                    )

                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]

                    try:

                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            'realsense_camera_link',
                            rclpy.time.Time(),
                            timeout=Duration(seconds=0.1)
                        )

                        pose_map = do_transform_pose_stamped(
                            pose,
                            transform
                        )

                        pose_map.header.frame_id = 'map'

                        self.box_pub.publish(
                            pose_map
                        )

                        self.get_logger().info(
                            f"BOX DETECTED | "
                            f"x={pose_map.pose.position.x:.3f} "
                            f"y={pose_map.pose.position.y:.3f}"
                        )

                    except Exception as e:

                        self.get_logger().warn(
                            f"Box TF failed: {e}"
                        )

                    marker = Marker()

                    marker.header.frame_id = (
                        "realsense_camera_link"
                    )

                    marker.header.stamp = (
                        self.get_clock().now().to_msg()
                    )

                    marker.ns = "box"

                    marker.id = 999

                    marker.type = Marker.CUBE

                    marker.action = Marker.ADD

                    marker.pose = pose.pose

                    marker.scale.x = 0.24
                    marker.scale.y = 0.16
                    marker.scale.z = 0.10

                    marker.color.r = 0.8
                    marker.color.g = 0.8
                    marker.color.b = 0.8
                    marker.color.a = 0.8

                    self.marker_pub.publish(marker)

    # ======================================================
    # PROCESS CUBE
    # ======================================================

    def process_cluster(
        self,
        cluster
    ):

        xyz = cluster[:, :3]

        min_xyz = np.min(xyz, axis=0)
        max_xyz = np.max(xyz, axis=0)

        extent = max_xyz - min_xyz

        sx = float(extent[0])
        sy = float(extent[1])
        sz = float(extent[2])

        if sz < 0.01:
            return

        if sx > 0.08 or sy > 0.08:
            return

        ratio = max(sx, sy) / (
            min(sx, sy) + 1e-6
        )

        if ratio > 2.0:
            return

        if not (
            0.01 < sx < 0.08 and
            0.01 < sy < 0.08 and
            0.01 < sz < 0.08
        ):
            return

        r = cluster[:, 3]
        g = cluster[:, 4]
        b = cluster[:, 5]

        h, s, v = self.rgb_to_hsv_vectorized(
            r,
            g,
            b
        )

        red_cnt = np.sum(
            self.is_red(h, s, v)
        )

        green_cnt = np.sum(
            self.is_green(h, s, v)
        )

        blue_cnt = np.sum(
            self.is_blue(h, s, v)
        )

        total = cluster.shape[0]

        color_counts = {
            'red': red_cnt,
            'green': green_cnt,
            'blue': blue_cnt
        }

        color = max(
            color_counts,
            key=color_counts.get
        )

        confidence = (
            color_counts[color] / total
        )

        if confidence < 0.25:
            return

        center = np.mean(
            xyz,
            axis=0
        )

        cx = float(center[0])
        cy = float(center[1])
        cz = float(center[2])

        self.get_logger().info(
            f"{color.upper()} CUBE DETECTED | "
            f"x={cx:.3f} "
            f"y={cy:.3f} "
            f"z={cz:.3f}"
        )

        pose = PoseStamped()

        pose.header.frame_id = (
            "realsense_camera_color_optical_frame"
        )

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = cx
        pose.pose.position.y = cy
        pose.pose.position.z = cz

        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

    # ======================================================
    # GREY FILTER
    # ======================================================

    def _is_grey_hsl_vectorized_from_rgb(
        self,
        r,
        g,
        b
    ):

        c_max = np.maximum(
            np.maximum(r, g),
            b
        )

        c_min = np.minimum(
            np.minimum(r, g),
            b
        )

        delta = c_max - c_min

        l = (c_max + c_min) / 2.0

        s_hsl = np.zeros_like(r)

        mask_delta = delta != 0

        denominator = (
            1.0 -
            np.abs(
                2.0 * l[mask_delta] - 1.0
            )
        )

        denominator = np.where(
            denominator == 0,
            1e-10,
            denominator
        )

        s_hsl[mask_delta] = (
            delta[mask_delta] /
            denominator
        )

        grey_cond = (
            (s_hsl < 0.2) &
            (l < 0.4)
        )

        return grey_cond

    # ======================================================
    # BOX ESTIMATION
    # ======================================================

    def estimate_box_from_points(
        self,
        points,
        box_size=(0.24, 0.16)
    ):

        if len(points) < 50:
            return None, None, None

        pts = np.asarray(
            points,
            dtype=np.float32
        )

        mean = np.mean(
            pts,
            axis=0
        )

        pts_centered = pts - mean

        _, S, Vt = np.linalg.svd(
            pts_centered,
            full_matrices=False
        )

        axes = Vt[:2]

        yaw = math.atan2(
            axes[0][1],
            axes[0][0]
        )

        center = np.mean(
            pts,
            axis=0
        )

        return center, yaw, axes

    # ======================================================
    # HSV
    # ======================================================

    def rgb_to_hsv_vectorized(
        self,
        r,
        g,
        b
    ):

        c_max = np.maximum(
            np.maximum(r, g),
            b
        )

        c_min = np.minimum(
            np.minimum(r, g),
            b
        )

        delta = c_max - c_min

        h = np.zeros_like(r)

        mask_r = (
            (delta != 0) &
            (c_max == r)
        )

        h[mask_r] = (
            60.0 *
            (((g[mask_r] - b[mask_r]) /
              delta[mask_r]) % 6)
        )

        mask_g = (
            (delta != 0) &
            (c_max == g)
        )

        h[mask_g] = (
            60.0 *
            (((b[mask_g] - r[mask_g]) /
              delta[mask_g]) + 2)
        )

        mask_b = (
            (delta != 0) &
            (c_max == b)
        )

        h[mask_b] = (
            60.0 *
            (((r[mask_b] - g[mask_b]) /
              delta[mask_b]) + 4)
        )

        s = np.zeros_like(r)

        mask = c_max != 0

        s[mask] = (
            delta[mask] / c_max[mask]
        )

        v = c_max

        return h, s, v

    # ======================================================
    # COLORS
    # ======================================================

    def is_red(self, h, s, v):

        return (
            ((h <= 25) | (h >= 335)) &
            (s > 0.4) &
            (v > 0.2)
        )

    def is_green(self, h, s, v):

        return (
            ((h <= 25) | (h >= 335)) &
            (s > 0.4) &
            (v > 0.2)
        )

    def is_blue(self, h, s, v):

        return (
            (h >= 185) &
            (h <= 250) &
            (s > 0.35) &
            (v > 0.15)
        )


# ==========================================================
# MAIN
# ==========================================================

def main():

    rclpy.init()

    node = Detection()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
