#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
import open3d as o3d

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Pose
from nav_msgs.msg import Odometry


class LidarICP(Node):
    def __init__(self):
        super().__init__('lidar_icp')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_map', 10)
        self.create_subscription(LaserScan, 
                                '/lidar/scan', 
                                self.scan_callback, 
                                qos_profile_sensor_data)
        self.create_subscription(Pose, '/initial_pose', self.init_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.last_update_pose = None

        self.T_map_to_odom = None
        self.T_map_to_odom_internal = None
        
        self.odom = None

        self.map = []
        self.map_idx = 0

        # PARAMETERS:
        # Map param
        self.max_scans_in_vicinity = 10
        self.scan_vicinity_radius = 2.5
        self.voxel_size = 0.05  # Downsampling

        # ICP param
        self.icp_distance_threshold = 0.1
        self.local_map_radius = 4.5  # How big area perform icp on and add to map
        self.min_fitness = 0.6
        self.max_inlier_rmse = 0.2

        # How often to run icp
        self.max_angular_speed = 0.0 # Faster and icp won't be performed
        self.max_linear_speed = 0.0
        self.linear_run_icp = 0.3
        self.angular_run_icp = 0.1

        # Publishing map to odom param
        # Lower value equals smoother (although bigger delay)
        self.alpha_xy = 0.15
        self.alpha_yaw = 0.02

        self.tf_timer = self.create_timer(0.1, self.publish_map_to_odom)

        self.get_logger().info("ICP node running...")

    def odom_callback(self, msg):
        self.odom = msg
    
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
        self.T_map_to_odom_internal = self.T_map_to_odom.copy()

        self.get_logger().info("Initial pose set")

    def scan_callback(self, msg):
        if self.T_map_to_odom is None or self.odom is None:
            self.get_logger().info("Waiting for map odom transform...", once=True)
            return
        
        vx = self.odom.twist.twist.linear.x
        vy = self.odom.twist.twist.linear.y
        linear_speed = np.sqrt(vx**2 + vy**2)

        if abs(self.odom.twist.twist.angular.z) > self.max_angular_speed or linear_speed > self.max_linear_speed:
            # self.get_logger().info("Rotating too fast, skipping icp")
            return

        start_time = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            T_map_to_lidar_pred = self.get_internal_lidar_pose(start_time)
            if T_map_to_lidar_pred is None:
                return
        except Exception as e:
            self.get_logger().warn(f"Could not get internal lidar pose: {e}")
            return
        x, y, yaw = self.pose_from_T(T_map_to_lidar_pred)

        if not self.should_run_icp(x, y, yaw):
            self.publish_map()
            return

        current_pcd = self.scan_msg_to_map_pcd(msg, x, y, yaw)
        if current_pcd is None:
            return
            
        if not self.map:
            self.get_logger().info("Initializing map...")
            map_entry = {
                "id": self.map_idx,
                "pc": current_pcd,
                "pose": (x, y, yaw),
            }
            self.map.append(map_entry)
            self.last_update_pose = (x, y, yaw)
            self.map_idx += 1
            self.publish_map()
            return

        map_pc = o3d.geometry.PointCloud()
        for m in self.map:
            map_pc += m["pc"]

        # map_pc = map_pc.voxel_down_sample(self.voxel_size)

        # Create Local map for icp
        map_pts = np.asarray(map_pc.points)
        dx = map_pts[:, 0] - x
        dy = map_pts[:, 1] - y
        mask = (dx*dx + dy*dy) < self.local_map_radius**2
        local_map = o3d.geometry.PointCloud()
        local_map.points = o3d.utility.Vector3dVector(map_pts[mask])
        local_map = local_map.voxel_down_sample(self.voxel_size)

        if len(local_map.points) < 100:
            self.get_logger().info("Skipping ICP due to too few points in local map")
            map_entry = {
                "id": self.map_idx,
                "pc": current_pcd,
                "pose": (x, y, yaw),
            }
            self.map.append(map_entry)
            self.last_update_pose = (x, y, yaw)
            self.map_idx += 1
            self.publish_map()
            return

        # Perform ICP using open3d
        icp_result = o3d.pipelines.registration.registration_icp(
            source=current_pcd,
            target=local_map,
            max_correspondence_distance=self.icp_distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        T_icp = icp_result.transformation

        dx, dy, dyaw = self.pose_from_T(T_icp)
        dtrans = np.hypot(dx, dy)
        
        if icp_result.fitness < self.min_fitness or icp_result.inlier_rmse > self.max_inlier_rmse:
            self.get_logger().warn(f"icp fitness low: {icp_result.fitness:.2f} or inlier rmse high: {icp_result.inlier_rmse:.2f}")
            self.last_update_pose = (x, y, yaw)  # So it does not perform unnecessary icp
            return

        # This is just to avoid huge jumps
        if dtrans > 0.2 or abs(dyaw) > np.deg2rad(4.0):
            self.get_logger().warn("The icp wanted to jump way too far!!!")
            return

        self.get_logger().info("ICP successful")
        self.T_map_to_odom_internal = T_icp @ self.T_map_to_odom_internal

        
        self.get_logger().info(f"icp fitness: {icp_result.fitness:.2f}")

        current_pcd.transform(T_icp)
        T_map_to_lidar_corr = T_icp @ T_map_to_lidar_pred
        x_corr, y_corr, yaw_corr = self.pose_from_T(T_map_to_lidar_corr)

        map_entry = {
                "id": self.map_idx,
                "pc": current_pcd,
                "pose": (x_corr, y_corr, yaw_corr),
            }
        self.map_idx += 1
        self.map.append(map_entry)
        self.prune_old_scans_in_vicinity(x_corr, y_corr)

        self.last_update_pose = (x, y, yaw)

        self.publish_map()

    def publish_map(self):
        if not self.map:
            return
        map_pc = o3d.geometry.PointCloud()
        for m in self.map:
            map_pc += m["pc"]

        map_np = np.asarray(map_pc.points)
        points = map_np.tolist()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'      
        
        msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(msg)
    
    def should_run_icp(self, x, y, yaw):
        if self.last_update_pose is None:
            return True

        last_x, last_y, last_yaw = self.last_update_pose
        dist = np.sqrt((x-last_x)**2 + (y-last_y)**2)
        angle_diff = abs(self.wrap_to_pi(yaw-last_yaw))

        return dist > self.linear_run_icp or angle_diff > self.angular_run_icp

    def wrap_to_pi(self, angle):
        return (angle+np.pi)%(2*np.pi)-np.pi
    
    def publish_map_to_odom(self):
        if self.T_map_to_odom is None:
            return

        x_pub, y_pub, yaw_pub = self.pose_from_T(self.T_map_to_odom)
        x_int, y_int, yaw_int = self.pose_from_T(self.T_map_to_odom_internal)
        
        dx = x_int - x_pub
        dy = y_int - y_pub
        dyaw = self.wrap_to_pi(yaw_int - yaw_pub)

        # clamp
        max_step_xy = 0.01          # 1 cm per timer tick
        max_step_yaw = np.deg2rad(0.2)

        dx = np.clip(dx, -max_step_xy, max_step_xy)
        dy = np.clip(dy, -max_step_xy, max_step_xy)
        dyaw = np.clip(dyaw, -max_step_yaw, max_step_yaw)

        x_new = x_pub + self.alpha_xy * dx
        y_new = y_pub + self.alpha_xy * dy
        yaw_new = yaw_pub + self.alpha_yaw * dyaw

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        self.T_map_to_odom = self.T_from_pose(x_new, y_new, yaw_new)

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

    def pose_from_T(self, T):
        x = float(T[0, 3])
        y = float(T[1, 3])
        yaw = float(np.arctan2(T[1, 0], T[0, 0]))
        return x, y, yaw
    
    def T_from_pose(self, x, y, yaw):
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array([
            [c, -s, 0, x],
            [s,  c, 0, y],
            [0,  0, 1, 0],
            [0,  0, 0, 1],
        ], dtype=float)

    def tf_to_T(self, tf_msg):
        q = tf_msg.transform.rotation
        x = tf_msg.transform.translation.x
        y = tf_msg.transform.translation.y
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return self.T_from_pose(x, y, yaw)

    def get_internal_lidar_pose(self, time):
        past_time = time - Duration(seconds=0.5)

        tf_odom_to_lidar = self.tf_buffer.lookup_transform(
            'odom',
            'lidar_link',
            time,
            rclpy.duration.Duration(seconds=0.1)
        )
        tf_old = self.tf_buffer.lookup_transform(
            'odom',
            'lidar_link',
            past_time,
            rclpy.duration.Duration(seconds=0.1)
        )

        T_odom_to_lidar = self.tf_to_T(tf_odom_to_lidar)
        T_old = self.tf_to_T(tf_old)
        x_now, y_now, yaw_now = self.pose_from_T(T_odom_to_lidar)
        x_old, y_old, yaw_old = self.pose_from_T(T_old)
        dist = np.hypot(x_now - x_old, y_now - y_old)
        dyaw = self.wrap_to_pi(yaw_now - yaw_old)

        max_stationary_dist = 0.01         
        max_stationary_yaw = np.deg2rad(1)
        if dist > max_stationary_dist or abs(dyaw) > max_stationary_yaw:
            return None

        T_map_to_lidar = self.T_map_to_odom_internal @ T_odom_to_lidar
        return T_map_to_lidar

    def scan_msg_to_map_pcd(self, msg, x, y, yaw):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_mask = self.adaptive_scan_radius_mask(msg)

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        lx = valid_ranges * np.cos(valid_angles)
        ly = valid_ranges * np.sin(valid_angles)

        # body_mask = ~(
        #     (lx > -0.25) & (lx < 0.25) &
        #     (ly > -0.15) & (ly < 0.15)
        # )

        # lx = lx[body_mask]
        # ly = ly[body_mask]

        gx = lx * np.cos(yaw) - ly * np.sin(yaw) + x
        gy = lx * np.sin(yaw) + ly * np.cos(yaw) + y

        scan_pts = np.column_stack((gx, gy, np.zeros(len(gx))))

        dx = scan_pts[:, 0] - x
        dy = scan_pts[:, 1] - y
        mask = (dx * dx + dy * dy) < self.local_map_radius ** 2

        cropped = scan_pts[mask]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cropped)

        pcd = pcd.voxel_down_sample(self.voxel_size)
        # pcd, _ = pcd.remove_radius_outlier(
        #     nb_points=2,
        #     radius=0.12
        # )
        # pcd = self.two_band_radius_filter(pcd, sensor_x=x, sensor_y=y)

        if len(np.asarray(pcd.points)) < 20:
            return None

        return pcd

    def adaptive_scan_radius_mask(
            self,
            msg,
            beta=5.0,
            radius_min=0.08,
            radius_max=0.55,
            near_range=1.5,
            min_neighbors=2,
            max_window_beams=12,
            ):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = len(ranges)

        valid = np.isfinite(ranges)
        valid &= ranges > msg.range_min
        valid &= ranges < msg.range_max

        angle_inc = abs(float(msg.angle_increment))
        safe_ranges = np.where(valid, ranges, 0.0).astype(np.float32)

        # Adaptive radius
        radii = beta * angle_inc * safe_ranges  # beta is how many beams to check
        radii = np.clip(radii, radius_min, radius_max).astype(np.float32)
        radii_sq = radii * radii

        # Convert each adaptive physical radius into a beam-index window
        denom = safe_ranges * angle_inc

        windows = np.ones(n, dtype=np.int32)
        usable = valid & (denom > 1e-6)

        windows[usable] = (
            np.ceil(radii[usable] / denom[usable]).astype(np.int32) + 1
        )

        windows = np.minimum(windows, max_window_beams)

        neighbor_counts = np.zeros(n, dtype=np.int32)

        # Compare beams separated by offset k
        for k in range(1, max_window_beams + 1):
            left = slice(0, n - k)
            right = slice(k, n)

            r_i = safe_ranges[left]
            r_j = safe_ranges[right]

            valid_pair = valid[left] & valid[right]

            dtheta = k * angle_inc
            cos_dtheta = np.cos(dtheta)

            # Polar distance between beam i and beam i+k
            dist_sq = (
                r_i * r_i
                + r_j * r_j
                - 2.0 * r_i * r_j * cos_dtheta
            )

            # Count point j as neighbor of point i
            keep_for_left = (
                valid_pair
                & (k <= windows[left])
                & (dist_sq <= radii_sq[left])
            )

            # Count point i as neighbor of point j
            keep_for_right = (
                valid_pair
                & (k <= windows[right])
                & (dist_sq <= radii_sq[right])
            )

            neighbor_counts[left] += keep_for_left.astype(np.int32)
            neighbor_counts[right] += keep_for_right.astype(np.int32)

        keep = valid & (neighbor_counts >= min_neighbors)

        return keep

    def two_band_radius_filter(self, pcd, sensor_x, sensor_y):
        pts = np.asarray(pcd.points)
        if len(pts) == 0:
            return pcd

        dx = pts[:, 0] - sensor_x
        dy = pts[:, 1] - sensor_y
        dists = np.hypot(dx, dy)

        outlier_band_split = 2.5

        near_mask = dists < outlier_band_split
        far_mask = ~near_mask

        near_idx = np.where(near_mask)[0]
        far_idx = np.where(far_mask)[0]

        filtered_parts = []

        if len(near_idx) > 0:
            near_pcd = pcd.select_by_index(near_idx.tolist())
            near_pcd, _ = near_pcd.remove_radius_outlier(
                nb_points=2,
                radius=0.15
            )
            filtered_parts.append(near_pcd)

        if len(far_idx) > 0:
            far_pcd = pcd.select_by_index(far_idx.tolist())
            far_pcd, _ = far_pcd.remove_radius_outlier(
                nb_points=2,
                radius=0.3
            )
            filtered_parts.append(far_pcd)

        if not filtered_parts:
            return o3d.geometry.PointCloud()

        out = o3d.geometry.PointCloud()
        for part in filtered_parts:
            out += part

        return out

    def prune_old_scans_in_vicinity(self, x, y):
        nearby = []

        for i, entry in enumerate(self.map):
            sx, sy, _ = entry["pose"]
            dist = np.hypot(sx - x, sy - y)
            if dist <= self.scan_vicinity_radius:
                nearby.append((i, entry))

        if len(nearby) <= self.max_scans_in_vicinity:
            return

        # Sort by id so the oldest scans come first
        nearby.sort(key=lambda item: item[1]["id"])

        # Remove as many oldest nearby scans as needed, but keep the newest one
        num_to_remove = len(nearby) - self.max_scans_in_vicinity
        removed = 0

        newest_id = max(entry["id"] for _, entry in nearby)

        indices_to_remove = []
        for i, entry in nearby:
            if removed >= num_to_remove:
                break
            if entry["id"] == newest_id:
                continue
            indices_to_remove.append(i)
            removed += 1

        for i in sorted(indices_to_remove, reverse=True):
            removed_entry = self.map.pop(i)
            self.get_logger().info(
                f"Pruned old nearby scan id={removed_entry['id']}"
            )

def main():
    rclpy.init()
    node = LidarICP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()  