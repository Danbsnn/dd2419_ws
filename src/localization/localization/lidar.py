#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
import open3d as o3d

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
        self.create_subscription(LaserScan, 
                                '/lidar/scan', 
                                self.scan_callback, 
                                qos_profile_sensor_data)

        self.create_subscription(Pose, 
                                '/initial_pose', 
                                self.init_pose_callback,
                                10)

        self.map_pcd = None
        self.last_update_pose = None
        
        # FIX: Initialize to identity (map and odom aligned at start)
        self.T_map_to_odom = np.eye(4)

        # ICP param
        self.icp_distance_threshold = 0.2
        self.voxel_size = 0.05
        self.local_map_radius = 3.0

        # Publish TF continuously
        self.tf_timer = self.create_timer(0.05, self.publish_map_to_odom)

        self.get_logger().info("Lidar node running...")

    def scan_callback(self, msg):
        if self.last_scan is None:
            self.last_scan = msg
            return
        scan = self.last_scan
        self.last_scan = msg

        start_time = rclpy.time.Time.from_msg(scan.header.stamp)
        end_time = start_time + rclpy.duration.Duration(seconds=scan.scan_time)
        
        try:
            tf_start = self.tf_buffer.lookup_transform(
                'odom', 
                'lidar_link', 
                start_time,
                rclpy.duration.Duration(seconds=0.02)
            )
            tf_end = self.tf_buffer.lookup_transform(
                'odom', 
                'lidar_link', 
                end_time,
                rclpy.duration.Duration(seconds=0.02)
            )
        except Exception as e:
            return

        x1, y1 = tf_start.transform.translation.x, tf_start.transform.translation.y
        x2, y2 = tf_end.transform.translation.x, tf_end.transform.translation.y
        q = tf_start.transform.rotation
        (_, _, yaw1) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        q = tf_end.transform.rotation
        (_, _, yaw2) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.should_run_icp(x1, y1, yaw1):
            return
        
        if self.rotating_fast(yaw1, yaw2, scan.scan_time):
            self.get_logger().warn("Rotating too fast, skipping scan")
            return

        # Motion compensation
        yaw_diff = np.unwrap([yaw1, yaw2])
        yaws = np.linspace(yaw_diff[0], yaw_diff[1], len(scan.ranges))
        pos_x = np.linspace(x1, x2, len(scan.ranges))
        pos_y = np.linspace(y1, y2, len(scan.ranges))

        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        valid_mask = (ranges > scan.range_min) & (ranges < scan.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) < 5:
            return
        
        yaws = yaws[valid_mask]
        pos_x = pos_x[valid_mask]
        pos_y = pos_y[valid_mask]
        
        # Convert to odom frame
        lx = valid_ranges * np.cos(valid_angles)
        ly = valid_ranges * np.sin(valid_angles)
        gx = lx * np.cos(yaws) - ly * np.sin(yaws) + pos_x
        gy = lx * np.sin(yaws) + ly * np.cos(yaws) + pos_y

        points = np.column_stack((gx, gy, np.zeros(len(gx))))
        
        # Create point cloud in odom frame
        current_pcd = o3d.geometry.PointCloud()
        current_pcd.points = o3d.utility.Vector3dVector(points)
        current_pcd = current_pcd.voxel_down_sample(self.voxel_size)
        
        # FIX: Initialize map
        if self.map_pcd is None:
            self.get_logger().info("Initializing map with first scan...")
            
            # Transform from odom to map (inverse of T_map_to_odom)
            T_odom_to_map = np.linalg.inv(self.T_map_to_odom)
            current_pcd.transform(T_odom_to_map)
            
            self.map_pcd = current_pcd
            self.last_update_pose = (x1, y1, yaw1)
            self.publish_map()
            return

        # FIX: Transform robot pose to map frame for local map extraction
        robot_in_odom = np.array([x1, y1, 0, 1])
        T_odom_to_map = np.linalg.inv(self.T_map_to_odom)
        robot_in_map = T_odom_to_map @ robot_in_odom

        # Create local map around robot
        map_pts = np.asarray(self.map_pcd.points)
        dx = map_pts[:, 0] - robot_in_map[0]
        dy = map_pts[:, 1] - robot_in_map[1]
        mask = (dx*dx + dy*dy) < self.local_map_radius**2
        
        local_map = o3d.geometry.PointCloud()
        local_map.points = o3d.utility.Vector3dVector(map_pts[mask])
        
        if len(local_map.points) < 10:
            self.get_logger().warn("Local map too small")
            return

        current_pcd.estimate_normals()
        local_map.estimate_normals()

        # FIX: ICP with correct initial guess direction
        # Init guess: transform from source (odom) to target (map)
        icp_result = o3d.pipelines.registration.registration_icp(
            source=current_pcd,      # in odom frame
            target=local_map,        # in map frame
            max_correspondence_distance=self.icp_distance_threshold,
            init=T_odom_to_map,      # odom → map (correct direction!)
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )

        if icp_result.fitness < 0.3:
            self.get_logger().warn(f"ICP fitness low: {icp_result.fitness:.2f}")
            return
        
        self.get_logger().info(f"ICP fitness: {icp_result.fitness:.2f}, RMSE: {icp_result.inlier_rmse:.3f}")

        # FIX: ICP returns T_odom_to_map, we need T_map_to_odom
        T_odom_to_map_corrected = icp_result.transformation
        self.T_map_to_odom = np.linalg.inv(T_odom_to_map_corrected)

        # Transform scan to map and add to global map
        current_pcd.transform(T_odom_to_map_corrected)
        self.map_pcd += current_pcd
        self.map_pcd = self.map_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        self.last_update_pose = (x1, y1, yaw1)
        self.publish_map()

    def publish_map(self):
        if self.map_pcd is None:
            return
        map_np = np.asarray(self.map_pcd.points)
        points = map_np.tolist()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(cloud_msg)
    
    def should_run_icp(self, x, y, yaw):
        if self.last_update_pose is None:
            return True

        last_x, last_y, last_yaw = self.last_update_pose
        dist = np.sqrt((x-last_x)**2 + (y-last_y)**2)
        angle_diff = abs(self.wrap_to_pi(yaw-last_yaw))

        return dist > 0.5 or angle_diff > 0.26

    def wrap_to_pi(self, angle):
        return (angle+np.pi)%(2*np.pi)-np.pi

    def rotating_fast(self, yaw1, yaw2, dt):
        if dt == 0:
            return False
        angular_vel = abs(self.wrap_to_pi(yaw2-yaw1)) / dt
        return angular_vel > 0.4

    def init_pose_callback(self, msg):
        self.get_logger().info("Received initial pose - updating map→odom transform")
    
        x = msg.position.x
        y = msg.position.y
        q = msg.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
        self.T_map_to_odom = np.array([
            [np.cos(yaw), -np.sin(yaw), 0, x],
            [np.sin(yaw),  np.cos(yaw), 0, y],
            [0,            0,           1, 0],
            [0,            0,           0, 1]
        ])

    def publish_map_to_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
    
        T = self.T_map_to_odom
    
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = 0.0
    
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
