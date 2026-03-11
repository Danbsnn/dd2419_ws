#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import open3d as o3d  # <--- ICP Library

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class LidarICP(Node):
    def __init__(self):
        super().__init__('lidar_icp')

        self.last_scan = None
        self.map_pcd = None  # This will store our reference map
        
        # Current "Best Guess" of pose (from ICP)
        self.icp_pose = np.eye(4) 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pc_pub = self.create_publisher(PointCloud2, '/icp_map', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.get_logger().info("Lidar ICP Node started...")

    def scan_callback(self, msg):
        if self.last_scan is None:
            self.last_scan = msg
            return
        
        scan = self.last_scan
        self.last_scan = msg

        # 1. Project Scan to Local Points (Lidar Frame)
        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        mask = (ranges > scan.range_min) & (ranges < scan.range_max)
        
        lx = ranges[mask] * np.cos(angles[mask])
        ly = ranges[mask] * np.sin(angles[mask])
        lz = np.zeros_like(lx)
        local_points = np.column_stack((lx, ly, lz))

        # Create Open3D Point Cloud for current scan
        current_pcd = o3d.geometry.PointCloud()
        current_pcd.points = o3d.utility.Vector3dVector(local_points)

        # 2. Get Initial Guess from Odometry (TF)
        try:
            # We look for the transform from odom to base_link to see 
            # how much the robot moved since the last scan
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            
            # Convert TF to a 4x4 Transformation Matrix
            initial_guess = np.eye(4)
            initial_guess[0, 3] = t.transform.translation.x
            initial_guess[1, 3] = t.transform.translation.y
            # (Simplifying rotation for 2D)
        except Exception:
            initial_guess = self.icp_pose # Fallback to last known ICP pose

        # 3. Handle Map Generation
        if self.map_pcd is None:
            self.get_logger().info("Initializing Map with first scan...")
            self.map_pcd = current_pcd
            return

        # 4. Apply ICP
        # threshold: max distance between points to consider them a match (meters)
        threshold = 0.2 
        reg_p2p = o3d.pipelines.registration.registration_icp(
            current_pcd, self.map_pcd, threshold, initial_guess,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        # This is your new, corrected pose!
        self.icp_pose = reg_p2p.transformation
        
        # Log the refined X and Y
        x, y = self.icp_pose[0, 3], self.icp_pose[1, 3]
        self.get_logger().info(f"Refined Pose: x={x:.3f}, y={y:.3f}")

        # 5. Publish Map (Optional: just for RViz)
        self.publish_map()

    def publish_map(self):
        points = np.asarray(self.map_pcd.points)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(cloud_msg)

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
