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

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_map', 10)
        self.create_subscription(LaserScan, 
                                '/lidar/scan', 
                                self.scan_callback, 
                                qos_profile_sensor_data)
        self.create_subscription(Pose, '/initial_pose', self.init_pose_callback, 10)

        self.map_pcd = None
        self.last_icp_pose = None
        self.last_update_pose = None

        self.T_map_to_odom = None

        # ICP param
        self.icp_distance_threshold = 0.2
        self.voxel_size = 0.05
        self.local_map_radius = 4

        self.tf_timer = self.create_timer(0.05, self.publish_map_to_odom)

        self.get_logger().info("Lidar node running...")

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

    def scan_callback(self, msg):
        if self.T_map_to_odom is None:
            self.get_logger().info("Waiting for map odom transform...", once=True)
            return

        start_time = rclpy.time.Time.from_msg(msg.header.stamp)

        try:
            tf_start = self.tf_buffer.lookup_transform(
                'map', 
                'lidar_link', #msg.header.frame_id, 
                start_time,
                rclpy.duration.Duration(seconds=0.02)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform lidar to map: {e}")
            return
        
        x, y = tf_start.transform.translation.x, tf_start.transform.translation.y
        q = tf_start.transform.rotation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.should_run_icp(x, y, yaw):
            self.publish_map()
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if len(valid_ranges) < 20:
            return
        
        # lidar_link
        lx = valid_ranges * np.cos(valid_angles)
        ly = valid_ranges * np.sin(valid_angles)
        # odom-frame
        gx = lx * np.cos(yaw) - ly * np.sin(yaw) + x
        gy = lx * np.sin(yaw) + ly * np.cos(yaw) + y

        scan_pts = np.column_stack((gx, gy, np.zeros(len(gx))))

        # Confine scan points to the local map, so we aren't comparing different things
        dx = scan_pts[:, 0] - x
        dy = scan_pts[:, 1] - y
        mask = (dx*dx + dy*dy) < self.local_map_radius**2
        cropped_pts = scan_pts[mask]

        current_pcd = o3d.geometry.PointCloud()
        current_pcd.points = o3d.utility.Vector3dVector(cropped_pts)
        current_pcd = current_pcd.voxel_down_sample(self.voxel_size)

        if self.map_pcd is None:
            self.get_logger().info("Initializing map...")
            self.map_pcd = current_pcd
            self.last_icp_pose = (x, y, yaw)
            self.last_update_pose = (x, y, yaw)
            self.publish_map()
            return
        
        # Create Local map for icp
        map_pts = np.asarray(self.map_pcd.points)
        dx = map_pts[:, 0] - x
        dy = map_pts[:, 1] - y
        mask = (dx*dx + dy*dy) < self.local_map_radius**2
        local_map = o3d.geometry.PointCloud()
        local_map.points = o3d.utility.Vector3dVector(map_pts[mask])
        local_map = local_map.voxel_down_sample(self.voxel_size)

        current_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.5, max_nn=30
    )
)
        local_map.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.5, max_nn=30
    )
)
        
        # Perform ICP using open3d
        icp_result = o3d.pipelines.registration.registration_icp(
            source=current_pcd,
            target=local_map,
            max_correspondence_distance=self.icp_distance_threshold,
            init=np.eye(4),  # Identity since both are in map frame
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        T_icp = icp_result.transformation
       
        self.T_map_to_odom = T_icp @ self.T_map_to_odom

        if icp_result.fitness < 0.3 or icp_result.inlier_rmse > 0.2:
            self.get_logger().warn(f"icp fitness low: {icp_result.fitness:.2f}, resetting map")
            self.map_pcd = None
            return
        
        self.get_logger().info(f"icp fitness: {icp_result.fitness:.2f}")

        current_pcd.transform(T_icp)

        self.map_pcd += current_pcd
        self.map_pcd = self.map_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        map_size = len(np.asarray(self.map_pcd.points))
        self.get_logger().info(f"Map size after voxel filter: {map_size} points")

        self.last_update_pose = (x, y, yaw)

        self.publish_map()


    def publish_map(self):
        if self.map_pcd is None:
            return
        map_np = np.asarray(self.map_pcd.points)
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

        return dist > 0.1 or angle_diff > 0.05

    def wrap_to_pi(self, angle):
        return (angle+np.pi)%(2*np.pi)-np.pi
    
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()  
