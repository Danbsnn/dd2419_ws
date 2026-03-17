#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
import open3d as o3d

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar')

        self.last_scan = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_map', 10)
        self.create_subscription(LaserScan, 
                                '/lidar/scan', 
                                self.scan_callback, 
                                qos_profile_sensor_data)

        self.map_pcd = None
        self.last_icp_pose = None
        self.last_update_pose = None

        # ICP param
        self.icp_distance_threshold = 0.2
        self.voxel_size = 0.05
        self.local_map_radius = 3.0

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
                'map', 
                'lidar_link', #msg.header.frame_id, 
                start_time,
                rclpy.duration.Duration(seconds=0.02)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform laser to map for start: {e}")
            return
        try:
            # laser_frame to map
            tf_end = self.tf_buffer.lookup_transform(
                'map', 
                'lidar_link', # msg.header.frame_id,
                end_time,
                rclpy.duration.Duration(seconds=0.02)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform laser to map for end: {e}")
            return

        x1, y1 = tf_start.transform.translation.x, tf_start.transform.translation.y
        x2, y2 = tf_end.transform.translation.x, tf_end.transform.translation.y
        q = tf_start.transform.rotation
        (_, _, yaw1) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        q = tf_end.transform.rotation
        (_, _, yaw2) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.should_run_icp(x1, y1, yaw1):
            self.publish_map()
            return
        
        if self.rotating_fast(yaw1, yaw2, scan.scan_time):
            self.get_logger().warn("Rotating too fast")
            self.publish_map()
            return

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
        
        # lidar_link
        lx = valid_ranges * np.cos(valid_angles)
        ly = valid_ranges * np.sin(valid_angles)
        # map-frame
        gx = lx * np.cos(yaws) - ly * np.sin(yaws) + pos_x
        gy = lx * np.sin(yaws) + ly * np.cos(yaws) + pos_y

        points_np = np.column_stack((gx, gy, np.zeros(len(gx))))
        
        current_pcd = o3d.geometry.PointCloud()
        current_pcd.points = o3d.utility.Vector3dVector(points_np)
        current_pcd = current_pcd.voxel_down_sample(self.voxel_size)

        if self.map_pcd is None:
            self.get_logger().info("Initializing map with first scan...")
            self.map_pcd = current_pcd
            self.last_icp_pose = (x1, y1, yaw1)
            self.last_update_pose = (x1, y1, yaw1)
            self.publish_map()
            return

        # Create Local map for icp
        map_pts = np.asarray(self.map_pcd.points)
        dx = map_pts[:, 0] - x1
        dy = map_pts[:, 1] - y1
        mask = (dx*dx + dy*dy) < self.local_map_radius**2
        local_map = o3d.geometry.PointCloud()
        local_map.points = o3d.utility.Vector3dVector(map_pts[mask])
        local_map = local_map.voxel_down_sample(self.voxel_size)

        current_pcd.estimate_normals()
        local_map.estimate_normals()

        # Perform ICP using open3d
        icp_result = o3d.pipelines.registration.registration_icp(
            source=current_pcd,
            target=local_map,
            max_correspondence_distance=self.icp_distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        T = icp_result.transformation

        if icp_result.fitness < 0.3:
            self.get_logger().warn(f"icp fitness low: {icp_result.fitness:.2f}")
            self.publish_map()
            return
        
        self.get_logger().info(f"icp fitness: {icp_result.fitness:.2f}")

        current_pcd.transform(T)

        self.map_pcd += current_pcd
        self.map_pcd = self.map_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        map_size = len(np.asarray(self.map_pcd.points))
        self.get_logger().info(f"Map size after voxel filter: {map_size} points")

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
        angular_vel = abs(self.wrap_to_pi(yaw2-yaw1)) / dt
        return angular_vel > 0.4


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