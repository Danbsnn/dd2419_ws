#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

class Lidar(Node):
    def __init__(self):
        super().__init__('dbscan')

        self.last_scan = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_points', 10)
        self.create_subscription(LaserScan, 
                                '/lidar/scan', 
                                self.scan_callback, 
                                qos_profile_sensor_data)

        self.get_logger().info("lidar node running...")

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
        gx = lx * np.cos(-yaws) - ly * np.sin(-yaws) + pos_x
        gy = lx * np.sin(-yaws) + ly * np.cos(-yaws) + pos_y

        points_np = np.column_stack((gx, gy))
        
        points = [[x, y, 0.0] for x, y in points_np]
        if not points:
            return
                
        header = Header()
        header.stamp = scan.header.stamp 
        header.frame_id = 'map'      
        
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(cloud_msg)


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