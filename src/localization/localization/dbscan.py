#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import math
from sklearn.cluster import DBSCAN

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PointStamped
from tf_transformations import euler_from_quaternion

class LidarNode(Node):
    def __init__(self):
        super().__init__('Lidar_DBSCAN')

        # 1. Setup TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_points', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("lidar node running...")

    def scan_callback(self, msg):
        try:
            # Look up the transform from laser to map
            trans = self.tf_buffer.lookup_transform(
                'map', 
                msg.header.frame_id,  # laser-frame
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform laser to map: {e}")
            return

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        
        q = trans.transform.rotation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        points_in_map = []
        angle = msg.angle_min
        
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # laser-frame
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                
                # map-frame
                gx = lx * math.cos(yaw) - ly * math.sin(yaw) + tx
                gy = lx * math.sin(yaw) + ly * math.cos(yaw) + ty
                
                points_in_map.append([gx, gy])
            
            angle += msg.angle_increment
            
        if len(points_in_map) < 5:
            return
        
        # Clustering
        points_np = np.array(points_in_map)
        clustering = DBSCAN(eps=0.1, min_samples=5).fit(points_np)
        
        clustered_points = []
        for i, label in enumerate(clustering.labels_):
            if label != -1:
                clustered_points.append([points_np[i][0], points_np[i][1], 0.0])
        
        if not clustered_points:
            return
                
        header = Header()
        header.stamp = msg.header.stamp 
        header.frame_id = 'map'      
        
        cloud_msg = point_cloud2.create_cloud_xyz32(header, clustered_points)
        self.pc_pub.publish(cloud_msg)


def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()  
