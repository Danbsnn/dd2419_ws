#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import math
from sklearn.cluster import DBSCAN

class LidarNode(Node):
    def __init__(self):
        super().__init__('Lidar_DBSCAN')

        # Publisher for filtered points
        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_points', 10)

        # Subscriber for raw scan
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("LiDAR DBSCAN Filter Node started")

    def scan_callback(self, msg):
        points = []
        angle = msg.angle_min
        
        # 1. Convert Polar to Cartesian
        for r in msg.ranges:
            if not (msg.range_min < r < msg.range_max): # Use the scan's own limits
                angle += msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])
            angle += msg.angle_increment
            
        if len(points) < 5:
            return
        
        points_np = np.array(points)
        
        # 2. DBSCAN: eps=0.1 means points within 10cm are a cluster
        # min_samples=5 means you need 5 points to consider it a real object
        clustering = DBSCAN(eps=0.1, min_samples=5).fit(points_np)
        labels = clustering.labels_
        
        clustered_points = []
        for i, label in enumerate(labels):
            if label != -1: # Filter out noise (-1)
                x, y = points_np[i]
                clustered_points.append([x, y, 0.0]) # Add Z=0 for PointCloud2
                
        if not clustered_points:
            return
        
        # 3. Publish as PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id # Usually 'laser_frame'
        
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
