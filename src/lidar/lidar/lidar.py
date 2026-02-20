#!/usr/bin/env python

from jupyter_server_terminals import msg

import rcply
import rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2

from tf2_ros import Buffer, TransformListener

from laser_geometry import LaserProjection

from sensor_masgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
import math
from sklearn.cluster import DBSCAN


class LidarNode(Node):
    def __inti__(self):
        super().__init__('Lidar_DBSCAN')

        #publisher: Lidar points in map frame
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/lidar_points',
            10
        )

        # subscriber: raw Lidar scan
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # laser projection
        self.lp = LaserProjection()
        self.get_logger().info("LiDAR node started")

        def scan_callback(self, msg):
            # Convert LaserScan to PointCloud2 in the LiDAR frame
            points = []
            angle = msg.angle_min
            # convert polar coordinates to cartesian
            for r in msg.ranges:
                if np.isinf(r) or np.isnan(r):
                    angle += msg.angle_increment
                    continue

                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
                angle += msg.angle_increment
                
            if len(points) < 5:
                return
            
            points = np.array(points)
            
            #DBSCAN
            clustering = DBSCAN(eps=0.1, min_samples=5).fit(points)
            labels = clustering.labels_
            clustered_points = []
            
            # remove noise
            for i, label in enumerate(labels):
                if label != -1:
                    x, y = points[i]
                    clustered_points.append([x, y, 0.0])
                    
            if not clustered_points:
                return
            
            
            # publish PointCloud2
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id
            
            cloud_msg = point_cloud2.create_cloud_xyz32(
                header, 
                clustered_points
            )

            self.cloud_pub.publish(cloud_msg)


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

            