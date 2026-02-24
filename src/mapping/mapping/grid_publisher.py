#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import os
from shapely.geometry import Point as ShapePoint, Polygon
from tf_transformations import quaternion_from_euler

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class GridPublisher(Node):
    def __init__(self):
        super().__init__('grid_publisher')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/map_objects', 10)
        
        self.og_timer = self.create_timer(2.0, self.publish_map)

        workspace_path = "/home/snowwhite/dd2419_ws/src/mapping/map/workspace_1.csv"
        map_path = "/home/snowwhite/dd2419_ws/src/mapping/map/map_1_1.csv"

        self.workspace = np.loadtxt(workspace_path, delimiter=',', skiprows=1)*0.01
        raw_map = np.genfromtxt(map_path, delimiter=',', skip_header=1, dtype=None, encoding='utf-8')
        self.object_types = [row[0] for row in raw_map]
        self.object_coords = np.array([[row[1], row[2]] for row in raw_map]) * 0.01
        self.object_angles = [row[3] for row in raw_map]

        self.resolution = 0.05  # 5cm cells
        max_x = int(np.max(self.workspace[:, 0]))
        max_y = int(np.max(self.workspace[:, 1]))
        self.width = int(max_x/(self.resolution))
        self.height = int(max_y/(self.resolution))
        self.get_logger().info(f"Initialized {self.width}x{self.height} cells")

        self.static_grid = self.generate_workspace()

        self.publish_objects()

    def publish_map(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()

        m.info.resolution = self.resolution
        m.info.width = self.width
        m.info.height = self.height
        m.info.origin.position.x = 0.0
        m.info.origin.position.y = 0.0
        m.info.origin.position.z = 0.0

        grid = self.static_grid.copy()
        # mark objects as occupied
        for i, (ox, oy) in enumerate(self.object_coords):
            if self.object_types[i] in ['O', 'B']:
                gx, gy = int(ox/self.resolution), int(oy/self.resolution)
                grid[max(0, gy-1):gy+2, max(0, gx-1):gx+2] = 100

        m.data = grid.flatten().tolist()
        self.map_pub.publish(m)

        ma = MarkerArray()
        
        for i, (ox, oy) in enumerate(self.object_coords):
            obj_type = self.object_types[i]
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = float(ox)
            marker.pose.position.y = float(oy)
            marker.pose.orientation.w = 1.0

            if obj_type == 'O':  # Object = Red cube
                marker.type = Marker.CUBE
                marker.ns = "O"
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.pose.position.z = 0.05
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            elif obj_type == 'B': # Box = Gray cube
                marker.type = Marker.CUBE
                marker.ns = "B"
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.pose.position.z = 0.05
                marker.color.r = 0.8
                marker.color.g = 0.8
                marker.color.b = 0.8
                marker.color.a = 1.0

            elif obj_type == 'S': # Start = blue sphere
                marker.type = Marker.SPHERE
                marker.ns = "S"
                marker.scale.x = 0.15
                marker.scale.y = 0.1
                marker.scale.z = 0.15
                marker.pose.position.z = 0.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.8 

            ma.markers.append(marker)
        
        self.marker_pub.publish(ma)
        
        # Publish the array
        self.marker_pub.publish(ma)

    def publish_objects(self):
        static_transforms = []

        for i, (ox, oy) in enumerate(self.object_coords):
            if self.object_types[i] in ['O', 'B']:
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'map'

                if self.object_types[i] == 'B':
                    t.child_frame_id = f"Box_{i}"
                elif self.object_types[i] == 'O':
                    t.child_frame_id = f"Cube_{i}"
            
                t.transform.translation.x = float(ox)
                t.transform.translation.y = float(oy)
                t.transform.translation.z = 0.0

                q = quaternion_from_euler(0.0, 0.0, self.object_angles[i])
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
        
                static_transforms.append(t)

            self.tf_static_broadcaster.sendTransform(static_transforms)


    def generate_workspace(self):
        self.workspace_poly = Polygon(self.workspace)
        grid = np.full((self.height, self.width), 100, dtype=np.int8)

        for r in range(self.height):
            for c in range(self.width):
                x = c*self.resolution
                y = r*self.resolution
                if self.workspace_poly.contains(ShapePoint(x, y)):
                    grid[r, c] = 0

        return grid



def main():
    rclpy.init()
    node = GridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()