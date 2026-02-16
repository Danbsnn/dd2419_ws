#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os
from shapely.geometry import Point as ShapePoint, Polygon

class GridPublisher(Node):
    def __init__(self):
        super().__init__('grid_publisher')

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.og_timer = self.create_timer(2.0, self.publish_map)

        workspace_path = "/home/snowwhite/dd2419_ws/src/mapping/map/workspace_1.csv"
        map_path = "/home/snowwhite/dd2419_ws/src/mapping/map/map_1_1.csv"

        self.workspace = np.loadtxt(workspace_path, delimiter=',', skiprows=1)*0.01
        raw_map = np.genfromtxt(map_path, delimiter=',', skip_header=1, dtype=None, encoding='utf-8')
        self.object_types = [row[0] for row in raw_map]
        self.object_coords = np.array([[row[1], row[2]] for row in raw_map]) * 0.01

        self.get_logger().info(str(self.object_coords))

        self.resolution = 0.05  # 5cm cells
        max_x = int(np.max(self.workspace[:, 0]))
        max_y = int(np.max(self.workspace[:, 1]))
        self.width = int(max_x/(self.resolution))
        self.height = int(max_y/(self.resolution))
        self.get_logger().info(f"Initialized {self.width}x{self.height} cells")

        self.static_grid = self.generate_workspace()

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
