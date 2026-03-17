#!/usr/bin/env python

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import os
from shapely.geometry import Point as ShapePoint, Polygon
from tf_transformations import quaternion_from_euler

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped


class GridPublisher(Node):
    def __init__(self):
        super().__init__('grid_publisher')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publishers
        self.map_pub = self.create_publisher(
                                OccupancyGrid, 
                                '/map', 
                                10)
        
        self.marker_pub = self.create_publisher(
                                MarkerArray, 
                                '/map_objects', 
                                10)

        # Subscribers
        self.detection_sub = self.create_subscription(
                                PoseStamped, 
                                '/detected_object',
                                self.detection_callback,
                                10
                            )
        
        self.box_sub = self.create_subscription(
                                PoseStamped,
                                '/box_detected',
                                self.box_callback,
                                10
                            )
        
        self.pose_sub = self.create_subscription(
                                PoseStamped,
                                '/localized_pose',
                                self.pose_callback,
                                10
                            )
        
        # Initialize variables
        self.robot_pose = None
        self.detection_range = 0.5
        self.duplicate_threshold = 0.20
        
        # Timer for publishing the map
        self.og_timer = self.create_timer(
                                0.2,
                                self.publish_map)

        # Load workspace and map
        workspace_path = "/home/snowwhite/dd2419_ws/src/mapping/map/workspace_1.csv"
        map_path = "/home/snowwhite/dd2419_ws/src/mapping/map/map_1_1.csv"

        self.workspace = np.loadtxt(workspace_path, delimiter=',', skiprows=1)*0.01
        raw_map = np.genfromtxt(map_path, delimiter=',', skip_header=1, dtype=None, encoding='utf-8')
        self.object_types = [row[0] for row in raw_map]
        self.object_coords = [[row[1]*0.01, row[2]*0.01, row[3]] for row in raw_map]

        self.resolution = 0.05  # 5cm cells
        max_x = int(np.max(self.workspace[:, 0]))
        max_y = int(np.max(self.workspace[:, 1]))
        self.width = int(max_x/(self.resolution))
        self.height = int(max_y/(self.resolution))
        self.get_logger().info(f"Initialized {self.width}x{self.height} cells")

        # Generate the static grid based on the workspace once at the start
        self.og_grid = self.generate_workspace()  # static cells
        self.dynamic_grid = self.og_grid.copy()   # updated cells
        self.publish_objects()

    # Subscribers callbacks
    def detection_callback(self, msg: PoseStamped):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(f"Detected object is in '{msg.header.frame_id}' frame. Please change to 'map'!")
            return
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        is_duplicate = False
        for i, (ox, oy, _) in enumerate(self.object_coords):
            distance = math.hypot(new_x - ox, new_y - oy)
            if distance < self.duplicate_threshold:
                is_duplicate = True
        if not is_duplicate:
            self.get_logger().info(f"New object discovered at ({new_x:.2f}, {new_y:.2f})")
            self.object_coords.append([new_x, new_y, 0])
            self.object_types.append('O')
            self.publish_objects()
                

    def box_callback(self, msg: PoseStamped):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(f"Detected box is in '{msg.header.frame_id}' frame. Please change to 'map'!")
            return
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        is_duplicate = False
        for i, (ox, oy, _) in enumerate(self.object_coords):
            distance = math.hypot(new_x - ox, new_y - oy)
            if distance < self.duplicate_threshold:
                is_duplicate = True
        if not is_duplicate:
            self.get_logger().info(f"New box discovered at ({new_x:.2f}, {new_y:.2f})")
            self.object_coords.append([new_x, new_y, 0])
            self.object_types.append('B')
            self.publish_objects()

    def pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    # Timer callback
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

        grid = self.dynamic_grid    

        # mark objects as occupied
        for i, (ox, oy, _) in enumerate(self.object_coords):
            if self.object_types[i] in ['O', 'B']:
                gx, gy = int(ox/self.resolution), int(oy/self.resolution)
                grid[max(0, gy-1):gy+2, max(0, gx-1):gx+2] = 100

        grid = self.update_visibility(grid)

        # robot place as free cells 20cm x 35cm
        if self.robot_pose is not None:
            rx = self.robot_pose.pose.position.x
            ry = self.robot_pose.pose.position.y
            half_w = 0.1  # 20cm /2
            half_l = 0.175 # 35cm /2
            min_x = int(max(0, (rx-half_w)/self.resolution))
            max_x = int(min(self.width, (rx+half_w)/self.resolution))
            min_y = int(max(0, (ry-half_l)/self.resolution))
            max_y = int(min(self.height, (ry+half_l)/self.resolution))
            grid[min_y:max_y, min_x:max_x] = 0

        self.dynamic_grid = grid.copy()
        
        m.data = grid.flatten().tolist()
        self.map_pub.publish(m)

        ma = MarkerArray()
        
        for i, (ox, oy, _) in enumerate(self.object_coords):
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
        
        # Publish the array
        self.marker_pub.publish(ma)  

    # Functions
    def publish_objects(self):
        static_transforms = []

        for i, (ox, oy, angle) in enumerate(self.object_coords):
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

                q = quaternion_from_euler(0.0, 0.0, angle)
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
                    grid[r, c] = -1 # unknow cell

        return grid

    def update_visibility(self, grid):

        if self.robot_pose is None:
            return grid

        rx = self.robot_pose.pose.position.x
        ry = self.robot_pose.pose.position.y

        orientation = self.robot_pose.pose.orientation
        yaw = 2 * math.atan2(orientation.z, orientation.w)

        near = 0.2   
        far = 0.6   
        fov = math.radians(80)

        # trapeze visibility infront of robot
        min_x = int(max(0, (rx-far)/self.resolution))
        max_x = int(min(self.width, (rx+far)/self.resolution))
        min_y = int(max(0, (ry-near)/self.resolution))
        max_y = int(min(self.height, (ry+far)/self.resolution))

        for r in range(min_y, max_y):
            for c in range(min_x, max_x):

                if grid[r, c] != -1:
                    continue

                x = c*self.resolution
                y = r*self.resolution

                dx = x - rx
                dy = y - ry

                distance = math.hypot(dx, dy)

                if distance < near or distance > far:
                    continue

                angle = math.atan2(dy, dx)
                angle_diff = math.atan2(
                    math.sin(angle - yaw),
                    math.cos(angle - yaw)
                )

                if abs(angle_diff) <= fov/2:
                    grid[r, c] = 0 # free cell

        return grid

        



def main():
    rclpy.init()
    node = GridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()