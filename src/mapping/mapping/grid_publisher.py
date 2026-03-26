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

from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from tf2_geometry_msgs import do_transform_pose


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

        self.odom_pose_pub = self.create_publisher(
                                Pose, 
                                '/init_pose', 
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
                                '/odom_pose',
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
        self.origin_x = np.min(self.workspace[:,0])
        self.origin_y = np.min(self.workspace[:,1])
        raw_map = np.genfromtxt(map_path, delimiter=',', skip_header=1, dtype=None, encoding='utf-8')
        self.object_types = [row[0] for row in raw_map]
        self.object_coords = [[row[1]*0.01, row[2]*0.01, row[3]] for row in raw_map]

        self.resolution = 0.05  # 5cm cells
        max_x = int(np.max(self.workspace[:, 0]))
        max_y = int(np.max(self.workspace[:, 1]))
        self.width = int(max_x/(self.resolution))
        self.height = int(max_y/(self.resolution))
        self.get_logger().info(f"Initialized {self.width}x{self.height} cells")

        # Find starting pose from map file
        self.start_x = None
        self.start_y = None

        for i, obj_type in enumerate(self.object_types):
            if obj_type == 'S':
                self.start_x = self.object_coords[i][0]
                self.start_y = self.object_coords[i][1]
                break

        if self.start_x is None:
            self.get_logger().error("No starting point 'S' found in map file!")
        else:
            init_pose_msg = Pose()
            init_pose_msg.position.x = float(self.start_x)
            init_pose_msg.position.y = float(self.start_y)
            init_pose_msg.position.z = 0.0

            init_pose_msg.orientation.x = 0.0
            init_pose_msg.orientation.y = 0.0
            init_pose_msg.orientation.z = 0.0
            init_pose_msg.orientation.w = 1.0

            self.odom_pose_pub(init_pose_msg)
            self.get_logger().info(f"Published map→odom at ({self.start_x:.2f}, {self.start_y:.2f})")
            

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        m.info.origin.position.x = self.origin_x
        m.info.origin.position.y = self.origin_y
        m.info.origin.position.z = 0.0

        grid = self.dynamic_grid    

        # mark objects as occupied
        for i, (ox, oy, _) in enumerate(self.object_coords):
            if self.object_types[i] in ['O', 'B']:
                gx, gy = int((ox-self.origin_x)/self.resolution), int((oy-self.origin_y)/self.resolution)
                grid[max(0, gy-1):gy+2, max(0, gx-1):gx+2] = 100

        grid = self.update_visibility(grid)

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

    # First generation of workspace/map
    def generate_workspace(self):
        self.workspace_poly = Polygon(self.workspace)
        grid = np.full((self.height, self.width), 100, dtype=np.int8)

        for r in range(self.height):
            for c in range(self.width):

                x = c * self.resolution + self.origin_x
                y = r * self.resolution + self.origin_y

                if self.workspace_poly.contains(ShapePoint(x, y)):
                    grid[r, c] = -1  # unknown

        return grid

    # Update on the free cells infront of the robot
    def update_visibility(self, grid):

        if self.robot_pose is None:
            return grid

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.robot_pose.header.frame_id,
                rclpy.time.Time()
            )

            pose_transformed = do_transform_pose(
                self.robot_pose.pose,
                transform
            )

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return grid

        rx = pose_transformed.position.x
        ry = pose_transformed.position.y
        orientation = pose_transformed.orientation

        # quaternion → yaw
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # rectangle of vision and camera parameters
        cam_offset = 0.1
        length = 0.8
        width = 0.5

        camx = rx + cam_offset * math.cos(yaw)
        camy = ry + cam_offset * math.sin(yaw)

        # conversion in grid coordinates
        grid_cx = int((camx - self.origin_x) / self.resolution)
        grid_cy = int((camy - self.origin_y) / self.resolution)

        radius = int(length / self.resolution) + 2

        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):

                r = grid_cy + dr
                c = grid_cx + dc

                # if cell is in the rectangle
                if r < 0 or r >= self.height or c < 0 or c >= self.width:
                    continue

                # if cell is currently unknow
                if grid[r, c] != -1:
                    continue

                # conversion grid -> irl
                x = c * self.resolution + self.origin_x
                y = r * self.resolution + self.origin_y

                dx = x - camx
                dy = y - camy

                # projection in robot frame 
                forward =  math.cos(yaw) * dx + math.sin(yaw) * dy
                lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy

                if 0 < forward < length and abs(lateral) < width / 2:
                    grid[r, c] = 0 # set as free 

        return grid

            



def main():
    rclpy.init()
    node = GridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
