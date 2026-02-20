#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
import random

# newwwwwwwwwwwwwwwwwwwww

from geometry_msgs.msg import PoseStamped, Point 
from nav_msgs.msg import OccupancyGrid, Path
import heapq   # priority queue for A*
import math 
from tf2_ros import TransformBroadcaster

"""
Exploration Path Planner

- Uses OccupancyGrid from grid_publisher.py
- Selects random free cells (value == 0)
- Publishes goal positions in map frame
"""


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planning')

        # Map data
        # these variables will store inforamtion about the occupancy grid map,
        self.grid = None        # 2D numpy array of occupancy(map) values
        self.resolution = None  # meters per cell
        self.width = None
        self.height = None
        self.origin = None  # map origin in pose

        # robot state 
        self.robot_pose = None   # current pose (from localization)
        self.goal = None  # target goal position

        # Path 
        self.current_path = [] # list of grid cells froming the path 
        self.path_idx = 0 # which waypoint we are currently following

        # (output) Publisher in Rviz visualization
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # publisher (motion control) next waypoint for controller
        self.point_pub = self.create_publisher(
            Point,
            '/path',
            10
        )

        # (input)Subscriber to occupancy grid map from mapping node
        self.create_subscription(
            OccupancyGrid,
            '/occ_grid',
            self.map_callback,
            10
        )
        # Subscriber to localization robot pose
        self.create_subscription(
            PoseStamped,
            '/localized_pose',
            self.pose_callback,
            10
        )
        # Subscriber to exploration / user goal
        self.create_subscription(
            Point,
            '/next_point',
            self.goal_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        # timer to send waypoints
        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info("Path planner started")


    # Map
    def map_callback(self, msg: OccupancyGrid):
        """
        Receives occupancy grid and converts it to numpy array.
        """    

        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = msg.info.origin

        # convert 1D list to 2D grid
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            (self.height, self.width)
        )


    # Robot pose
    def pose_callback(self, msg: PoseStamped):
        # Receives robot pose from localization node
        self.robot_pose = msg.pose
        #self.try_plan_path()

    # Goal pose
    def goal_callback(self, msg: Point):
        # Receives a new goal and triggers path planning.
        self.goal = msg
        self.try_plan_path()

    # planning
    def try_plan_path(self):
        """
        Main planning function:
        Converts start & goal to grid coordinates,
        runs A*, stores path.
        """
        # ensure we have all required data
        if self.grid is None or self.robot_pose is None or self.goal is None:
            self.get_logger().warn("Missing data for planning")
            return
        
        # convert robot positon -> grid coordinates
        start = self.world_to_grid(
            self.robot_pose.position.x, 
            self.robot_pose.position.y
        )
        # convert goal position -> grid coordinates
        goal = self.world_to_grid(
            self.goal.x, 
            self.goal.y
        )

        path = self.a_star(start, goal)

        if path is None:
            self.get_logger().warn("No path found")
            return
        # store path and reset waypoint index
        self.current_path = path
        self.path_idx = 0
        # publish path for visualization
        self.publish_path(path)

        self.get_logger().info(f"Planned path with {len(path)} points")
          
        
    # timer, Control loop 
    def control_loop(self):
        """
        Sends next waypoint to controller.
        Robot moves waypoint by waypoint.
        """
        if not self.current_path:
            return

        if self.robot_pose is None:
            return
        if self.path_idx >= len(self.current_path):
            return
        # get next waypoint in grid
        gy, gx = self.current_path[self.path_idx]
        # convert to world coordinates
        x, y = self.grid_to_world(gx, gy)

        # distance from robot to waypoint
        dx = x - self.robot_pose.position.x
        dy = y - self.robot_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        #if close enough -> go to next waypoint
        if dist < 0.1:
            self.path_idx += 1
            return
        
        # publish waypoint
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0

        self.point_pub.publish(p)


    # coordinates
    def world_to_grid(self, x, y):
        """
        Convert world coordinates (meters)
        → grid coordinates (cell index)
        """
        gx = int((x - self.origin.position.x) / self.resolution)
        gy = int((y - self.origin.position.y) / self.resolution)
        return (gy, gx)
    
    def grid_to_world(self, gx, gy):
        """
        Convert grid cell → world coordinates.
        Uses cell center.
        """
        x = self.origin.position.x + (gx + 0.5) * self.resolution
        y = self.origin.position.y + (gy + 0.5) * self.resolution
        return x, y
    
    # A* algorithm
    def a_star(self, start, goal):
        # Implement A* pathfinding here
        # Return list of (x, y) grid coordinates from start to goal
        # distance heuristic
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        # 4-connected grid neighbors (up, down, left, right)
        neighbors = [(1,0), (-1,0), (0,1), (0,-1)]
        # priority queue for open set
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {} # parent pointers
        g_cost = {start: 0} # cost from start to node

        while open_set:
            _, current = heapq.heappop(open_set)
            # Goal reached
            if current == goal:
                return self.reconstruct_path(came_from, current)
            # explore neighbors
            for dx, dy in neighbors:
                ny = current[0] + dy
                nx = current[1] + dx
            # check boundaries
            if ny < 0 or nx < 0 or ny >= self.height or nx >= self.width:
                continue
            # skip obstacles 
            if self.grid[ny, nx] != 0:
                continue

            neightbor = (ny, nx)
            tentative_g_cost = g_cost[current] + 1
            # better path found
            if neightbor not in g_cost or tentative_g_cost < g_cost[neightbor]:
                came_from[neightbor] = current
                g_cost[neightbor] = tentative_g_cost
                f = tentative_g_cost + heuristic(neightbor, goal)
                heapq.heappush(open_set, (f, neightbor))

        return None
    
    def reconstruct_path(self, came_from, current):
        """
        Backtracks from goal to start using parent dictionary.
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    

    # publish path to Rviz2 visualization
    def publish_path(self, grid_path):
        """
        Publishes nav_msgs/Path for RViz visualization.
        """
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()


        for gy, gx in grid_path:
            x, y = self.grid_to_world(gx, gy)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(grid_path)} waypoints")

def main():
    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    

