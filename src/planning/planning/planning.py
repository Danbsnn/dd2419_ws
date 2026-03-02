#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
import heapq
import math

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path


class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planning')

        self.grid = None
        self.resolution = None
        self.width = None
        self.height = None
        self.origin = None
        self.robot_pose = None

        # final goal inside workspace
        self.goal = Point()
        #self.goal.x = 0.8
        #self.goal.y = 0.3
        
        """self.goals = [
            (0.8, 0.3),
            #(1.3, 2.2),
            (1.4, 0.2),
            #(3.2, 1.5)
        ]"""
        self.goals = [
            (0.8, 0.3),
            (1.0, 2.2),
            (2.5, 0.2),
            (3.2, 1.5)
        ]

        self.goal_index = 0
        self.current_path = []
        self.path_idx = 0

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)  # this node
        self.point_pub = self.create_publisher(Point, '/goal', 10) # subscribe

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/localized_pose', self.pose_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Path planner started")


    def map_callback(self, msg):

        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = msg.info.origin

        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            (self.height, self.width)
        )


    def pose_callback(self, msg):
        self.robot_pose = msg.pose


    def control_loop(self):

        if self.grid is None or self.robot_pose is None:
            return

        # if no path → plan
        if not self.current_path:

            if self.goal_index >= len(self.goals):
                self.get_logger().info("All goals reached")
                return

            gx, gy = self.goals[self.goal_index]

            start = self.world_to_grid(
                self.robot_pose.position.x,
                self.robot_pose.position.y
            )

            goal = self.world_to_grid(gx, gy)

            path = self.a_star(start, goal)

            if path is None:
                self.get_logger().warn("No path found")
                return

            self.current_path = path
            self.path_idx = 0
            self.publish_path(path)

            self.get_logger().info(f"Planned path with {len(path)} points")

        # follow path
        if self.path_idx >= len(self.current_path):

            self.goal_index += 1
            self.current_path = []
            self.path_idx = 0
            return

        gy, gx = self.current_path[self.path_idx]
        x, y = self.grid_to_world(gx, gy)

        dx = x - self.robot_pose.position.x
        dy = y - self.robot_pose.position.y
        dist = math.hypot(dx, dy)

        if dist < 0.05: # 0.05
            self.path_idx += 1
            return

        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0

        self.point_pub.publish(p)


    def world_to_grid(self, x, y):
        gx = int((x - self.origin.position.x) / self.resolution)
        gy = int((y - self.origin.position.y) / self.resolution)
        return gy, gx


    def grid_to_world(self, gx, gy):
        x = self.origin.position.x + (gx + 0.5) * self.resolution
        y = self.origin.position.y + (gy + 0.5) * self.resolution
        return x, y


    def a_star(self, start, goal):

        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])

        neighbors = [(1,0),(-1,0),(0,1),(0,-1)]

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_cost = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dy, dx in neighbors:
                ny = current[0] + dy
                nx = current[1] + dx

                if 0 <= ny < self.height and 0 <= nx < self.width:
                    if self.grid[ny, nx] != 0:
                        continue

                    neighbor = (ny, nx)
                    tentative = g_cost[current] + 1

                    if neighbor not in g_cost or tentative < g_cost[neighbor]:
                        came_from[neighbor] = current
                        g_cost[neighbor] = tentative
                        f = tentative + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f, neighbor))

        return None


    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


    def publish_path(self, grid_path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gy, gx in grid_path:
            x, y = self.grid_to_world(gx, gy)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main():

    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()