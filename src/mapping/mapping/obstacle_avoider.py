#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class SimplePathSplitter(Node):
    def __init__(self):
        super().__init__('simple_path_splitter')

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/planned_goal',
            self.goal_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/odom_pose',
            self.pose_callback,
            10
        )

        self.reached_sub = self.create_subscription(
            Bool,
            '/target_reached',
            self.reached_callback,
            10
        )

        # Publisher to motion_control
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.map_msg = None
        self.robot_pose = None

        self.pending_final_goal = None
        self.waiting_for_midpoint = False

        # Simple tuning
        self.occupied_threshold = 50
        self.step_size = 0.05       # sample spacing along line
        self.offset_distance = 0.35 # side-step distance for midpoint

        self.get_logger().info("Simple path splitter started.")

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    def reached_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.waiting_for_midpoint and self.pending_final_goal is not None:
            self.get_logger().info("Midpoint reached, sending final goal.")
            self.goal_pub.publish(self.pending_final_goal)
            self.waiting_for_midpoint = False
            self.pending_final_goal = None

    def goal_callback(self, goal_msg: PoseStamped):
        if self.map_msg is None or self.robot_pose is None:
            self.get_logger().warn("Waiting for map and robot pose.")
            return

        start_x = self.robot_pose.pose.position.x
        start_y = self.robot_pose.pose.position.y
        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y

        # Clear any old state
        self.pending_final_goal = None
        self.waiting_for_midpoint = False

        if not self.line_blocked(start_x, start_y, goal_x, goal_y):
            self.get_logger().info("Direct path is free, sending goal directly.")
            self.goal_pub.publish(goal_msg)
            return

        self.get_logger().info("Direct path blocked, trying simple split.")

        midpoint = self.find_simple_midpoint(start_x, start_y, goal_x, goal_y, goal_msg)

        if midpoint is None:
            self.get_logger().warn("Could not find simple midpoint, sending original goal anyway.")
            self.goal_pub.publish(goal_msg)
            return

        self.pending_final_goal = goal_msg
        self.waiting_for_midpoint = True

        self.get_logger().info(
            f"Sending midpoint first: ({midpoint.pose.position.x:.2f}, {midpoint.pose.position.y:.2f})"
        )
        self.goal_pub.publish(midpoint)

    def world_to_grid(self, x, y):
        info = self.map_msg.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        return gx, gy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.map_msg.info.width and 0 <= gy < self.map_msg.info.height

    def is_occupied_world(self, x, y):
        gx, gy = self.world_to_grid(x, y)

        if not self.in_bounds(gx, gy):
            return True  # treat outside map as blocked

        idx = gy * self.map_msg.info.width + gx
        value = self.map_msg.data[idx]

        # Unknown (-1) is also treated as blocked to stay conservative
        return value >= self.occupied_threshold

    def line_blocked(self, x0, y0, x1, y1):
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(dist / self.step_size))

        for i in range(steps + 1):
            t = i / steps
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            if self.is_occupied_world(x, y):
                return True

        return False

    def find_simple_midpoint(self, x0, y0, x1, y1, template_goal):
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return None

        # Unit perpendiculars
        px = -dy / dist
        py = dx / dist

        # Base midpoint on the straight segment
        mx = 0.5 * (x0 + x1)
        my = 0.5 * (y0 + y1)

        # Try left then right, increasing offset a bit
        offsets = [
            self.offset_distance,
            -self.offset_distance,
            2.0 * self.offset_distance,
            -2.0 * self.offset_distance,
        ]

        for off in offsets:
            cx = mx + off * px
            cy = my + off * py

            # Both subsegments must be free
            if self.line_blocked(x0, y0, cx, cy):
                continue
            if self.line_blocked(cx, cy, x1, y1):
                continue

            midpoint = PoseStamped()
            midpoint.header.frame_id = template_goal.header.frame_id
            midpoint.header.stamp = self.get_clock().now().to_msg()
            midpoint.pose.position.x = cx
            midpoint.pose.position.y = cy
            midpoint.pose.position.z = template_goal.pose.position.z
            midpoint.pose.orientation = template_goal.pose.orientation
            return midpoint

        return None


def main():
    rclpy.init()
    node = SimplePathSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
