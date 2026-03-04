#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path

from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planning')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_pose = None
        self.current_path = []
        self.path_idx = 0

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.point_pub = self.create_publisher(Point, '/goal', 10)

        # subscriptions
        self.create_subscription(PoseStamped, '/localized_pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Path planner started")

    # ---------------- ROBOT POSE ----------------
    def pose_callback(self, msg):
        #self.robot_pose = msg.pose
         # Convert robot pose to map frame
        try:
            pose_msg_map = self.tf_buffer.transform(msg, "map")
            self.robot_pose = pose_msg_map.pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

    # ---------------- GOAL ----------------
    def goal_callback(self, msg):
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        # create a simple straight path
        if self.robot_pose is not None:
            start = (self.robot_pose.position.x, self.robot_pose.position.y)
            self.current_path = [start, (goal_x, goal_y)]
            self.path_idx = 0
            self.publish_path(self.current_path)
            self.get_logger().info(f"Received new goal: x={goal_x:.2f}, y={goal_y:.2f}")

    # ---------------- CONTROL LOOP ----------------
    def control_loop(self):
        if self.robot_pose is None or not self.current_path:
            return

        if self.path_idx >= len(self.current_path):
            return

        target = self.current_path[self.path_idx]
        dx = target[0] - self.robot_pose.position.x
        dy = target[1] - self.robot_pose.position.y
        dist = (dx**2 + dy**2)**0.5

        # move to next waypoint if close
        if dist < 0.05:
            self.path_idx += 1
            return

        # publish current waypoint for robot to follow
        p = Point()
        p.x = target[0]
        p.y = target[1]
        p.z = 0.0
        self.point_pub.publish(p)

    # ---------------- PUBLISH PATH ----------------
    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path:
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