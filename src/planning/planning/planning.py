#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planning')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_pose = None
        self.current_path = []
        self.path_idx = 0

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.point_pub = self.create_publisher(Point, '/goal', 10)

        # Subscriptions
        self.create_subscription(PoseStamped, '/localized_pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Path planner started")

    # ---------------- ROBOT POSE ----------------
    def pose_callback(self, msg: PoseStamped):
        try:
            # Transform robot pose into 'map' frame
            transform = self.tf_buffer.lookup_transform(
                'map',                 # target frame
                msg.header.frame_id,   # source frame
                rclpy.time.Time()      # latest transform
            )
            pose_map = do_transform_pose(msg, transform)
            self.robot_pose = pose_map.pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed for robot pose: {e}")
            return

    # ---------------- GOAL ----------------
    def goal_callback(self, msg: PoseStamped):
        try:
            # Transform goal pose into 'map' frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time()
            )
            goal_map = do_transform_pose(msg, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed for goal: {e}")
            return

        goal_x = goal_map.pose.position.x
        goal_y = goal_map.pose.position.y

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