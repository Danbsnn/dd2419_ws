#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import random
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class RandomGoalGenerator(Node):
    def __init__(self):
        super().__init__('random_goal_generator')

        # Publisher for goal
        self.pub = self.create_publisher(Pose, '/goal', 10)

        # Subscriber for robot position
        self.sub = self.create_subscription(
            Pose,
            '/localized_pose',
            self.loc_callback,
            10
        )

        # Map size (rectangle 1.5m x 2m), considering robot starting at 0,0,0
        self.x_min = 0.0
        self.x_max = 1.5
        self.y_min = 0.0
        self.y_max = 2.0

        # Tolerance = 5 cm
        self.dist_tolerance = 0.05
        #self.angle_tolerance = 0.1

        # Current goal
        self.goal = Pose()

        # Generate first goal
        self.generate_random_goal()

    def loc_callback(self, msg):
        x_robot = msg.position.x
        y_robot = msg.position.y
        #w_robot = msg.orientation.w

        x_goal = self.goal.position.x
        y_goal = self.goal.position.y
        #w_goal = self.goal.orientation.w

        # Distance robot -> goal
        dist = math.sqrt((x_goal - x_robot)**2 + (y_goal - y_robot)**2)

        if dist < self.dist_tolerance: # and abs(w_goal - w_robot) < self.angle_tolerance:
            self.get_logger().info('Goal reached, generating new goal...')
            self.generate_random_goal()

    def generate_random_goal(self):
        self.goal = Pose()

        # Random point in rectangle
        self.goal.position.x = random.uniform(self.x_min, self.x_max)
        self.goal.position.y = random.uniform(self.y_min, self.y_max)
        self.goal.position.z = 0.0

        # Orientation (not used for now)
        #angle = random.uniform(-math.pi, math.pi)
        #qx, qy, qz, qw = quaternion_from_euler(0, 0, angle)
        #self.goal.orientation.x = qx
        #self.goal.orientation.y = qy
        #self.goal.orientation.z = qz
        #self.goal.orientation.w = qw

        self.goal.orientation.w = 1.0 # No rotation

        self.pub.publish(self.goal)

        self.get_logger().info(
            f'New goal: x={self.goal.position.x:.3f}, y={self.goal.position.y:.3f}, angle={angle:.3f}'
        )


def main():
    rclpy.init()
    node = RandomGoalGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
