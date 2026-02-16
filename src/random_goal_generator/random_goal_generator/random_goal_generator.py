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

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.latest_map = None

        
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

    def map_callback(self, msg):
        """Stores the map"""
        self.latest_map = msg
        # Adjust bounds
        self.x_max = msg.info.width * msg.info.resolution
        self.y_max = msg.info.height * msg.info.resolution

    def is_point_occupied(self, x, y):
        """Checks if a world coordinate is occupied in the current map."""
        if self.latest_map is None:
            return True
        
        info = self.latest_map.info
        # Convert world (m) to grid indices
        grid_x = int(x / info.resolution)
        grid_y = int(y / info.resolution)

        # Bounds check
        if 0 <= grid_x < info.width and 0 <= grid_y < info.height:
            index = (grid_y * info.width) + grid_x
            value = self.latest_map.data[index]
            # 0 is Free. Anything else (100, -1) is treated as occupied
            return value != 0
        return True

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
        new_x = 0.0
        new_y = 0.0
        valid_found = False
        attempts = 0
        while not valid_found and attempts < 100:
            temp_x = random.uniform(self.x_min, self.x_max)
            temp_y = random.uniform(self.y_min, self.y_max)
            
            if not self.is_point_occupied(temp_x, temp_y):
                new_x = temp_x
                new_y = temp_y
                valid_found = True
            attempts += 1

        if valid_found:
            self.goal.position.x = new_x
            self.goal.position.y = new_y
            self.goal.orientation.w = 1.0
            self.pub.publish(self.goal)
            self.get_logger().info(f'New valid goal: x={new_x:.2f}, y={new_y:.2f}')
        else:
            self.get_logger().error('Failed to find a valid goal after 100 attempts!')


def main():
    rclpy.init()
    node = RandomGoalGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
