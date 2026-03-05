#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class FirstGoalNode(Node):
    def __init__(self):
        super().__init__('first_goal_node')
        self.get_logger().info("FirstGoalNode started")

        # Subscribe to detected objects
        self.sub = self.create_subscription(
            PoseStamped,
            '/detected_object',
            self.object_callback,
            10
        )

        # Publisher for navigation goal
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.goal_set = False

    def object_callback(self, msg: PoseStamped):
        # Only take the first detected object as the goal
        if not self.goal_set:
            self.goal_pub.publish(msg)
            self.goal_set = True
            self.get_logger().info(
                f"First goal set at X={msg.pose.position.x:.2f}, "
                f"Y={msg.pose.position.y:.2f}, "
                f"Z={msg.pose.position.z:.2f}"
            )

def main():
    rclpy.init()
    node = FirstGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
