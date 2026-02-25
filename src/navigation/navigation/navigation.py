#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, PoseStamped

class Navigation(Node):

    def __init__(self):
        super().__init__('navigation')

        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/map_objects',
            self.marker_callback,
            10
        )

        # self.goal_pub = self.create_publisher(Point, '/goal', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.goal_sent = False
        self.get_logger().info('Navigation Running...')

    def marker_callback(self, msg):
        if self.goal_sent:
            return

        for marker in msg.markers:
            if marker.ns == "O":
                goal = PoseStamped()
                goal.header = marker.header
                goal.pose.position.x = marker.pose.position.x - 0.5
                goal.pose.position.y = marker.pose.position.y
                goal.pose.position.z = 0.0
                goal.pose.orientation.x = 0.0
                goal.pose.orientation.y = 0.0
                goal.pose.orientation.z = 0.0
                goal.pose.orientation.w = 1.0

                self.goal_pub.publish(goal)
                
                self.get_logger().info(
                    f"Publishing goal: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
                )

                self.goal_sent = True
                break

def main():
    rclpy.init()
    node = Navigation()
    rclpy.spin(node)
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
