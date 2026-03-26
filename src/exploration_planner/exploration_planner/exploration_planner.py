#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robp_interfaces.srv import GetNextFrontier
import math


class SimpleTaskPlanner(Node):

    def __init__(self):
        super().__init__('simple_task_planner')

        self.robot_position = None
        self.current_goal = None
        self.exploration_done = False

        # publisher
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # subscriber
        self.loc_sub = self.create_subscription(
            PoseStamped,
            '/odom_pose',
            self.loc_callback,
            10
        )

        # frontier service client
        self.frontier_client = self.create_client(
            GetNextFrontier,
            'get_next_frontier'
        )

        # timer
        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )


    # Callbacks
    def loc_callback(self, msg):
        self.robot_position = msg


    # Navigation
    def arrived_at_goal(self):

        if self.robot_position is None or self.current_goal is None:
            return False

        dx = self.robot_position.pose.position.x - self.current_goal.pose.position.x
        dy = self.robot_position.pose.position.y - self.current_goal.pose.position.y

        distance = math.hypot(dx, dy)

        return distance < 0.2


    def publish_goal(self, x, y):

        goal = PoseStamped()

        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.current_goal = goal

        self.goal_pub.publish(goal)

        self.get_logger().info(f"New exploration goal: {x:.2f}, {y:.2f}")


    # Service call
    def request_next_frontier(self):
        # Vérifie que le service existe
        if not self.frontier_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Frontier service not available")
            return

        # Crée la requête
        req = GetNextFrontier.Request()

        # Appel non bloquant
        future = self.frontier_client.call_async(req)

        # Ajoute un callback pour traiter la réponse
        future.add_done_callback(self.frontier_response_callback)
        
    def frontier_response_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        if not result.success:
            self.get_logger().info("Exploration complete!")
            self.exploration_done = True
            return

        # Publie le nouveau goal
        self.publish_goal(result.x, result.y)

    # Main loop
    def timer_callback(self):

        if self.exploration_done:
            return

        if self.current_goal is None:
            self.request_next_frontier()
            return

        if self.arrived_at_goal():
            self.request_next_frontier()


def main():

    rclpy.init()

    node = SimpleTaskPlanner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()