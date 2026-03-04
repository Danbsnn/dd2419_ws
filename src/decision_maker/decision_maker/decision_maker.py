#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rosb_interfaces.srv import GetNextFrontier
from rosb_interfaces.srv import SelectObject
from rosb_interfaces.srv import SelectBox
from rosb_interfaces.srv import ComputeApproachGoal
from std_srvs.srv import Trigger

import math

class DecisionMaker(Node):
    def __init__(self):
        super().__init__('decision_maker')

        # Initializing variables
        self.status = "EXPLORE"
        self.current_object = None
        self.robot_position = None
        self.frontier_goal = None
        self.selected_box = None
        self.approach_goal = None
        self.goal_reached = False
        self.done_logged = False
        
        # Publisher for goal
        self.goal_pub = self.create_publisher(
            PoseStamped, 
            '/goal', 
            10
        )

        # Subscriber for robot position
        self.loc_sub = self.create_subscription(
            PoseStamped,
            '/localized_pose',
            self.loc_callback,
            10
        )

        # Subscriber for goal reached
        self.goal_reached_sub = self.create_subscription(
            PoseStamped,
            '/goal_reached',
            self.goal_reached_callback,
            10
        )

        # Timer for state machine execution
        self.timer = self.create_timer(
            1.0, 
            self.timer_callback
        )

        # Clients for services
        self.pick_client = self.create_client(Trigger, 'arm_pick')
        self.drop_client = self.create_client(Trigger, 'arm_drop')
        self.select_object_client = self.create_client(SelectObject, 'compute_select_object')
        self.select_box_client = self.create_client(SelectBox, 'compute_select_box')
        self.approach_goal_client = self.create_client(ComputeApproachGoal, 'compute_approach_goal')
        self.frontier_client = self.create_client(GetNextFrontier, 'get_next_frontier')




    # Subscribers callbacks
    def loc_callback(self, msg):
        self.robot_position = msg

    def goal_reached_callback(self, msg):
        self.goal_reached = msg

    # Functions
    def publish_goal(self, goal):
        if goal is not None:
            self.goal_pub.publish(goal)

    # Services call
    def call_service_get_next_frontier(self):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service not available")
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()
    
    def call_service_compute_select_object(self):
        response = self.call_service(self.frontier_client, GetNextFrontier.Request())
        if response and response.success:
            self.frontier_goal = response.frontier
            return True
        return False
    
    def call_service_compute_select_box(self):
        response = self.call_service(self.select_box_client, SelectBox.Request())
        if response and response.success:
            return response.box
        return None
    
    def call_service_compute_approach_goal(self, target, robot):
        if self.robot_position is None:
            return None

        req = ComputeApproachGoal.Request()
        req.target = target
        req.robot = self.robot_position

        response = self.call_service(self.approach_goal_client, req)
        if response and response.success:
            return response.approach_goal
        return None
    
    def call_service_pick_object(self):
        response = self.call_service(self.pick_client, Trigger.Request())
        return response.success if response else False
    
    def call_service_drop_object(self):
        response = self.call_service(self.drop_client, Trigger.Request())
        return response.success if response else False
    

    
    # Timer callback
    def timer_callback(self):
        if self.status == "EXPLORE":

            # First frontier
            if self.frontier_goal is None:
                if self.call_service_get_next_frontier():
                    self.publish_goal(self.frontier_goal)
                else:
                    self.status = "CHOOSE_OBJECT"
                return
            
            if self.goal_reached:
                self.goal_reached = None
                if self.call_service_get_next_frontier():
                    self.publish_goal(self.frontier_goal)
                else:
                    self.status = "CHOOSE_OBJECT"
        
        elif self.status == "CHOOSE_OBJECT":
            self.selected_object = self.call_service_compute_select_object()
            if self.selected_object is not None:
                self.current_object = self.selected_object
                self.status = "GO_TO_OBJECT"
            else:
                self.status = "DONE"

        elif self.status == "GO_TO_OBJECT":
            if self.approach_goal is None:
                self.approach_goal = self.call_service_compute_approach_goal(self.current_object, self.robot_position)
                if self.approach_goal:
                    self.publish_goal(self.approach_goal)
                return
            if self.goal_reached:
                self.approach_goal = None
                self.goal_reached = False
                self.status = "PICK"

        elif self.status == "PICK":
            if self.call_service_pick_object():
                self.status = "CHOOSE_BOX"
            else:
                self.status = "CHOOSE_OBJECT"

        elif self.status == "CHOOSE_BOX":
            self.selected_box = self.call_service_compute_select_box()
            self.box_position = self.selected_box
            self.status = "GO_TO_BOX"

        elif self.status == "GO_TO_BOX":
            if self.approach_goal is None:
                self.approach_goal = self.call_service_compute_approach_goal(self.box_position, self.robot_position)
                self.publish_goal(self.approach_goal)
                return 
            
            if self.goal_reached:
                self.approach_goal = None
                self.goal_reached = False
                self.status = "DROP"

        elif self.status == "DROP":
            self.call_service_drop_object()
            self.current_object = None
            self.status = "CHOOSE_OBJECT"

        elif self.status == "DONE":
            if not self.done_logged:
                self.get_logger().info("Task completed!")
                self.done_logged = True


def main():
    rclpy.init()
    node = DecisionMaker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
