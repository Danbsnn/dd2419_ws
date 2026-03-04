#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robp_interfaces.srv import ArmControl  # Service type for pick/drop
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener

class SimpleTaskPlanner(Node):
    def __init__(self):
        super().__init__('simple_task_planner')

        # Initializing variables
        self.status = "GO_TO_OBJECT"
        self.robot_position = None
        self.object_position = None
        self.box_position = None
        self.current_object = None
        self.pick_position = None
        self.drop_position = None
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for goal
        self.goal_pub = self.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            10
        )

        # Subscriber for robot position
        self.loc_sub = self.create_subscription(
            PoseStamped,
            '/localized_pose',
            self.loc_callback,
            10
        )
        # Subscriber for object pose
        self.object_loc_sub = self.create_subscription(
            MarkerArray,
            '/map_objects',
            self.object_loc_callback,
            10
        )
        # Subscriber for box pose
        self.box_loc_sub = self.create_subscription(
            MarkerArray,
            '/box_detected',
            self.box_loc_callback,
            10
        )

        # Timer for simple task execution
        self.timer = self.create_timer(
            1.0, 
            self.timer_callback
        )

        # Clients for pick/drop
        self.pick_client = self.create_client(ArmControl, '/arm/execute')


    # Subscribers callbacks
    def loc_callback(self, msg):
        self.robot_position = msg

    def object_loc_callback(self, msg):
        if len(msg.markers) > 0:
            marker = msg.markers[0]
            marker_base = self.tf_buffer.transform(marker, 'base_link')
            x = marker_base.pose.position.x
            y = marker_base.pose.position.y
            z = marker_base.pose.position.z
            self.pick_position = (x, y, z)

            self.object_position = PoseStamped()
            self.object_position.header = marker.header
            self.object_position.pose.position.x = marker.pose.position.x
            self.object_position.pose.position.y = marker.pose.position.y
            self.object_position.pose.position.z = marker.pose.position.z
            self.object_position.pose.orientation = marker.pose.orientation
            self.current_object = self.object_position 

    def box_loc_callback(self, msg):
        if len(msg.markers) > 0:
            marker = msg.markers[0]
            marker_base = self.tf_buffer.transform(marker, 'base_link')
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z
            self.drop_position = (x, y, z)

            self.box_position = PoseStamped()
            self.box_position.header = marker.header
            self.box_position.pose.position.x = marker.pose.position.x
            self.box_position.pose.position.y = marker.pose.position.y
            self.box_position.pose.position.z = marker.pose.position.z
            self.box_position.pose.orientation = marker.pose.orientation
            
    # Functions
    def arrived_at_goal(self):
        if self.robot_position is None:
            return False
        if self.status == "GO_TO_OBJECT" and self.object_position is not None:
            dx = self.robot_position.pose.position.x - self.object_position.pose.position.x
            dy = self.robot_position.pose.position.y - self.object_position.pose.position.y
            distance = (dx**2 + dy**2)**0.5
            return distance < 0.1
        elif self.status == "GO_TO_BOX" and self.box_position is not None:
            dx = self.robot_position.pose.position.x - self.box_position.pose.position.x
            dy = self.robot_position.pose.position.y - self.box_position.pose.position.y
            distance = (dx**2 + dy**2)**0.5
            return distance < 0.1
        return False
    
    def publish_goal(self, goal):
        if goal is not None:
            goal_pose = PoseStamped()
            goal_pose.header = goal.header
            goal_pose.pose.position.x = goal.pose.position.x - 0.5
            goal_pose.pose.position.y = goal.pose.position.y 
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_pose)

    def call_service_object(self, x, y, z, mode):
        if not self.pick_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Pick service not available')
            return False
        req = ArmControl.Request()
        req.x = x
        req.y = y
        req.z = z
        req.message = mode

        future = self.pick_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Object picked successfully')
            return True
        else:
            self.get_logger().error('Failed to pick object')
            return False

    # Timer callback for simple task execution
    def timer_callback(self):
        if self.status == "GO_TO_OBJECT":
            self.publish_goal(self.object_position)
            if self.arrived_at_goal():
                self.status = "PICK"

        elif self.status == "PICK":
            if self.call_service_object(self.pick_position[0], self.pick_position[1], self.pick_position[2], "pick"):
                self.status = "GO_TO_BOX"

        elif self.status == "GO_TO_BOX":
            self.publish_goal(self.box_position)
            if self.arrived_at_goal():
                self.status = "DROP"

        elif self.status == "DROP":
            if self.call_service_object(self.drop_position[0], self.drop_position[1], self.drop_position[2], "drop"):
                self.status = "DONE"

        elif self.status == "DONE":
            if not hasattr(self, 'done_logged'):
                self.get_logger().info("Task completed!")
                self.done_logged = True

    


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
