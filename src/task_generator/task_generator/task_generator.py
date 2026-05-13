#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import math

from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray

from robp_interfaces.srv import GetNextFrontier, ArmCommand
from robp_interfaces.msg import PoseStampedWithType


class GeneralPlanner(Node):

    def __init__(self):
        super().__init__('general_planner')

        # variables
        self.current_goal = None
        self.state = 'STARTING'
        self.frontier_result = None
        self.reached = False
        self.objects = {}
        self.boxes = {}
        self.picked_ids = set()

        self.current_target_id = None

        # Arm service state
        self.arm_future = None
        self.arm_done = False

        # publisher
        self.goal_pub = self.create_publisher(
            PoseStampedWithType,
            '/goal_pose',
            10
        )

        # subscribers
        self.create_subscription(
            MarkerArray,
            '/map_objects',
            self.detection_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/target_reached',
            self.reached_callback,
            10
        )

        # service clients
        self.frontier_client = self.create_client(
            GetNextFrontier,
            'get_next_frontier'
        )

        self.arm_client = self.create_client(
            ArmCommand,
            '/arm_command'
        )

        # timer
        self.timer = self.create_timer(0.2, self.timer_callback)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # =============================
    # Callbacks
    # =============================

    def detection_callback(self, msg):
        new_objects = {}
        new_boxes = {}

        for marker in msg.markers:
            if marker.ns == "O":
                new_objects[marker.id] = marker.pose
            elif marker.ns == "B":
                new_boxes[marker.id] = marker.pose

        self.objects = new_objects
        self.boxes = new_boxes

    def reached_callback(self, msg):
        self.reached = msg.data

    # =============================
    # TF
    # =============================

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            xpose = transform.transform.translation.x
            ypose = transform.transform.translation.y   
            zpose = transform.transform.translation.z   
            return (xpose, ypose, zpose)

        except Exception:
            return None

    # =============================
    # Navigation
    # =============================

    def publish_goal(self, t, x, y, w=1.0):
        goal = PoseStampedWithType()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = w

        goal.type = t

        self.current_goal = goal
        print("publish goal entered")
        self.goal_pub.publish(goal)

    # =============================
    # Object / Box Selection
    # =============================

    def get_closest_object(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None, None

        min_dist = float("inf")
        chosen_id = None
        chosen_pose = None

        for obj_id, pose in self.objects.items():

            if obj_id in self.picked_ids:
                continue

            dx = robot_pose[0] - pose.position.x
            dy = robot_pose[1] - pose.position.y
            dist = math.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                chosen_id = obj_id
                chosen_pose = pose

        return chosen_id, chosen_pose

    def get_closest_box(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None

        min_dist = float("inf")
        chosen_pose = None

        for pose in self.boxes.values():
            dx = robot_pose[0] - pose.position.x
            dy = robot_pose[1] - pose.position.y
            dist = math.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                chosen_pose = pose

        return chosen_pose

    def remove_picked_object(self, obj_id):
        self.picked_ids.add(obj_id)

    def get_near_object(self, x, y):
        return (x, y, 1)

    def get_near_box(self, x, y):
        return (x, y, 1)

    # =============================
    # ARM SERVICE
    # =============================

    def call_arm(self, command):

        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Arm service not available")
            return

        req = ArmCommand.Request()
        req.command = command

        self.arm_future = self.arm_client.call_async(req)
        self.arm_future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Arm success: {result.success}")
            self.arm_done = True
        except Exception as e:
            self.get_logger().error(f"Arm call failed: {e}")
            self.arm_done = False

    # =============================
    # MAIN STATE MACHINE
    # =============================

    def timer_callback(self):

        if self.state == 'STARTING':
            self.state = 'CHOOSE_OBJECT'

        elif self.state == 'CHOOSE_OBJECT':

            obj_id, pose = self.get_closest_object()

            if pose is not None:
                print("Object chosen")
                self.current_target_id = obj_id

                x, y, w = self.get_near_object(
                    pose.position.x,
                    pose.position.y
                )

                self.publish_goal("O", x, y, w)
                self.state = "GO_TO_OBJECT"

            else:
                self.state = "CHOOSE_EXPLO"

        elif self.state == 'GO_TO_OBJECT':
            print("state go to object reached")
            if self.reached:
                print("Object reached")
                self.reached = False
                self.state = "PICK_OBJECT"

        # ================= PICK =================

        elif self.state == 'PICK_OBJECT':

            #if self.arm_future is None:
                #self.arm_done = False
                #self.call_arm("pick")

            #elif self.arm_done:
                print("Object picked")
                #self.arm_future = None
                self.remove_picked_object(self.current_target_id)
                self.state = "CHOOSE_BOX"

        # ================= BOX =================

        elif self.state == 'CHOOSE_BOX':

            box = self.get_closest_box()

            if box is None:
                return

            print("Box chosen")

            x, y, w = self.get_near_box(
                box.position.x,
                box.position.y
            )

            self.publish_goal("B", x, y, w)
            self.state = "GO_TO_BOX"

        elif self.state == 'GO_TO_BOX':

            if self.reached:
                print("Box reached")
                self.reached = False
                self.state = "DROP_OBJECT"

        # ================= DROP =================

        elif self.state == 'DROP_OBJECT':

            #if self.arm_future is None:
                #self.arm_done = False
                #self.call_arm("drop")

            #elif self.arm_done:
                print("Object dropped")
                #self.arm_future = None
                self.state = "CHOOSE_OBJECT"

        # ================= EXPLORE =================

        elif self.state == 'CHOOSE_EXPLO':
            print("Exploration mode")
            self.state = "WAIT_FRONTIER"

        elif self.state == "WAIT_FRONTIER":
            print("Exploring goal chosen")
            self.state = "EXPLORE"

        elif self.state == 'EXPLORE':
            print("Exploration done")
            self.state = "DONE"

        elif self.state == 'DONE':
            print('MISSION COMPLETE')


def main():
    rclpy.init()
    time.sleep(1.0)
    node = GeneralPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
