#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from robp_interfaces.srv import GetNextFrontier


class GeneralPlanner(Node):

    def __init__(self):
        super().__init__('general_planner')

        # variables
        self.current_goal = None
        self.state = 'STARTING'
        self.frontier_result = None
        self.reached = False
        self.objects = {}        # id -> pose
        self.boxes = {}          # id -> pose
        self.picked_ids = set()  # already picked object ids


        # publisher
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # listeners
        self.detection_sub = self.create_subscription(
            MarkerArray,
            '/map_objects',
            self.detection_callback,
            10
        )

        self.reached_sub = self.create_subscription(
            Bool,
            '/target_reached',
            self.reached_callback,
            10
        )

        # frontier service client
        self.frontier_client = self.create_client(
            GetNextFrontier,
            'get_next_frontier'
        )

        # timer
        self.timer = self.create_timer(
            0.2,
            self.timer_callback
        )

        # transform initialisation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.reached = msg

    def get_robot_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            xpose = transform.transform.translation.x
            ypose = transform.transform.translation.x
            zpose = transform.transform.translation.x
            
            # map_pose = do_transform_pose(pose, transform) # transform between 0,0,0 in base_link frame and map frame
            return (xpose, ypose, zpose)

        except Exception:
            return None
        
    # Navigation
    def publish_goal(self, x, y, w=1.0):

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = w

        self.current_goal = goal
        self.goal_pub.publish(goal)

    def get_closest_object(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None, None

        min_dist = float("inf")
        chosen_id = None
        chosen_pose = None

        for obj_id, pose in self.objects.items():

            if obj_id in self.picked_ids: # if object not alread picked
                continue

            dx = robot_pose[0]- pose.position.x
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

    def get_near_object(self, obj_x, obj_y):
        return (obj_x, obj_y, 1)

    def get_near_box(self, box_x, box_y):
        return(box_x, box_y, 1)

    # Service call
    # def request_next_frontier(self):

    #     if not self.frontier_client.wait_for_service(timeout_sec=1.0):
    #         return

    #     req = GetNextFrontier.Request()
    #     future = self.frontier_client.call_async(req)
    #     future.add_done_callback(self.frontier_response_callback)
            
    # def frontier_response_callback(self, future):

    #     try:
    #         result = future.result()
    #     except Exception:
    #         self.frontier_result = False
    #         return

    #     if not result.success:
    #         self.frontier_result = False
    #     else:
    #         self.frontier_result = result

    # Main loop
    def timer_callback(self):

        if self.state == 'STARTING':
            # Wait and then go to CHOOSE_OBJECT
            self.state = 'CHOOSE_OBJECT'

        elif self.state == 'CHOOSE_OBJECT':
            # Find closest object position and change state to GO_TO_OBJECT. If no object found, go to state EXPLORE

            obj_id, pose = self.get_closest_object()

            if pose is not None:
                print("Object choosed")
                self.current_target_id = obj_id
                position_o_x, position_o_y, orientation_o = self.get_near_object(pose.position.x, pose.position.y)
                self.publish_goal(
                    position_o_x,
                    position_o_y,
                    orientation_o
                )
                self.state = "GO_TO_OBJECT"

            else:
                self.state = "CHOOSE_EXPLO"

        elif self.state == 'GO_TO_OBJECT':
            # Publish goal. When goal reached, go to state PICK_OBJECT
            if self.reached:
                print("Object reached")
                self.reached = False
                self.state = "PICK_OBJECT"
            
        elif self.state == 'PICK_OBJECT':
            # Call pick object service. When response, go to CHOOSE_BOX
            print("Object picked") # Simulate the pick service not functionnal yet
            self.remove_picked_object(self.current_target_id)
            self.state = "CHOOSE_BOX"
        
        elif self.state == 'CHOOSE_BOX':
            # Find closest box position and change state to GO_TO_BOX
            box = self.get_closest_box()
            print("Box choosed")
            position_b_x, position_b_y, orientation_b = self.get_near_box(box.position.x, box.position.y)
            self.publish_goal(
                position_b_x,
                position_b_y,
                orientation_b
            )
            self.state = "GO_TO_BOX"
            

        elif self.state == 'GO_TO_BOX':
            # When goal reached, go to state DROP_OBJECT
            if self.reached:
                print("Box reached")
                self.reached = False
                self.state = "DROP_OBJECT"
            
        elif self.state == 'DROP_OBJECT':
            # Call drop object service. When response, go to state CHOOSE_OBJECT
            print("Object dropped")
            self.state = "CHOOSE_OBJECT"
        
        elif self.state == 'CHOOSE_EXPLO':
            # Call GetNextFrontier. If response, go to state EXPLORE. If no response, go to state DONE
            # self.frontier_result = None
            # self.request_next_frontier()
            print("Exploration mode")
            self.state = "WAIT_FRONTIER"

        elif self.state == "WAIT_FRONTIER":

            # if self.frontier_result is None:
            #     return  # still waiting

            # result = self.frontier_result
            # self.frontier_result = None

            # if result is False:
            #     self.state = "DONE"
            # else:
            #     self.publish_goal(result.x, result.y, 1.0)
            #     self.state = "EXPLORE"
            print("Exploring goal choosed")
            self.state = "EXPLORE"
            

        elif self.state == 'EXPLORE':
            # When goal reached, go to state CHOOSE_OBJECT
            # if self.reached:
            #     self.state = "CHOOSE_OBJECT"
            print("Exploration goal reached")
            self.state = "DONE"


        elif self.state == 'DONE':
            # Stop robot
            print('MISSION COMPLETE')



def main():

    rclpy.init()

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