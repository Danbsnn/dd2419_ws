#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from nav_msgs.msg import Path
import math
import time


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.motor_pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.reached_pub = self.create_publisher(Bool, '/target_reached', 10)



        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10
        )
        self.reverse_sub = self.create_subscription(
            Bool, '/reverse_robot', self.reverse_callback, 10)

        self.reversing = False
        self.reverse_end_time = 0.0

        self.L = 0.30         # wheel separation (m)

        # Controller gains
        self.k1 = 1.2  # Velocity constant
        self.k2 = 2.0  # Angle
        self.k3 = 5  # Angle to velo constant
        self.v_max = 0.2
        self.omega_max = 0.7

        self.robot_frame = 'odom'
        self.lookahead_distance = 0.20
        self.goal_tolerance = 0.05

        self.x = None
        self.y = None
        self.theta = None  
        
        self.x_t = None
        self.y_t = None
        self.current_path = None
        self.final_x = None
        self.final_y = None
        
        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Motion Control Running...")

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def reverse_callback(self, msg):
        if msg.data:
            self.get_logger().info("Reverse command received.")

            self.reversing = True

            # Reverse for 0.5 seconds
            self.reverse_end_time = time.time() + 0.5

    def transform_pose(self, input_pose, target_frame):
        try:
            if not self.tf_buffer.can_transform(target_frame, input_pose.header.frame_id, rclpy.time.Time()):
                self.get_logger().warn(f'Transform from {input_pose.header.frame_id} to {target_frame} not ready')
                return None

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                input_pose.header.frame_id,
                rclpy.time.Time()
            )
            pose_transformed = tf2_geometry_msgs.do_transform_pose(input_pose.pose, transform)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = target_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose_transformed
            return pose_stamped
           
        except TransformException as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return None

    def transform_path(self, input_path, target_frame):
        transformed_path = Path()
        transformed_path.header.frame_id = target_frame
        transformed_path.header.stamp = self.get_clock().now().to_msg()

        for pose_stamped in input_path.poses:
            transformed_pose = self.transform_pose(pose_stamped, target_frame)
            if transformed_pose is None:
                return None
            transformed_path.poses.append(transformed_pose)

        return transformed_path

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                'base_link',
                rclpy.time.Time()
            )

            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y

            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.theta = yaw
            
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

    def pick_lookahead_point(self):
        if self.current_path is None or len(self.current_path.poses) == 0:
            return None

        for i, pose in enumerate(self.current_path.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            d = math.hypot(px - self.x, py - self.y)

            if d >= self.lookahead_distance:
                self.current_path.poses = self.current_path.poses[i:]
                return pose
                
        self.current_path.poses = [self.current_path.poses[-1]]
        return self.current_path.poses[-1]

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path.")
            self.current_path = None
            self.final_x = None
            self.final_y = None
            return

        if msg.header.frame_id != self.robot_frame:
            self.get_logger().info(
                f"Transforming path from {msg.header.frame_id} to {self.robot_frame}..."
            )
            msg = self.transform_path(msg, self.robot_frame)
            if msg is None:
                self.get_logger().warn("Failed to transform path.")
                return

        final_target = msg.poses[-1]
        self.final_x = final_target.pose.position.x
        self.final_y = final_target.pose.position.y       

        self.current_path = msg

    def control_loop(self):
        if self.reversing:
            motor_msg = DutyCycles()

            if time.time() < self.reverse_end_time:
                motor_msg.duty_cycle_left = -0.2
                motor_msg.duty_cycle_right = -0.2
            else:
                motor_msg.duty_cycle_left = 0.0
                motor_msg.duty_cycle_right = 0.0
                self.reversing = False

                self.get_logger().info("Finished reversing.")

            self.motor_pub.publish(motor_msg)
            return
        
        if self.final_x is None or self.final_y is None:
            return

        if self.current_path is None:
            return

        self.get_robot_pose()
        target_pose = self.pick_lookahead_point()
        if target_pose is None:
            return

        self.x_t = target_pose.pose.position.x
        self.y_t = target_pose.pose.position.y
        d_final = math.hypot(self.final_x - self.x, self.final_y - self.y)
        
        msg = DutyCycles()
        
        if d_final < self.goal_tolerance:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.motor_pub.publish(msg)

            reached_msg = Bool()
            reached_msg.data = True
            self.reached_pub.publish(reached_msg)

            self.get_logger().info("Final path goal reached!")

            self.current_path = None
            self.final_x = None
            self.final_y = None
            return

        # Calculate errors
        dx = self.x_t - self.x
        dy = self.y_t - self.y
        d = math.hypot(dx, dy)
        theta_d = math.atan2(dy, dx)
        alpha = self.wrap_to_pi(theta_d - self.theta)

        # Phase 1: Rotate to face the target
        if alpha < 0:
            sign_alpha = -1
        else:
            sign_alpha = 1
       
        omega = sign_alpha * min(self.k2 * abs(alpha), self.omega_max)

        v_r = omega * self.L / 2.0
        v_l = -omega * self.L / 2.0

        # Phase 2: Drive straight to target
        gradual_const = math.exp(-self.k3*abs(alpha)**2)
        # self.get_logger().info(f"Transition speed constant: {gradual_const}")
        v = min(self.k1 * d, self.v_max)
       
        if d > 0.05 and v < 0.1:
            self.get_logger().info("Robot is moving too slow, so a min velocity is applied.")
            v = max(v, 0.1)

        v *= gradual_const
        # self.get_logger().info(f"Current Linear Speed: {v}")

        v_r += v
        v_l += v

        msg.duty_cycle_right = v_r
        msg.duty_cycle_left = v_l

        # self.get_logger().info(f"Moving with speed v_l = {v_l} and v_r = {v_r}")
        self.motor_pub.publish(msg)

def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
