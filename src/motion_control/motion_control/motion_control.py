#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

import math


class MotionControl(Node):

    def __init__(self):
        super().__init__('motion_control')

        self.motor_pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.reached_pub = self.create_publisher(Bool, '/target_reached', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(PoseStamped, 
                                '/localized_pose', 
                                self.pose_callback, 
                                10
                                )
        self.goal_sub = self.create_subscription(
                                Point,
                                '/goal',
                                self.goal_callback,
                                10
                            )
        self.goal_pose_sub = self.create_subscription(PoseStamped,
                                                '/goal_pose',
                                                self.goal_pose_callback,
                                                10)
        

        self.L = 0.30         # wheel separation (m)

        # Controller gains
        self.k1 = 0.5  # Velocity constant
        self.k2 = 2.0  # Angle
        self.k3 = 5  # Angle to velo constant
        self.v_max = 0.6
        self.omega_max = 0.6

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Motion Control Running...")

        self.x = None
        self.y = None
        self.theta = None   
        self.robot_frame = None
        self.x_t = None
        self.y_t = None

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

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

    def control_loop(self):

        if self.x is None or self.theta is None or self.x_t is None or self.y_t is None:
            return

        msg = DutyCycles()

        # Calculate errors
        dx = self.x_t - self.x
        dy = self.y_t - self.y
        d = math.sqrt(dx**2 + dy**2)
        theta_d = math.atan2(dy, dx)
        alpha = self.wrap_to_pi(theta_d - self.theta)

        # self.get_logger().info(f"Pos: ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})")
        # self.get_logger().info(f"Goal: ({self.x_t:.2f}, {self.y_t:.2f})")
        # self.get_logger().info(f"Distance: {d:.2f}, alpha: {alpha:.2f}")

        if d < 0.05:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.motor_pub.publish(msg)

            # Publish target reached
            reached_msg = Bool()
            reached_msg.data = True
            self.reached_pub.publish(reached_msg)

            self.get_logger().info("Target reached!")
            self.x_t = None
            self.y_t = None
            return

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


    def pose_callback(self, msg):
        # Update current pose from localization
        self.robot_frame = msg.header.frame_id # 'odom'
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        # Where the robot is facing
        (_, _, yaw) = euler_from_quaternion([msg.pose.orientation.x,
                                            msg.pose.orientation.y,
                                            msg.pose.orientation.z,
                                            msg.pose.orientation.w])
        self.theta = yaw

    def goal_callback(self, msg):
        self.x_t = msg.x 
        self.y_t = msg.y
        """(_, _, yaw) = euler_from_quaternion([
                                    msg.pose.orientation.x,
                                    msg.pose.orientation.y,
                                    msg.pose.orientation.z,
                                    msg.pose.orientation.w
                                    ])
        self.theta_t = yaw"""
        self.get_logger().info(f"New goal: ({self.x_t:.2f}, {self.y_t:.2f})")

    def goal_pose_callback(self, msg):
        if self.robot_frame is None:
            self.get_logger().warn("Cannot set goal yet; robot state unknown!")
            return

        # If the goal frame is different from odom
        if msg.header.frame_id != self.robot_frame:
            self.get_logger().info(f"Transforming goal from {msg.header.frame_id} to {self.robot_frame}...")
            transformed_goal = self.transform_pose(msg, self.robot_frame)
            
            if transformed_goal:
                self.x_t = transformed_goal.pose.position.x
                self.y_t = transformed_goal.pose.position.y
                self.get_logger().info(f"New goal (Transformed): ({self.x_t:.2f}, {self.y_t:.2f})")
        else:
            self.x_t = msg.pose.position.x
            self.y_t = msg.pose.position.y
            self.get_logger().info(f"New goal: ({self.x_t:.2f}, {self.y_t:.2f})")

def main():
    rclpy.init()
    node = MotionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



