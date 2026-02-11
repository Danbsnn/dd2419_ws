#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped
from tf_transformations import euler_from_quaternion

import math


class MotionControl(Node):

    def __init__(self):
        super().__init__('motion_control')

        self.motor_pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.reached_pub = self.create_publisher(Bool, '/target_reached', 10)

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

        self.L = 0.30         # wheel separation (m)

        # Controller gains
        self.k_rho = 0.2
        self.k_alpha = 0.5
        self.v_max = 0.2
        self.omega_max = 1.0

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Motion Control Running...")

        self.x = None
        self.y = None
        self.theta = None   
        self.x_t = None
        self.y_t = None

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def control_loop(self):

        if self.x is None or self.theta is None or self.x_t is None:
            return  # Wait for first pose update

        msg = DutyCycles()

        # Calculate errors
        dx = self.x_t - self.x
        dy = self.y_t - self.y
        rho = math.sqrt(dx**2 + dy**2)
        theta_d = math.atan2(dy, dx)
        alpha = self.wrap_to_pi(theta_d - self.theta)

        self.get_logger().info(f"Pos: ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})")
        self.get_logger().info(f"Goal: ({self.x_t:.2f}, {self.y_t:.2f})")
        self.get_logger().info(f"rho: {rho:.2f}, alpha: {alpha:.2f}")

        if rho < 0.05:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.motor_pub.publish(msg)

            # Publish target reached
            reached_msg = Bool()
            reached_msg.data = True
            self.reached_pub.publish(reached_msg)

            self.get_logger().info("Target reached!")
            return

        # Phase 1: Rotate to face the target
        if abs(alpha) > 0.1:  # 0.1 radians ≈ 5.7 degrees threshold
            v = 0.0
            omega = self.k_alpha * alpha
            omega = max(min(omega, self.omega_max), -self.omega_max)

            v_r = omega * self.L / 2.0
            v_l = -omega * self.L / 2.0
            
            msg.duty_cycle_right = max(min(v_r / self.v_max, 1.0), -1.0)
            msg.duty_cycle_left = max(min(v_l / self.v_max, 1.0), -1.0)

        # Phase 2: Drive straight to target
        else:
            v = self.k_rho * rho
            v = max(min(v, self.v_max), -self.v_max)
            omega = 0.0

            v_r = v
            v_l = v
            
            msg.duty_cycle_right = max(min(v_r / self.v_max, 1.0), -1.0)
            msg.duty_cycle_left = max(min(v_l / self.v_max, 1.0), -1.0)

        self.motor_pub.publish(msg)


    def pose_callback(self, msg):
        # Update current pose from localization
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
        self.get_logger().info(f"New goal: ({self.x_t:.2f}, {self.y_t:.2f})")

def main():
    rclpy.init()
    node = MotionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



