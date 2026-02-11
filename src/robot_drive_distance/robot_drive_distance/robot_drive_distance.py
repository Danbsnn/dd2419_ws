"""
 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
import math
import random

class DriveToPoint(Node):
    def __init__(self, target_x, target_y):
        super().__init__('drive_to_point')

        # Publishers & subscribers
        self.pub = self.create_publisher(
            DutyCycles, '/phidgets/motor/duty_cycles', 10
        )
        self.sub = self.create_subscription(
            Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10
        )

        # Encoder values
        self.start_left = None
        self.start_right = None
        self.prev_left = None
        self.prev_right = None
        self.left_encoder = 0
        self.right_encoder = 0

        # Robot parameters
        self.r = 0.047                 # wheel radius (m)
        self.L = 0.30                  # wheel separation (m)
        self.ticks_per_rev = 2872

        # Pose (odometry)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Target
        self.x_t = target_x
        self.y_t = target_y

        # Control gains
        self.k_rho = 0.8
        self.k_alpha = 2.0

        # Limits
        self.v_max = 0.4
        self.omega_max = 2.0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f"Driving to target (x={self.x_t}, y={self.y_t})"
        )

    # --------------------------------------------------
    # Encoder callback
    # --------------------------------------------------
    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        if self.start_left is None:
            self.start_left = self.left_encoder
            self.start_right = self.right_encoder
            self.prev_left = self.left_encoder
            self.prev_right = self.right_encoder

    # --------------------------------------------------
    # Utility functions
    # --------------------------------------------------
    def ticks_to_meters(self, ticks):
        return (2 * math.pi * self.r) * (ticks / self.ticks_per_rev)

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # --------------------------------------------------
    # Main control loop
    # --------------------------------------------------
    def control_loop(self):
        if self.start_left is None:
            return

        # Encoder increments
        d_left_ticks = self.left_encoder - self.prev_left
        d_right_ticks = self.right_encoder - self.prev_right

        self.prev_left = self.left_encoder
        self.prev_right = self.right_encoder

        # Distance traveled by wheels
        d_left = self.ticks_to_meters(d_left_ticks)
        d_right = self.ticks_to_meters(d_right_ticks)

        # Forward kinematics (odometry)
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.L

        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta = self.wrap_to_pi(self.theta + d_theta)

        # Distance & heading to target
        dx = self.x_t - self.x
        dy = self.y_t - self.y
        rho = math.sqrt(dx**2 + dy**2)
        theta_d = math.atan2(dy, dx)
        alpha = self.wrap_to_pi(theta_d - self.theta)

        self.get_logger().info(
            f"x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f}, ρ={rho:.2f}"
        )

        msg = DutyCycles()

        # Stop condition
        if rho < 0.05:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.pub.publish(msg)
            self.get_logger().info("Target reached!")
            self.timer.cancel()
            return

        # Control law
        v = self.k_rho * rho
        omega = self.k_alpha * alpha

        # Saturation
        v = max(min(v, self.v_max), -self.v_max)
        omega = max(min(omega, self.omega_max), -self.omega_max)

        # Inverse kinematics
        v_r = v + (omega * self.L / 2)
        v_l = v - (omega * self.L / 2)

        # Convert to duty cycle (simple proportional mapping)
        msg.duty_cycle_right = max(min(v_r / self.v_max, 1.0), -1.0)
        msg.duty_cycle_left = max(min(v_l / self.v_max, 1.0), -1.0)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveToPoint(target_x=-0.5, target_y=-0.5)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
class randompoint(Node)


if __name__ == "__main__":
    main()
"""


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles, Encoders
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

import math
import random
import time


class DriveToPoint(Node):

    def __init__(self):
        super().__init__('drive_random_targets')

        # -----------------------------
        # Publishers
        # -----------------------------
        self.motor_pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.reached_pub = self.create_publisher(Bool, '/target_reached', 10)
        self.target_pub = self.create_publisher(Point, '/current_target', 10)

        # -----------------------------
        # Subscriber
        # -----------------------------
        self.sub = self.create_subscription(Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10)

        # -----------------------------
        # Encoder state
        # -----------------------------
        self.start_left = None
        self.prev_left = None
        self.prev_right = None
        self.left_encoder = 0
        self.right_encoder = 0

        # -----------------------------
        # Robot parameters
        # -----------------------------
        self.r = 0.047        # wheel radius (m)
        self.L = 0.30         # wheel separation (m)
        self.ticks_per_rev = 2872

        # -----------------------------
        # Odometry state
        # -----------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # -----------------------------
        # Controller gains
        # -----------------------------
        self.k_rho = 0.5
        self.k_alpha = 1.5
        self.v_max = 0.3
        self.omega_max = 1.5

        # -----------------------------
        # Random target region
        # -----------------------------
        self.x_min, self.x_max = 0.0, 0.8
        self.y_min, self.y_max = 0.0, 0.8

        # Pause parameters
        self.pause_until = None
        self.pause_time = 1.0  # seconds

        # Generate first target
        self.generate_new_goal()

        # Start control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(" Random target navigation started")


    # -----------------------------
    # Generate random goal
    # -----------------------------
    def generate_new_goal(self):
        self.x_t = random.uniform(self.x_min, self.x_max)
        self.y_t = random.uniform(self.y_min, self.y_max)

        # Publish current target
        p = Point()
        p.x = self.x_t
        p.y = self.y_t
        p.z = 0.0
        self.target_pub.publish(p)

        self.get_logger().info(f" New target -> ({self.x_t:.2f}, {self.y_t:.2f})")


    # -----------------------------
    # Encoder callback
    # -----------------------------
    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        if self.start_left is None:
            self.start_left = self.left_encoder
            self.prev_left = self.left_encoder
            self.prev_right = self.right_encoder


    # -----------------------------
    # Utility functions
    # -----------------------------
    def ticks_to_meters(self, ticks):
        return (2 * math.pi * self.r) * (ticks / self.ticks_per_rev)

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


    # -----------------------------
    # Main control loop
    # -----------------------------
    def control_loop(self):

        if self.start_left is None:
            return

        msg = DutyCycles()

        # -----------------------------
        # Pause logic
        # -----------------------------
        if self.pause_until is not None:
            if time.time() < self.pause_until:
                # Stay stopped during pause
                msg.duty_cycle_left = 0.0
                msg.duty_cycle_right = 0.0
                self.motor_pub.publish(msg)
                return
            else:
                # Pause finished, generate new target
                self.pause_until = None
                self.generate_new_goal()

        # -----------------------------
        # Encoder increments
        # -----------------------------
        dL_ticks = self.left_encoder - self.prev_left
        dR_ticks = self.right_encoder - self.prev_right

        self.prev_left = self.left_encoder
        self.prev_right = self.right_encoder

        dL = self.ticks_to_meters(dL_ticks)
        dR = self.ticks_to_meters(dR_ticks)

        # -----------------------------
        # Odometry update
        # -----------------------------
        d_center = (dL + dR) / 2.0
        d_theta = (dR - dL) / self.L

        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta = self.wrap_to_pi(self.theta + d_theta)

        # -----------------------------
        # Error to goal
        # -----------------------------
        dx = self.x_t - self.x
        dy = self.y_t - self.y
        rho = math.sqrt(dx**2 + dy**2)
        theta_d = math.atan2(dy, dx)
        alpha = self.wrap_to_pi(theta_d - self.theta)

        # -----------------------------
        # Target reached
        # -----------------------------
        if rho < 0.05:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.motor_pub.publish(msg)

            # Publish target reached
            reached_msg = Bool()
            reached_msg.data = True
            self.reached_pub.publish(reached_msg)

            self.get_logger().info("Target reached, pausing...")

            # Start pause timer
            self.pause_until = time.time() + self.pause_time

            return  # do not run controller during pause

        # -----------------------------
        # Controller law
        # -----------------------------
        v = self.k_rho * rho
        omega = self.k_alpha * alpha

        # Saturation
        v = max(min(v, self.v_max), -self.v_max)
        omega = max(min(omega, self.omega_max), -self.omega_max)

        # Differential drive inverse kinematics
        v_r = v + omega * self.L / 2.0
        v_l = v - omega * self.L / 2.0

        msg.duty_cycle_right = max(min(v_r / self.v_max, 0.5), -0.5)
        msg.duty_cycle_left = max(min(v_l / self.v_max, 0.5), -0.5)

        self.motor_pub.publish(msg)


# -----------------------------
# Main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)

    node = DriveToPoint()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

