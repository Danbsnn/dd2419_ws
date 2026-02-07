#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
import math

class DriveDistance(Node):
    def __init__(self, target_distance_m):
        super().__init__('drive_distance')

        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.sub = self.create_subscription(Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10)

        self.start_left = None
        self.start_right = None
        self.left_encoder = 0
        self.right_encoder = 0

        # Wheel parameters
        self.left_radius = 0.0485  # meters
        self.right_radius = 0.047  # meters
        self.ticks_per_revolution = 300  # adjust to your encoders

        self.target_distance = target_distance_m
        self.timer = self.create_timer(0.05, self.drive)
        self.get_logger().info(f"Driving forward {self.target_distance} meters...")

    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        if self.start_left is None:
            self.start_left = self.left_encoder
            self.start_right = self.right_encoder

    def ticks_to_meters(self, ticks, radius):
        circumference = 2 * math.pi * radius
        return circumference * (ticks / self.ticks_per_revolution)

    def drive(self):
        if self.start_left is None:
            return

        delta_left = self.left_encoder - self.start_left
        delta_right = self.right_encoder - self.start_right

        # Distance traveled per wheel
        left_dist = self.ticks_to_meters(delta_left, self.left_radius)
        right_dist = self.ticks_to_meters(delta_right, self.right_radius)

        # Average distance
        distance_traveled = (left_dist + right_dist) / 2.0
        self.get_logger().info(f"Distance traveled: {distance_traveled:.3f} m (L: {left_dist:.3f}, R: {right_dist:.3f})")

        msg = DutyCycles()
        if distance_traveled < self.target_distance:
            msg.duty_cycle_left = 0.5
            msg.duty_cycle_right = 0.5
        else:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.get_logger().info("Target distance reached!")
            self.timer.cancel()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveDistance(target_distance_m=1.0)  # drive 1 meter
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

