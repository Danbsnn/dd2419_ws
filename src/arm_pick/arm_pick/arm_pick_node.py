#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from robp_interfaces.msg import ArmControl


class ArmPickup(Node):
    def __init__(self):
        super().__init__('arm_pick')

        # Publisher for arm control
        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        # Predefined joint positions
        self.poses = {
            "HOME_OPEN": [30, 120, 90, 200, 120, 120],
            "BEND_OPEN": [30, 120, 90, 200,  70, 120],
            "GRIP_BENT": [120,120,90,200,  70, 120],
            "HOME_GRIP": [120, 120, 90, 200, 120, 120],
        }

        # Assume starting at HOME_OPEN
        self.current_position = self.poses["HOME_OPEN"]

        # Scaling factor: milliseconds per degree of joint movement
        self.time_per_degree = 20  # adjust as needed

        # Minimum movement time (ms)
        self.min_time = 500

    def move(self, name):
        """Move arm to a named pose with smooth transition time."""
        target = [float(x) for x in self.poses[name]]

        # Calculate largest joint displacement
        max_delta = max(abs(t - c) for t, c in zip(target, self.current_position))

        # Scale time based on max_delta
        move_time = max(int(max_delta * self.time_per_degree), self.min_time)

        # Prepare message
        msg = ArmControl()
        msg.position = target
        msg.time = [move_time] * 6

        # Publish command
        self.pub.publish(msg)
        self.get_logger().info(f"Moving → {name} over {move_time} ms")

        # Update current position
        self.current_position = target

        # Wait for motion to complete
        time.sleep(move_time / 1000.0)

    def run_sequence(self):
        """Run full pickup sequence."""
        self.get_logger().info("Starting pickup sequence")

        # Pickup sequence
        self.move("HOME_OPEN")
        self.move("BEND_OPEN")
        self.move("GRIP_BENT")
        self.move("HOME_GRIP")

        # Hold at HOME_GRIP for 10 seconds
        self.get_logger().info("Holding for 10 seconds")
        time.sleep(10)

        # Return sequence
        self.move("GRIP_BENT")
        self.move("BEND_OPEN")
        self.move("HOME_OPEN")

        self.get_logger().info("Sequence complete. Shutting down.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmPickup()
    node.run_sequence()


if __name__ == "__main__":
    main()