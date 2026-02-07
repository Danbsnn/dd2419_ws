#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from robp_interfaces.msg import ArmControl


class ArmPickup(Node):
    def __init__(self):
        super().__init__('arm_pick')
        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        self.t = 3000  # milliseconds per move

        # Predefined joint positions
        self.poses = {
            "HOME_OPEN": [30, 120, 90, 200, 120, 120],
            "BEND_OPEN": [30, 120, 90, 200,  70, 120],
            "GRIP_BENT": [105,120,90,200,  70, 120],
            "HOME_GRIP": [105, 120, 90, 200, 120, 120],
        }

    def move(self, name):
        msg = ArmControl()
        msg.position = [float(x) for x in self.poses[name]]
        msg.time = [self.t] * 6
        self.pub.publish(msg)
        self.get_logger().info(f"Moving → {name}")
        time.sleep(self.t / 1000.0)  # wait for motion to finish

    def run_sequence(self):
        self.get_logger().info("Starting pickup sequence")

        # Pickup sequence
        self.move("HOME_OPEN")
        self.move("BEND_OPEN")
        self.move("GRIP_BENT")
        self.move("HOME_GRIP")

        self.get_logger().info("Holding for 10 seconds")
        time.sleep(10)

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