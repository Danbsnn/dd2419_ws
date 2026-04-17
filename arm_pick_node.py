#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from robp_interfaces.msg import ArmControl


class ArmPickup(Node):
    def __init__(self):
        super().__init__('arm_pick')

        # Publisher
        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        # Joint poses
        self.poses = {
            "HOME":    [40, 120, 30, 220, 180, 120],
            "PICK":    [110, 120, 95, 205, 70, 120],
            "PICK_UP": [110, 120, 95, 205, 120, 120],
            "DROP":    [60, 120, 95, 205, 120, 120],
        }

        self.current_position = list(self.poses["HOME"])

        self.time_per_degree = 20
        self.min_time = 500

    def move(self, name):
        """Move arm to a named pose with smooth timing."""

        target = [float(x) for x in self.poses[name]]

        max_delta = max(
            abs(t - c) for t, c in zip(target, self.current_position)
        )

        move_time = max(int(max_delta * self.time_per_degree), self.min_time)

        msg = ArmControl()
        msg.position = target
        msg.time = [move_time] * 6

        self.pub.publish(msg)
        self.get_logger().info(f"Moving -> {name} over {move_time} ms")

        self.current_position = target

        time.sleep(move_time / 1000.0 + 0.5)

    def run_sequence(self):
        """Run full pickup sequence."""

        self.get_logger().info("Starting pickup sequence")

        self.move("HOME")
        self.get_logger().info("Waiting at home")
        time.sleep(5)
        self.move("PICK")

        self.get_logger().info("Waiting at pick")
        time.sleep(5)

        self.move("PICK_UP")
        self.get_logger().info("Waiting at pickup")
        time.sleep(5)
        

       

        


def main(args=None):
    rclpy.init(args=args)
    node = ArmPickup()
    node.run_sequence()


if __name__ == "__main__":
    main()
