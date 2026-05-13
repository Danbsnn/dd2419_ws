#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node

from robp_interfaces.msg import ArmControl
from ikpy.chain import Chain


class ArmIK(Node):
    def __init__(self):
        super().__init__("arm_ik_node")

        self.pub = self.create_publisher(ArmControl, "/arm/control", 10)

        # IK chain
        self.chain = Chain.from_urdf_file(
            "snow_minimal.urdf",
            base_elements=["base_link"],
            active_links_mask=[False, False, False, True, True, True, True, True, False],
        )

        # Servo state
        self.current_servo = [40, 120, 30, 220, 180, 120]

        # Detect pose
        self.detect = [40, 120, 30, 190, 100, 120]

        # Target XYZ
        self.target_xyz = np.array([0.2, 0.0, 0.0])

        self.get_logger().info("IK Node Ready")

    # ----------------------------------------------------------
    def move_joints(self, target, time_ms=3000, label="move"):
        msg = ArmControl()
        msg.position = target
        msg.time = [int(time_ms)] * 6

        self.pub.publish(msg)

        self.get_logger().info(f"{label} -> {target} | {time_ms} ms")

        self.current_servo = target
        time.sleep(time_ms / 1000.0 + 0.5)

    # ----------------------------------------------------------
    def move_sequential(self, target, order, time_ms=800, label="seq"):
        for idx in order:
            intermediate = list(self.current_servo)
            intermediate[idx] = target[idx]

            msg = ArmControl()
            msg.position = intermediate
            msg.time = [int(time_ms)] * 6

            self.pub.publish(msg)

            self.get_logger().info(
                f"{label} -> joint[{idx}] -> {intermediate}"
            )

            self.current_servo = intermediate
            time.sleep(time_ms / 1000.0 + 0.3)

    # ----------------------------------------------------------
    def solve_ik(self, xyz):
        ik_solution = self.chain.inverse_kinematics(xyz)

        # Extract actuated joints (joint1 → joint5)
        joint_angles = ik_solution[3:8]

        # Convert to degrees
        joint_angles_deg = np.degrees(joint_angles)

        return joint_angles_deg

    # ----------------------------------------------------------
    def ik_to_servo(self, joints_deg):
        j1, j2, j3, j4, j5 = joints_deg

        servo = list(self.current_servo)

        # Keep gripper unchanged
        servo[0] = self.current_servo[0]

        # Mapping
        servo[5] = j1 + 120   # base
        servo[4] = j2 + 120                   # shoulder
        servo[3] = 120 - j3   # elbow
        servo[2] = j4 + 120   # wrist
        servo[1] = j5 + 120   # wrist rotate

        return servo

    # ----------------------------------------------------------
    def run(self):
        self.get_logger().info("Starting IK sequence")

        # Move to DETECT
        self.move_joints(self.detect, time_ms=1500, label="DETECT")

        # Solve IK
        joints_deg = self.solve_ik(self.target_xyz)
        self.get_logger().info(f"IK solution (deg): {joints_deg}")

        # Convert to servo
        target_servo = self.ik_to_servo(joints_deg)
        self.get_logger().info(f"Servo command: {target_servo}")

        # Joint order
        forward_order = [4, 3, 2, 1, 5]
        reverse_order = [5, 1, 2, 3, 4]

        # Move to target (sequential)
        self.move_sequential(target_servo, forward_order, time_ms=800, label="SEQ_FORWARD")

        # Return to detect (reverse)
        self.move_sequential(self.detect, reverse_order, time_ms=800, label="SEQ_RETURN")

        self.get_logger().info("IK motion complete")


# ----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ArmIK()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()