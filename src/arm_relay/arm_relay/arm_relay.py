#!/usr/bin/env python3

import math
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from robp_interfaces.msg import ArmControl


class ArmCommandRelay(Node):
    def __init__(self):
        super().__init__("arm_command_relay")

        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "r_joint",
        ]

        self.rate_limit_sec = 0.1
        self.command_time_ms = 100

        self.startup_command = True
        self.startup_time_ms = 1500

        self.last_send_time = 0.0

        self.control_pub = self.create_publisher(
            ArmControl,
            "/arm/control",
            10,
        )

        self.create_subscription(
            DynamicJointState,
            "/joint_state_broadcaster/dynamic_joint_states",
            self.dynamic_joint_state_callback,
            10,
        )

        self.get_logger().info("Arm command relay started")

    def get_position_map_from_dynamic_msg(self, msg: DynamicJointState) -> Dict[str, float]:
        positions = {}

        for joint_name, interface_value in zip(msg.joint_names, msg.interface_values):
            if "position" not in interface_value.interface_names:
                continue

            idx = interface_value.interface_names.index("position")
            positions[joint_name] = interface_value.values[idx]

        return positions

    def joints_rad_to_motors_deg(self, joints_rad: Dict[str, float]) -> List[float]:
        j1 = math.degrees(joints_rad["joint1"])
        j2 = math.degrees(joints_rad["joint2"])
        j3 = math.degrees(joints_rad["joint3"])
        j4 = math.degrees(joints_rad["joint4"])
        j5 = math.degrees(joints_rad["joint5"])
        r = math.degrees(joints_rad["r_joint"])

        return [
            self.clamp(2.0 * (r + 84.0), 0.0, 240.0),
            self.clamp(j5 + 120.0, 0.0, 240.0),
            self.clamp(j4 + 120.0, 0.0, 240.0),
            self.clamp(120.0 - j3, 0.0, 240.0),
            self.clamp(j2 + 120.0, 0.0, 240.0),
            self.clamp(j1 + 120.0, 0.0, 240.0),
        ]

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def dynamic_joint_state_callback(self, msg: DynamicJointState):
        now = time.time()

        if not self.startup_command:
            if now - self.last_send_time < self.rate_limit_sec:
                return

        joint_positions = self.get_position_map_from_dynamic_msg(msg)

        missing = [name for name in self.joint_names if name not in joint_positions]
        if missing:
            return

        target_motors = self.joints_rad_to_motors_deg(joint_positions)

        control_msg = ArmControl()
        control_msg.position = target_motors

        if self.startup_command:
            control_msg.time = [self.startup_time_ms] * 6
            self.startup_command = False
        else:
            control_msg.time = [self.command_time_ms] * 6

        self.control_pub.publish(control_msg)
        self.last_send_time = now



def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
#!/usr/bin/env python3

import math
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from robp_interfaces.msg import ArmControl


class ArmCommandRelay(Node):
    def __init__(self):
        super().__init__("arm_command_relay")

        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "r_joint",
        ]

        self.rate_limit_sec = 0.1
        self.command_time_ms = 100

        self.startup_command = True
        self.startup_time_ms = 1500

        self.last_send_time = 0.0

        self.control_pub = self.create_publisher(
            ArmControl,
            "/arm/control",
            10,
        )

        self.create_subscription(
            DynamicJointState,
            "/joint_state_broadcaster/dynamic_joint_states",
            self.dynamic_joint_state_callback,
            10,
        )

        self.get_logger().info("Arm command relay started")

    def get_position_map_from_dynamic_msg(self, msg: DynamicJointState) -> Dict[str, float]:
        positions = {}

        for joint_name, interface_value in zip(msg.joint_names, msg.interface_values):
            if "position" not in interface_value.interface_names:
                continue

            idx = interface_value.interface_names.index("position")
            positions[joint_name] = interface_value.values[idx]

        return positions

    def joints_rad_to_motors_deg(self, joints_rad: Dict[str, float]) -> List[float]:
        j1 = math.degrees(joints_rad["joint1"])
        j2 = math.degrees(joints_rad["joint2"])
        j3 = math.degrees(joints_rad["joint3"])
        j4 = math.degrees(joints_rad["joint4"])
        j5 = math.degrees(joints_rad["joint5"])
        r = math.degrees(joints_rad["r_joint"])

        return [
            self.clamp(2.0 * (r + 84.0), 0.0, 240.0),
            self.clamp(j5 + 120.0, 0.0, 240.0),
            self.clamp(j4 + 120.0, 0.0, 240.0),
            self.clamp(120.0 - j3, 0.0, 240.0),
            self.clamp(j2 + 120.0, 0.0, 240.0),
            self.clamp(j1 + 120.0, 0.0, 240.0),
        ]

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def dynamic_joint_state_callback(self, msg: DynamicJointState):
        now = time.time()

        if not self.startup_command:
            if now - self.last_send_time < self.rate_limit_sec:
                return

        joint_positions = self.get_position_map_from_dynamic_msg(msg)

        missing = [name for name in self.joint_names if name not in joint_positions]
        if missing:
            return

        target_motors = self.joints_rad_to_motors_deg(joint_positions)

        control_msg = ArmControl()
        control_msg.position = target_motors

        if self.startup_command:
            control_msg.time = [self.startup_time_ms] * 6
            self.startup_command = False
        else:
            control_msg.time = [self.command_time_ms] * 6

        self.control_pub.publish(control_msg)
        self.last_send_time = now



def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()