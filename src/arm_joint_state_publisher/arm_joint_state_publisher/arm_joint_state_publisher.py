#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robp_interfaces.msg import ArmFeedback


class ArmJointStatePublisher(Node):
    def __init__(self):
        super().__init__('arm_joint_state_publisher')

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.create_subscription(
            ArmFeedback,  # change this to the correct message type
            '/arm/feedback',
            self.feedback_callback,
            10
        )

    def feedback_callback(self, msg):
        pos = msg.position  # the 6-element array

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'r_joint',
        ]

        # joints 1-5: motor 0-240 deg, center=120 deg = 0 rad
        # sign can be flipped per joint once tested in RViz2
        joint1 = math.radians(pos[5] - 120)   # base rotation
        joint2 = math.radians(pos[4] - 120)
        joint3 = math.radians(-pos[3] + 120)
        joint4 = math.radians(pos[2]-120 )
        joint5 = math.radians(pos[1]-120)   # gripper rotation

        # gripper: 0-160 deg, 30=closed, 115=open
        # mapped so that 0 rad = open (115 deg)
        r_joint = math.radians(pos[0]-80)

        js.position = [joint1, joint2, joint3, joint4, joint5, r_joint]

        self.joint_state_pub.publish(js)


def main():
    rclpy.init()
    node = ArmJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()