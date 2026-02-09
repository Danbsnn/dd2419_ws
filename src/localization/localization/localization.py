#!/usr/bin/env python

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class Localization(Node):

    def __init__(self):
        super().__init__('localization')

        # Subscribe to odometry path
        self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )

        # Publish localized pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/localized_pose',
            10
        )

        # Broadcast transform for map to odom
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Localization running")



    def path_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().info("No path available from odom") 
        
        latest_pose = msg.poses[-1].pose  # geometry_msgs/PoseStamped[] poses
        

def main():
    rclpy.init()
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()