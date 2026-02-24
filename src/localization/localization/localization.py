#!/usr/bin/env python

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class Localization(Node):

    def __init__(self):
        super().__init__('localization')

        # Subscribe to odometry path
        """self.create_subscription(
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
        )"""

        # Broadcast transform for map to odom
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        self.get_logger().info("Localization running")


    
    # def path_callback(self, msg):
    #     """Publishes latest odometry pose in the map frame"""
    #     if not msg.poses:
    #         self.get_logger().info("No path available from odom")
    #         return
        
    #     latest_pose = msg.poses[-1]  # geometry_msgs/PoseStamped[] poses
        
    #     localized_pose = PoseStamped()
    #     localized_pose.header.stamp = latest_pose.header.stamp
    #     localized_pose.header.frame_id = "map"
    #     localized_pose.pose = latest_pose.pose

    #     self.broadcast_transform(latest_pose.header.stamp)

    #     # self.get_logger().info(f"Localized Pose")
    #     self.pose_pub.publish(localized_pose)


    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        t.transform.translation.x = 0.49
        t.transform.translation.y = 0.50
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
