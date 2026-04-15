#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class FirstGoalNode(Node):
    def __init__(self):
        super().__init__('first_goal_node')
        self.get_logger().info("FirstGoalNode started")

        # Subscriber
        self.sub = self.create_subscription(
            PoseStamped,
            '/detected_object',
            self.object_callback,
            10
        )

        # Publisher
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish TF continuously
        self.create_timer(0.1, self.publish_camera_tf)

        self.goal_set = False

    # -----------------------------
    def publish_camera_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "realsense_camera_color_optical_frame"

        # ⚠️ SET THESE TO YOUR REAL CAMERA POSITION
        t.transform.translation.x = 0.5
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0

        # Identity rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    # -----------------------------
    def object_callback(self, msg: PoseStamped):
        if not self.goal_set:
            # ⚠️ Force goal into map frame (works if TF is correct)
            msg.header.frame_id = "map"

            self.goal_pub.publish(msg)
            self.goal_set = True

            self.get_logger().info(
                f"First goal set at X={msg.pose.position.x:.2f}, "
                f"Y={msg.pose.position.y:.2f}, "
                f"Z={msg.pose.position.z:.2f}"
            )


def main():
    rclpy.init()
    node = FirstGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
