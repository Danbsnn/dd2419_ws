#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class TestTransformPublisher(Node):
    def __init__(self, x: float, y: float, z: float):
        super().__init__('test_transform_publisher')

        self._x = x
        self._y = y
        self._z = z

        self._tf_broadcaster = TransformBroadcaster(self)

        # Publish at 10 Hz
        self._timer = self.create_timer(0.1, self.publish_transform)

        self.get_logger().info(
            f"Publishing test transform at x={x}, y={y}, z={z} relative to base_link"
        )

    def publish_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'object_detected/test'

        # Position from command line
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = self._z

        # Use parent's orientation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()

    # ----------------------------
    # Parse x, y, z from command line
    # ----------------------------
    if len(sys.argv) != 4:
        print("Usage: ros2 run <package> <node> x y z")
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    except ValueError:
        print("Coordinates must be numbers")
        return

    node = TestTransformPublisher(x, y, z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()