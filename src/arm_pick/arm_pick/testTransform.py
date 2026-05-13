#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException

class TestTransformPublisher(Node):
    def __init__(self):
        super().__init__('test_transform_publisher')
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(0.1, self.publish_transform)

    def publish_transform(self):
        try:
            # Get cube position relative to base_link
            t = self._tf_buffer.lookup_transform(
                'base_link',
                'blue_cube',
                rclpy.time.Time()
            )
        except TransformException:
            return

        # Replace the cube's orientation with the parent frame's (base_link) identity
        # i.e. no rotation relative to base_link → aligned with base_link axes
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        t.child_frame_id = t.child_frame_id + '1'
        self._tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TestTransformPublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()