#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster,Buffer, TransformListener, TransformException
from tf_transformations import quaternion_from_euler

class TestTransformPublisher(Node):
    def __init__(self):
        super().__init__('test_transform_publisher')

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener =TransformListener(self._tf_buffer,self)

        # Publish at 10 Hz
        self._timer = self.create_timer(0.1, self.publish_transform)

    def publish_transform(self):

        try:
            t = self._tf_buffer.lookup_transform(
                'base_link',
                'cube_detected',
                rclpy.time.Time()
            ) 


        except TransformException:
            return



        t.child_frame_id = t.child_frame_id+'1'



        self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TestTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()