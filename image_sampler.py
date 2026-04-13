#!/usr/bin/env python3
"""
Simple image sampler — runs for 60 s, saves every N-th frame as JPEG.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time


class ImageSampler(Node):
    def __init__(self):
        super().__init__('image_sampler')

        self.declare_parameter('camera_topic',   '/arm/camera/image_raw')
        self.declare_parameter('output_dir',     os.path.expanduser('~/cube_images'))
        self.declare_parameter('sample_every_n', 90)   # save 1 in every 10 frames
        self.declare_parameter('duration_sec',   60)

        self.out_dir  = self.get_parameter('output_dir').value
        self.every_n  = self.get_parameter('sample_every_n').value
        self.duration = self.get_parameter('duration_sec').value
        topic         = self.get_parameter('camera_topic').value

        os.makedirs(self.out_dir, exist_ok=True)

        self.bridge     = CvBridge()
        self._frame     = 0
        self._saved     = 0
        self._start     = None

        self.sub = self.create_subscription(Image, topic, self._cb, 10)
        self.get_logger().info(f"Sampler started → {self.out_dir}")

    def _cb(self, msg):
        if self._start is None:
            self._start = time.monotonic()

        if time.monotonic() - self._start >= self.duration:
            self.get_logger().info(f"Done — {self._saved} images saved.")
            raise SystemExit

        self._frame += 1
        if self._frame % self.every_n != 0:
            return

        img   = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        ts    = msg.header.stamp
        fname = f"{ts.sec:010d}_{ts.nanosec:09d}.jpg"
        cv2.imwrite(os.path.join(self.out_dir, fname), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        self._saved += 1

        if self._saved % 10 == 0:
            elapsed = int(time.monotonic() - self._start)
            self.get_logger().info(
                f"{self._saved} saved | {self.duration - elapsed}s left")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSampler()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()