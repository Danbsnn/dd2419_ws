#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped, PoseStamped
from robp_interfaces.msg import Encoders
from sensor_msgs.msg import Imu


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry_comp')

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/odom_pose',
            10
        )

        # Subscribe to encoder topic and call callback function on each recieved message
        self.create_subscription(
            Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10)

        self.create_subscription(
            Imu, 
            '/phidgets/imu/data_raw',
            self.imu_callback, 
            10
        )
        
        # 2D pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        
        self.gain = 0.9

        # IMU variables
        self._current_imu_yaw = None
        self._imu_yaw_offset = None

        self.left_encoder = None
        self.right_encoder = None

        self.get_logger().info("Odometry running...")

    def imu_callback(self, msg: Imu):
        """Updates the current yaw based on IMU data."""
        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        (_, _, yaw) = euler_from_quaternion(q)
        yaw = -yaw

        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = yaw

        yaw = yaw - self._imu_yaw_offset
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self._current_imu_yaw = yaw

    def encoder_callback(self, msg: Encoders):
        """Takes encoder readings and updates the odometry.

        This function is called every time the encoders are updated (i.e., when a message is published on the '/motor/encoders' topic).

        Your task is to update the odometry based on the encoder data in 'msg'. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- An encoders ROS message. To see more information about it 
        run 'ros2 interface show robp_interfaces/msg/Encoders' in a terminal.
        """

        # The kinematic parameters for the differential configuration
        ticks_per_rev = 48 * 64
        wheel_radius = 0.04921
        base = 0.3  # Measured on Snowwhite

        # Ticks since last message
        if self.left_encoder is None:
            self.left_encoder = msg.encoder_left
            self.right_encoder = msg.encoder_right
            return
        
        delta_ticks_left = msg.encoder_left - self.left_encoder
        delta_ticks_right = msg.encoder_right - self.right_encoder
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right


        K = 2 * math.pi / ticks_per_rev
        D = wheel_radius/2 * (K*delta_ticks_right + K*delta_ticks_left)

        if self._current_imu_yaw is None:
            return

        delta_theta = wheel_radius/base * (K*delta_ticks_right - K*delta_ticks_left)
        yaw_pred = self._yaw + delta_theta

        prev_yaw = self._yaw
        yaw_error = self._current_imu_yaw - yaw_pred
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        if abs(D) > 0.001 or abs(delta_theta) > 0.001: 
            # self._yaw = yaw_pred + self.gain * yaw_error
            self._yaw = self._current_imu_yaw
        else:
            # stationary → ignore IMU, trust encoders
            self._yaw = yaw_pred
            
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
        avg_yaw = (self._yaw + prev_yaw) / 2.0
        self._x = self._x + D * np.cos(avg_yaw)
        self._y = self._y + D * np.sin(avg_yaw) 
        
        # stamp = msg.header.stamp
        stamp = self.get_clock().now().to_msg()

        self.broadcast_transform(stamp, self._x, self._y, self._yaw)
        self.publish_pose(stamp, self._x, self._y, self._yaw)


    def broadcast_transform(self, stamp, x, y, yaw):
        """Takes a 2D pose and broadcasts it as a ROS transform.

        Broadcasts a 3D transform with z, roll, and pitch all zero. 
        The transform is stamped with the current time and is between the frames 'odom' -> 'base_link'.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)

    def publish_pose(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pose_pub.publish(pose)   



def main():
    rclpy.init()
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()