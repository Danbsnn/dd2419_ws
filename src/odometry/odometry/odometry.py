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

from nav_msgs.msg import Odometry

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        self._tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/odom_pose',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

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

        self._last_time = None
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_yaw = 0.0

        # Hardware states
        self.left_encoder = None
        self.right_encoder = None
        
        # Gyro integration variables
        self._last_imu_time = None
        self._gyro_delta_yaw = 0.0  # Accumulates rotation between encoder ticks

    def imu_callback(self, msg: Imu):
        """Integrates the Z-axis angular velocity to find the change in heading."""
        curr_time = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        
        if self._last_imu_time is None:
            self._last_imu_time = curr_time
            return
            
        dt = curr_time - self._last_imu_time
        self._last_imu_time = curr_time

        # Accumulate the change in yaw (radians per second * seconds)
        self._gyro_delta_yaw -= msg.angular_velocity.z * dt

    def encoder_callback(self, msg: Encoders):
        """Fuses encoder and gyro deltas to update the odometry."""
        ticks_per_rev = 48 * 64
        wheel_radius = 0.04921
        base = 0.3  # Measured on Snowwhite

        if self.left_encoder is None:
            self.left_encoder = msg.encoder_left
            self.right_encoder = msg.encoder_right
            return
        
        delta_ticks_left = msg.encoder_left - self.left_encoder
        delta_ticks_right = msg.encoder_right - self.right_encoder
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        K = 2 * math.pi / ticks_per_rev
        
        D = wheel_radius/2.0 * (K*delta_ticks_right + K*delta_ticks_left)
        delta_theta_enc = wheel_radius/base * (K*delta_ticks_right - K*delta_ticks_left)

        # complementary filter
        # 0.98 means we trust the gyro 98% for rotation, and encoders 2%
        alpha = 0
        delta_yaw_fused = alpha * self._gyro_delta_yaw + (1.0 - alpha) * delta_theta_enc
        
        self._gyro_delta_yaw = 0.0

        # update Yaw
        prev_yaw = self._yaw
        self._yaw += delta_yaw_fused
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        # angle averaging 
        avg_yaw = prev_yaw + 0.5 * math.atan2(
            math.sin(self._yaw - prev_yaw),
            math.cos(self._yaw - prev_yaw)
        )

        # update position
        self._x += D * np.cos(avg_yaw)
        self._y += D * np.sin(avg_yaw) 
        
        stamp = msg.header.stamp

        self.broadcast_transform(stamp, self._x, self._y, self._yaw)
        self.publish_pose(stamp, self._x, self._y, self._yaw)
        self.publish_odometry(stamp)

    def broadcast_transform(self, stamp, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._tf_broadcaster.sendTransform(t)

    def publish_pose(self, stamp, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01 

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pose_pub.publish(pose)

    def publish_odometry(self, stamp):
        odom = Odometry()
    
        # Header
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
    
        # Pose
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
    
        q = quaternion_from_euler(0.0, 0.0, self._yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
    
        # twist
        current_time = rclpy.time.Time.from_msg(stamp).nanoseconds / 1e9
    
        if self._last_time is None:
            self._last_time = current_time
            self.odom_pub.publish(odom)
            return
    
        dt = current_time - self._last_time
        self._last_time = current_time
    
        if dt <= 0:
            return
    
        # Linear velocity (in odom frame)
        vx = (self._x - self._last_x) / dt
        vy = (self._y - self._last_y) / dt
    
        # Angular velocity
        dyaw = math.atan2(
            math.sin(self._yaw - self._last_yaw),
            math.cos(self._yaw - self._last_yaw)
        )
        wz = dyaw / dt
    
        # Save last state
        self._last_x = self._x
        self._last_y = self._y
        self._last_yaw = self._yaw
    
        # Fill twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
    
        self.odom_pub.publish(odom)

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
