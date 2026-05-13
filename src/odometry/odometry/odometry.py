#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from robp_interfaces.msg import Encoders
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class OdometryEKF(Node):
    """2D EKF using orientation from imu/data_raw to perform correction step"""

    def __init__(self):
        super().__init__('odometry')

        self.ticks_per_rev = float(48 * 64)
        self.wheel_radius = float(0.0492125)
        self.base = float(0.30)

        # ekf tuning
        self.q_xy_base = float(1e-4)
        self.q_xy_per_meter = float(5e-3)
        self.q_yaw_base = float(5e-4)
        self.q_yaw_per_rad = float(2e-2)

        # no covariance for yaw from imu
        self.imu_yaw_variance_default = float(5e-3)

        # stationary robot
        self.stationary_distance_threshold = float(1e-4)
        self.stationary_yaw_threshold = float(1e-4)

        self._tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(PoseStamped, '/odom_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.create_subscription(
            Encoders,
            '/phidgets/motor/encoders',
            self.encoder_callback,
            10
        )

        self.create_subscription(
            Imu,
            '/phidgets/imu/data_raw',
            self.imu_callback,
            50
        )

        # state x = [x, y, yaw]^T
        self.x = np.zeros((3, 1), dtype=float)

        # state covariance P.
        self.P = np.diag([1e-4, 1e-4, 1e-3]).astype(float)

        self.left_encoder = None
        self.right_encoder = None

        # (for publishing odom).
        self._last_time = None
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_yaw = 0.0

        self._current_imu_yaw = None
        self._imu_yaw_offset = None
        self._current_imu_yaw_variance = self.imu_yaw_variance_default

        # ONLY ADDED
        self._gyro_z = 0.0

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_pose(self):
        return float(self.x[0, 0]), float(self.x[1, 0]), float(self.x[2, 0])

    def imu_callback(self, msg: Imu):

        # ONLY ADDED
        self._gyro_z = msg.angular_velocity.z

        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]

        if abs(q[0]) < 1e-12 and abs(q[1]) < 1e-12 and abs(q[2]) < 1e-12 and abs(q[3]) < 1e-12:
            return

        (_, _, yaw) = euler_from_quaternion(q)
        yaw = -yaw  # otherwise imu is wrong direction

        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = yaw

        yaw = self.wrap_angle(yaw - self._imu_yaw_offset)
        self._current_imu_yaw = yaw

        self._current_imu_yaw_variance = 1e-3

    def ekf_predict(self, d: float, dtheta: float):

        x, y, yaw = self.get_pose()
        half_turn = yaw + 0.5 * dtheta

        # motion model
        x_pred = x + d * math.cos(half_turn)
        y_pred = y + d * math.sin(half_turn)
        yaw_pred = self.wrap_angle(yaw + dtheta)

        self.x[0, 0] = x_pred
        self.x[1, 0] = y_pred
        self.x[2, 0] = yaw_pred

        # jacobian
        F = np.array([
            [1.0, 0.0, -d * math.sin(half_turn)],
            [0.0, 1.0,  d * math.cos(half_turn)],
            [0.0, 0.0, 1.0],
        ], dtype=float)

        # process noise.
        q_xy = self.q_xy_base + self.q_xy_per_meter * abs(d)
        q_yaw = self.q_yaw_base + self.q_yaw_per_rad * abs(dtheta)

        Q = np.diag([q_xy, q_xy, q_yaw]).astype(float)

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def ekf_update_yaw(self, yaw_meas: float, yaw_variance: float):

        H = np.array([[0.0, 0.0, 1.0]], dtype=float)
        R = np.array([[max(yaw_variance, 1e-9)]], dtype=float)

        innovation = self.wrap_angle(
            yaw_meas - float(self.x[2, 0])
        )

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K * innovation
        self.x[2, 0] = self.wrap_angle(float(self.x[2, 0]))

        I = np.eye(3)

        self.P = (
            (I - K @ H) @ self.P @ (I - K @ H).T +
            K @ R @ K.T
        )

        self.P = 0.5 * (self.P + self.P.T)

    def encoder_callback(self, msg: Encoders):

        if self.left_encoder is None:
            self.left_encoder = msg.encoder_left
            self.right_encoder = msg.encoder_right
            return

        delta_ticks_left = msg.encoder_left - self.left_encoder
        delta_ticks_right = msg.encoder_right - self.right_encoder

        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        K = 2.0 * math.pi / self.ticks_per_rev

        ds_left = self.wheel_radius * K * delta_ticks_left
        ds_right = self.wheel_radius * K * delta_ticks_right

        d = 0.5 * (ds_right + ds_left)
        dtheta = (ds_right - ds_left) / self.base

        # ONLY ADDED
        stationary = (
            abs(d) < self.stationary_distance_threshold and
            abs(dtheta) < self.stationary_yaw_threshold and
            abs(self._gyro_z) < 0.006
        )

        # ONLY ADDED
        if not stationary:
            self.ekf_predict(d, dtheta)

        # Correct with IMU yaw only if robot is not stationary
        if self._current_imu_yaw is not None:
            moving = (
                abs(d) > self.stationary_distance_threshold or
                abs(dtheta) > self.stationary_yaw_threshold
            )

            if moving:
                self.ekf_update_yaw(
                    self._current_imu_yaw,
                    self._current_imu_yaw_variance
                )

        # LEFT EXACTLY AS YOUR CODE
        stamp = self.get_clock().now().to_msg()

        x, y, yaw = self.get_pose()

        self.broadcast_transform(stamp, x, y, yaw)
        self.publish_pose(stamp, x, y, yaw)
        self.publish_odometry(stamp)

    def broadcast_transform(self, stamp, x: float, y: float, yaw: float):

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

    def publish_pose(self, stamp, x: float, y: float, yaw: float):

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

        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        x, y, yaw = self.get_pose()

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Pose covariance
        pose_cov = [0.0] * 36

        pose_cov[0] = float(self.P[0, 0])
        pose_cov[1] = float(self.P[0, 1])
        pose_cov[5] = float(self.P[0, 2])

        pose_cov[6] = float(self.P[1, 0])
        pose_cov[7] = float(self.P[1, 1])
        pose_cov[11] = float(self.P[1, 2])

        pose_cov[30] = float(self.P[2, 0])
        pose_cov[31] = float(self.P[2, 1])
        pose_cov[35] = float(self.P[2, 2])

        odom.pose.covariance = pose_cov

        current_time = rclpy.time.Time.from_msg(stamp).nanoseconds / 1e9

        if self._last_time is None:

            self._last_time = current_time
            self._last_x = x
            self._last_y = y
            self._last_yaw = yaw

            self.odom_pub.publish(odom)
            return

        dt = current_time - self._last_time
        self._last_time = current_time

        if dt <= 0.0:
            return

        vx_world = (x - self._last_x) / dt
        vy_world = (y - self._last_y) / dt

        dyaw = self.wrap_angle(yaw - self._last_yaw)
        wz = dyaw / dt

        # Convert planar world velocity to base_link velocity.
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        vx_body = cy * vx_world + sy * vy_world
        vy_body = -sy * vx_world + cy * vy_world

        self._last_x = x
        self._last_y = y
        self._last_yaw = yaw

        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)


def main():

    rclpy.init()

    node = OdometryEKF()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()