#!/usr/bin/env python3

import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from robp_interfaces.msg import ArmControl
from tf2_ros import Buffer, TransformListener, TransformException

class ArmDrop(Node):

    def __init__(self):
        super().__init__('arm_drop')
        self.state = "WAIT_FOR_DROP"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to check drop position
        self.timer = self.create_timer(1.0, self.extract_drop_position)

        # Theta1 origin w.r.t base_link
        self.x_origin_theta1 = -0.00450
        self.y_origin_theta1 = -0.04750
        self.z_origin_theta1 =  0.12915

        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        # Arm dimensions
        self.l1 = 0.10048
        self.l2 = 0.094714
        self.l3 = 0.05071 + 0.11260

        self.time_per_degree = 20
        self.min_time = 1000

        # Home position
        self.home_motors = [115, 120, 120, 120, 120, 120]
        self.current_motors = self.home_motors.copy()

        # Gripper positions
        self.GRIP_OPEN = 60
        self.GRIP_CLOSED = 120

    # -----------------------------
    # Send motor command
    # -----------------------------
    def send_motor_command(self, motors):
        max_delta = max(abs(a - b) for a, b in zip(motors, self.current_motors))
        move_time = max(int(max_delta * self.time_per_degree), self.min_time)

        msg = ArmControl()
        msg.position = motors
        msg.time = [move_time] * 6

        self.pub.publish(msg)
        time.sleep(move_time / 1000.0)
        self.current_motors = motors.copy()

    # -----------------------------
    # Inverse kinematics
    # -----------------------------
    def inverse_kinematics(self, x, y, z):
        theta_base = math.atan2(y, x)
        theta1_deg = 30.0
        theta1 = math.radians(theta1_deg)
        rho = math.sqrt(x*x + y*y)
        beta = math.atan2(z, rho)
        phi = theta1 - beta
        d = math.sqrt(rho*rho + z*z)
        r = math.sqrt(self.l1*self.l1 + d*d - 2*self.l1*d*math.cos(phi))
        c3 = (r*r - self.l2*self.l2 - self.l3*self.l3) / (2*self.l2*self.l3)
        c3 = max(-1.0, min(1.0, c3))
        theta3 = -math.acos(c3)
        A = math.acos((self.l1*self.l1 + r*r - d*d) / (2*self.l1*r))
        B = math.acos((self.l2*self.l2 + r*r - self.l3*self.l3) / (2*self.l2*r))
        elbow_angle = A + B
        theta2_deg = 180 - math.degrees(elbow_angle)
        theta2 = -math.radians(theta2_deg)
        return theta_base, theta1, theta2, theta3

    # -----------------------------
    # Motor conversion
    # -----------------------------
    def to_motor_angles(self, tb, t1, t2, t3):
        d = np.degrees
        motors = self.current_motors.copy()
        motors[5] = d(tb) + 120
        motors[4] = 4*d(t1)/3
        motors[3] = -d(t2) + 120
        motors[2] = d(t3) + 120
        return motors

    # -----------------------------
    # Sequential move
    # -----------------------------
    def move_sequential(self, target, order):
        motors = self.current_motors.copy()
        for idx in order:
            motors[idx] = target[idx]
            self.send_motor_command(motors)

    # -----------------------------
    # Move to xyz
    # -----------------------------
    def move_to_xyz(self, x, y, z):
        self.get_logger().info("Moving arm to drop location.")
        tb, t1, t2, t3 = self.inverse_kinematics(x, y, z)
        target_motors = self.to_motor_angles(tb, t1, t2, t3)
        self.move_sequential(target_motors, [5, 4, 2, 3])
        time.sleep(1)

        # Open gripper to drop
        self.get_logger().info("Releasing object.")
        open_gripper = target_motors.copy()
        open_gripper[0] = self.GRIP_OPEN
        self.send_motor_command(open_gripper)

        time.sleep(1)

        # Return home
        self.get_logger().info("Returning home.")
        self.move_sequential(self.home_motors, [3, 2, 4, 5])
        self.state = "DONE"
        self.get_logger().info("Drop sequence complete. Destroying node.")
        self.destroy_node()
        rclpy.shutdown()
        

    # -----------------------------
    # TF lookup
    # -----------------------------
    def extract_drop_position(self):
        if self.state != "WAIT_FOR_DROP":
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'object_detected/test',
                rclpy.time.Time()
            )
        except TransformException:
            return

        x_base = transform.transform.translation.x
        y_base = transform.transform.translation.y
        z_base = transform.transform.translation.z

        x_theta1 = x_base - self.x_origin_theta1
        y_theta1 = y_base - self.y_origin_theta1
        z_theta1 = z_base - self.z_origin_theta1

        self.get_logger().info(f"Drop target detected at x={x_theta1}, y={y_theta1}, z={z_theta1}")
        self.state = "MOVE_TO_DROP"
        self.timer.cancel()
        self.move_to_xyz(x_theta1, y_theta1, z_theta1)

# -----------------------------
# Main loop
# -----------------------------
def main():
    rclpy.init()
    node = ArmDrop()
    node.send_motor_command(node.home_motors)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()