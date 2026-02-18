#!/usr/bin/env python3

import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from robp_interfaces.msg import ArmControl


class ArmPickup(Node):

    def __init__(self):
        super().__init__('arm_pick')

        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        self.l1 = 9.8
        self.l2 = 9.5
        self.l3 = 5.1

        self.time_per_degree = 20
        self.min_time = 500

        self.home_motors = [30, 120, 120, 120, 120, 120]
        self.current_motors = self.home_motors.copy()


    # -------------------------------------------------
    # DETAILED SAFETY CHECK
    # -------------------------------------------------
    def check_bounds(self, x, y, z, alpha_deg):

        errors = []

        alpha_rad = math.radians(alpha_deg)
        lower_z = -18- 12 * math.sin(alpha_rad)

        if not (14 < x < 19):
            errors.append("x must be between 14 and 19")

        if not (-12 < y < 12):
            errors.append("y must be between -12 and 12")

        if not (lower_z < z < 12):
            errors.append(f"z must be between {lower_z:.2f} and 12")

        if not (-90 < alpha_deg < 90):
            errors.append("alpha must be between -90 and 90 degrees")

        return errors


    # -------------------------------------------------
    # SEND MOTOR COMMAND
    # -------------------------------------------------
    def send_motor_command(self, motors):

        max_delta = max(abs(a - b)
                        for a, b in zip(motors, self.current_motors))

        move_time = max(int(max_delta * self.time_per_degree),
                        self.min_time)

        msg = ArmControl()
        msg.position = motors
        msg.time = [move_time] * 6

        self.pub.publish(msg)
        time.sleep(move_time / 1000.0)

        self.current_motors = motors.copy()


    # -------------------------------------------------
    # INVERSE KINEMATICS
    # -------------------------------------------------
    def inverse_kinematics(self, x, y, z, alpha):

        theta_base = math.atan2(y, x)

        m = x - self.l3 * math.cos(alpha)
        n = z - self.l3 * math.sin(alpha)

        r = math.sqrt(m*m + n*n)

        c2 = (r*r - self.l1*self.l1 - self.l2*self.l2) / (2*self.l1*self.l2)
        c2 = max(-1.0, min(1.0, c2))

        theta2 = -abs(math.acos(c2))

        phi = math.atan2(n, m)

        beta = math.atan2(
            self.l2 * math.sin(theta2),
            self.l1 + self.l2 * math.cos(theta2)
        )

        theta1 = phi - beta
        theta3 = alpha - (theta1 + theta2)

        return theta_base, theta1, theta2, theta3


    # -------------------------------------------------
    # MOTOR CONVERSION
    # -------------------------------------------------
    def to_motor_angles(self, tb, t1, t2, t3):

        d = np.degrees
        motors = self.current_motors.copy()

        motors[5] = d(tb) + 120
        motors[4] = 4 * d(t1) / 3
        motors[3] = -d(t2) + 120
        motors[2] = d(t3) + 120

        return motors


    # -------------------------------------------------
    # SEQUENTIAL SAFE MOTION
    # -------------------------------------------------
    def move_sequential(self, target, order):

        motors = self.current_motors.copy()

        for idx in order:
            motors[idx] = target[idx]
            self.send_motor_command(motors)


    # -------------------------------------------------
    # MOVE FUNCTION
    # -------------------------------------------------
    def move_to_xyz(self, x, y, z, alpha_deg):

        errors = self.check_bounds(x, y, z, alpha_deg)

        if errors:
            self.get_logger().error("Points outside bounds:")
            for err in errors:
                self.get_logger().error(f"   - {err}")
            return

        self.get_logger().info("Input valid. Moving arm.")

        alpha = math.radians(alpha_deg)

        tb, t1, t2, t3 = self.inverse_kinematics(x, y, z, alpha)

        target_motors = self.to_motor_angles(tb, t1, t2, t3)

        # Move to target
        self.move_sequential(target_motors, [5, 4, 2, 3])

        time.sleep(10)

        # Return home
        self.move_sequential(self.home_motors, [3, 2, 4, 5])


# -------------------------------------------------
# MAIN LOOP
# -------------------------------------------------
def main():

    rclpy.init()
    node = ArmPickup()

    node.send_motor_command(node.home_motors)

    try:
        while True:

            user_input = input(
                "\nEnter x y z alpha_deg (or q to quit): "
            )

            if user_input.lower() == 'q':
                break

            try:
                x, y, z, alpha_deg = map(float, user_input.split())
                node.move_to_xyz(x, y, z, alpha_deg)
            except:
                print("Invalid input format. Use: x y z alpha_deg")

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()