#!/usr/bin/env python3

import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from robp_interfaces.msg import ArmControl
from tf2_ros import Buffer, TransformListener, TransformException

class ArmPickup(Node):

    def __init__(self):
        
        super().__init__('arm_pick')
        self.state = "WAIT_FOR_OBJECT"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically check object
        self.timer = self.create_timer(1.0, self.object_extract_position)
        #position of theta1 origin with respect to the base link
        self.x_origin_theta1 = -0.00450
        self.y_origin_theta1 = -0.04750
        self.z_origin_theta1 =  0.12915

        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        self.l1 = 0.10048
        self.l2 = 0.094714
        self.l3 = 0.05071 + 0.11260

        self.time_per_degree = 20
        self.min_time = 1000

        self.home_motors = [30, 120, 120, 120, 120, 120]
        self.current_motors = self.home_motors.copy()



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
    def inverse_kinematics(self, x, y, z):

        # -----------------------------
        # Base rotation
        # -----------------------------
        theta_base = math.atan2(y, x)

        # -----------------------------
        # Fixed shoulder reference
        # -----------------------------
        theta1_deg = 30.0
        theta1 = math.radians(theta1_deg)

        # -----------------------------
        # Cylindrical projection
        # -----------------------------
        rho = math.sqrt(x*x + y*y)

        beta = math.atan2(z, rho)

        phi = theta1 - beta

        # -----------------------------
        # Distance shoulder -> EE
        # -----------------------------
        d = math.sqrt(rho*rho + z*z)

        # -----------------------------
        # Triangle side r
        # -----------------------------
        r = math.sqrt(
            self.l1*self.l1 +
            d*d -
            2*self.l1*d*math.cos(phi)
        )

        # -----------------------------
        # Elbow angle theta3
        # -----------------------------
        c3 = (r*r - self.l2*self.l2 - self.l3*self.l3) / (2*self.l2*self.l3)

        c3 = max(-1.0, min(1.0, c3))

        # NEGATIVE solution (arm down)
        theta3 = -math.acos(c3)

        # -----------------------------
        # Angles A and B
        # -----------------------------
        A = math.acos(
            (self.l1*self.l1 + r*r - d*d) /
            (2*self.l1*r)
        )

        B = math.acos(
            (self.l2*self.l2 + r*r - self.l3*self.l3) /
            (2*self.l2*r)
        )

        # Elbow angle
        elbow_angle = A + B

        theta2_deg = 180 - math.degrees(elbow_angle)

        # Always go downward
        theta2 = -math.radians(theta2_deg)

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
    def move_to_xyz(self, x, y, z):
        self.get_logger().info("Moving arm.")

        tb, t1, t2, t3 = self.inverse_kinematics(x, y, z)

        target_motors = self.to_motor_angles(tb, t1, t2, t3)

        # Move to target
        self.move_sequential(target_motors, [5, 4, 2, 3])

        time.sleep(10)

        # Return home
        self.move_sequential(self.home_motors, [3, 2, 4, 5])
    # -------------------------------------------------
    # OBJECT POSITION EXTRACTION + STATE MACHINE
    # -------------------------------------------------
    def object_extract_position(self):

        if self.state != "WAIT_FOR_OBJECT":
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

        # Convert to theta1 frame
        x_theta1 = x_base - self.x_origin_theta1
        y_theta1 = y_base - self.y_origin_theta1
        z_theta1 = z_base - self.z_origin_theta1

        self.get_logger().info("Object detected. Moving to object.")

        self.state = "MOVE_TO_OBJECT"
        #To stop repeated triggering after detecting the object
        self.timer.cancel()
        # Execute pick sequence
        self.execute_pick(x_theta1, y_theta1, z_theta1)

    # -------------------------------------------------
    # PICK SEQUENCE
    # -------------------------------------------------
    def execute_pick(self, x, y, z):

        # -----------------------
        # Move to object
        # -----------------------
        tb, t1, t2, t3 = self.inverse_kinematics(x, y, z)
        target_motors = self.to_motor_angles(tb, t1, t2, t3)

        self.move_sequential(target_motors, [5, 4, 2, 3])

        time.sleep(2)

        # -----------------------
        # Close gripper
        # -----------------------
        self.get_logger().info("Gripping object.")

        grip_motors = target_motors.copy()
        grip_motors[0] = 115   # gripper motor index

        self.send_motor_command(grip_motors)

        time.sleep(2)

        # -----------------------
        # Return home (while gripping)
        # -----------------------
        self.get_logger().info("Returning home with object.")

        home_with_object = self.home_motors.copy()
        home_with_object[0] = 115  # keep gripper closed

        self.move_sequential(home_with_object, [3, 2, 4, 5])

        self.state = "DONE"

        self.get_logger().info("Pick sequence complete. Destroying node.")
        self.destroy_node()
        rclpy.shutdown()


# -------------------------------------------------
# MAIN LOOP
# -------------------------------------------------
def main():
    rclpy.init()
    node = ArmPickup()

    node.send_motor_command(node.home_motors)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()