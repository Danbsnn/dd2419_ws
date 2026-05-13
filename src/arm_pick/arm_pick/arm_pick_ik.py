#!/usr/bin/env python3

import time
import numpy as np
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import ArmControl
from tf2_ros import Buffer, TransformListener, TransformException


class SnowArmIK:
    def __init__(self):
        # Approximate kinematic dimensions from URDF, in metres
        self.z1 = 0.028505
        self.z2 = 0.036100
        self.l3 = 0.100480
        self.l4 = 0.094714
        self.l5 = 0.050710
        self.tool = 0.112600

        # Joint limits [joint1..joint5] in radians
        self.lower = np.array([-2.09, -1.57, -2.09, -2.09, -2.09], dtype=float)
        self.upper = np.array([2.09, 1.57, 2.09, 2.09, 2.09], dtype=float)

        self.q_seed = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

    @staticmethod
    def rot_x(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s,  c]
        ], dtype=float)

    @staticmethod
    def rot_z(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ], dtype=float)

    @staticmethod
    def make_T(R, p):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        return T

    @staticmethod
    def trans(x=0.0, y=0.0, z=0.0):
        T = np.eye(4)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    def fk(self, q):
        q1, q2, q3, q4, q5 = q

        T = np.eye(4)

        T = T @ self.trans(z=self.z1)
        T = T @ self.make_T(self.rot_z(q1), np.zeros(3))

        T = T @ self.trans(z=self.z2)
        T = T @ self.make_T(self.rot_x(q2), np.zeros(3))

        T = T @ self.trans(z=self.l3)
        T = T @ self.make_T(self.rot_x(q3), np.zeros(3))

        T = T @ self.trans(z=self.l4)
        T = T @ self.make_T(self.rot_x(q4), np.zeros(3))

        T = T @ self.trans(z=self.l5)
        T = T @ self.make_T(self.rot_z(q5), np.zeros(3))

        T = T @ self.trans(z=self.tool)

        return T

    def fk_position(self, q):
        return self.fk(q)[:3, 3]

    def ik_position(self, target_xyz, seed=None, keep_joint5_zero=True):
        target_xyz = np.array(target_xyz, dtype=float)

        if seed is None:
            seed = self.q_seed.copy()
        else:
            seed = np.array(seed, dtype=float)

        def residual(q):
            q_use = q.copy()

            if keep_joint5_zero:
                q_use[4] = 0.0

            p = self.fk_position(q_use)
            pos_err = p - target_xyz
            reg = 0.01 * (q_use - seed)

            if keep_joint5_zero:
                return np.hstack([pos_err, reg[:4]])
            return np.hstack([pos_err, reg])

        lb = self.lower.copy()
        ub = self.upper.copy()

        if keep_joint5_zero:
            eps = 1e-6
            lb[4] = -eps
            ub[4] = eps
            seed[4] = 0.0

        result = least_squares(
            residual,
            x0=seed,
            bounds=(lb, ub),
            method="trf",
            ftol=1e-6,
            xtol=1e-6,
            gtol=1e-6,
            max_nfev=200,
        )

        q_sol = result.x.copy()
        if keep_joint5_zero:
            q_sol[4] = 0.0

        reached = self.fk_position(q_sol)
        pos_error = np.linalg.norm(reached - target_xyz)
        success = bool(result.success and pos_error < 0.02)

        if success:
            self.q_seed = q_sol.copy()

        return {
            "success": success,
            "q": q_sol,
            "target_xyz": target_xyz,
            "reached_xyz": reached,
            "position_error": pos_error,
            "message": result.message,
        }


class ArmPickIK(Node):
    def __init__(self):
        super().__init__("arm_pick_ik")

        self.pub = self.create_publisher(ArmControl, "/arm/control", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("x_offset", 0.0)
        self.declare_parameter("y_offset", 0.0)
        self.declare_parameter("z_offset", 0.0)
        self.declare_parameter("approach_height", 0.04)

        # camera -> arm calibration
        self.declare_parameter("camera_to_arm_x", 0.0)
        self.declare_parameter("camera_to_arm_y", 0.0)
        self.declare_parameter("table_z_arm", 0.02)

        self.camera_frame = self.get_parameter("camera_frame").value
        self.x_offset = float(self.get_parameter("x_offset").value)
        self.y_offset = float(self.get_parameter("y_offset").value)
        self.z_offset = float(self.get_parameter("z_offset").value)
        self.approach_height = float(self.get_parameter("approach_height").value)

        self.camera_to_arm_x = float(self.get_parameter("camera_to_arm_x").value)
        self.camera_to_arm_y = float(self.get_parameter("camera_to_arm_y").value)
        self.table_z_arm = float(self.get_parameter("table_z_arm").value)

        self.ik = SnowArmIK()
        self.target_cube_frame = None

        # Servo order:
        # [gripper, wrist rotate, wrist, elbow, shoulder, base]
        self.current_servo = [40, 120, 30, 220, 180, 120]

        self.time_per_degree = 20
        self.min_time = 800

        self.get_logger().info(
            f"ArmPickIK started | camera_frame={self.camera_frame}"
        )

    def rad_to_servo(self, q):
        """
        Practical calibrated mapping.

        q = [joint1, joint2, joint3, joint4, joint5] in radians
        Returns:
        [gripper, wrist rotate, wrist, elbow, shoulder, base]
        """
        q1_deg, q2_deg, q3_deg, q4_deg, q5_deg = np.degrees(q)

        gripper = 40.0
        wrist_rotate = 120.0

        base = 120.0 + 0.12 * q1_deg
        shoulder = 70.0 - 0.057 * q2_deg
        elbow = 195.0 + 0.15 * q3_deg
        wrist = 95.0 - 0.02 * q4_deg

        self.get_logger().info(
            f"RAW servo | base={base:.1f}, shoulder={shoulder:.1f}, "
            f"elbow={elbow:.1f}, wrist={wrist:.1f}, wrist_rot={wrist_rotate:.1f}"
        )

        return [
            gripper,
            wrist_rotate,
            float(np.clip(wrist, 0, 240)),
            float(np.clip(elbow, 0, 240)),
            float(np.clip(shoulder, 0, 240)),
            float(np.clip(base, 0, 240)),
        ]

    def move_joints(self, target, label="custom"):
        target = [float(x) for x in target]

        max_delta = max(abs(t - c) for t, c in zip(target, self.current_servo))
        move_time = max(int(max_delta * self.time_per_degree), self.min_time)

        msg = ArmControl()
        msg.position = target
        msg.time = [move_time] * 6

        self.pub.publish(msg)
        self.get_logger().info(f"Moving -> {label} | joints={target} | time={move_time} ms")

        self.current_servo = target
        time.sleep(move_time / 1000.0 + 0.8)

    def get_target_xyz(self):
        possible_frames = ["red_cube", "green_cube", "blue_cube"]

        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.1)

            for frame in possible_frames:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.camera_frame,
                        frame,
                        rclpy.time.Time(),
                    )

                    x_cam = tf_msg.transform.translation.x + self.x_offset
                    y_cam = tf_msg.transform.translation.y + self.y_offset
                    z_cam = tf_msg.transform.translation.z + self.z_offset

                    self.target_cube_frame = frame

                    self.get_logger().info(
                        f"Detected {frame} in {self.camera_frame}: "
                        f"x={x_cam:.3f}, y={y_cam:.3f}, z={z_cam:.3f}"
                    )

                    x_arm = self.camera_to_arm_x + x_cam
                    y_arm = self.camera_to_arm_y + y_cam
                    z_arm = self.table_z_arm

                    self.get_logger().info(
                        f"Converted {frame} to arm_base_link: "
                        f"x={x_arm:.3f}, y={y_arm:.3f}, z={z_arm:.3f}"
                    )

                    return np.array([x_arm, y_arm, z_arm], dtype=float)

                except TransformException:
                    continue

        self.get_logger().warn("No cube frame found: red_cube / green_cube / blue_cube")
        return None

    def solve_pose(self, xyz, label="IK_TARGET"):
        sol = self.ik.ik_position(xyz)

        self.get_logger().info(
            f"{label} | success={sol['success']} | "
            f"target={np.round(sol['target_xyz'], 4)} | "
            f"reached={np.round(sol['reached_xyz'], 4)} | "
            f"err={sol['position_error']:.4f}"
        )

        if not sol["success"]:
            self.get_logger().error(f"IK failed: {sol['message']}")
            return None

        q_rad = sol["q"]
        q_deg = np.degrees(q_rad)
        servo_cmd = self.rad_to_servo(q_rad)

        self.get_logger().info(f"--- {label} ---")
        self.get_logger().info(f"Joint angles (rad): {np.round(q_rad, 4)}")
        self.get_logger().info(f"Joint angles (deg): {np.round(q_deg, 2)}")
        self.get_logger().info(f"Servo command:      {np.round(servo_cmd, 1)}")
        self.get_logger().info(f"FK reached xyz:     {np.round(sol['reached_xyz'], 4)}")

        return servo_cmd

    def run_sequence(self):
        self.get_logger().info("Starting IK move sequence")

        home = [40, 120, 30, 220, 180, 120]
        detect = [40, 120, 30, 190, 100, 120]

        self.move_joints(home, "HOME")
        time.sleep(1.0)

        self.move_joints(detect, "DETECT")
        time.sleep(1.5)

        target_xyz = self.get_target_xyz()
        if target_xyz is None:
            self.get_logger().error("No cube transform found.")
            return

        grasp_xyz = np.array([
            target_xyz[0],
            target_xyz[1],
            target_xyz[2] + 0.02
        ], dtype=float)

        pre_grasp_xyz = grasp_xyz.copy()
        pre_grasp_xyz[2] += self.approach_height

        self.get_logger().info(f"Target cube   = {self.target_cube_frame}")
        self.get_logger().info(f"Pre-grasp xyz = {np.round(pre_grasp_xyz, 4)}")
        self.get_logger().info(f"Grasp xyz     = {np.round(grasp_xyz, 4)}")

        pre_grasp_cmd = self.solve_pose(pre_grasp_xyz, "PRE_GRASP")
        if pre_grasp_cmd is None:
            return

        time.sleep(1.0)
        self.move_joints(pre_grasp_cmd, "PRE_GRASP")
        self.get_logger().info("Moved to PRE_GRASP.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmPickIK()

    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
