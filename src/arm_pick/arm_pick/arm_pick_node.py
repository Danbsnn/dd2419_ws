#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from robp_interfaces.msg import ArmControl
from tf2_ros import Buffer, TransformListener, TransformException


# ── Motor index mapping (ArmControl array) ──────────────────────────────────
# Index 0 = gripper   (not a CCD joint)
# Index 1 = link5     joint5  (wrist yaw,   axis Z)
# Index 2 = link4     joint4  (elbow pitch, axis X)
# Index 3 = link3     joint3  (elbow pitch, axis X)
# Index 4 = link2     joint2  (shoulder,    axis X)
# Index 5 = arm base  joint1  (base yaw,    axis Z)
#
# CCD iterates end→base: motor indices [1, 2, 3, 4, 5]
# The TF frame for motor index i  is  link(i)   (link1..link5)
# ────────────────────────────────────────────────────────────────────────────

MOTOR_TO_LINK = {
    1: 'link5',
    2: 'link4',
    3: 'link3',
    4: 'link2',
    5: 'link1',   # joint1 lives above arm_base_link, child is link1
}

# Joint axis in base_link frame (from URDF <axis xyz=...>)
MOTOR_AXIS = {
    1: np.array([0, 0, 1]),   # joint5  yaw
    2: np.array([1, 0, 0]),   # joint4  pitch
    3: np.array([1, 0, 0]),   # joint3  pitch
    4: np.array([1, 0, 0]),   # joint2  pitch
    5: np.array([0, 0, 1]),   # joint1  yaw
}

# Soft joint limits in degrees (tune to your robot's safe range)
MOTOR_LIMITS = {
    0: (80, 150),    # gripper  (open=80, closed=150 roughly)
    1: (0,   240),   # joint5
    2: (0,   240),   # joint4
    3: (0,   240),   # joint3
    4: (0,   240),   # joint2
    5: (0,   240),   # joint1
}


class ArmPickupCCD(Node):
    def __init__(self):
        super().__init__('arm_pick_ccd')

        self.state = "WAIT_FOR_OBJECT"

        # Frozen target — set once, never updated after that
        self._frozen_target: np.ndarray | None = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Poll for the object at 1 Hz until detected
        self.timer = self.create_timer(1.0, self._try_capture_target)

        # Publisher
        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        # Motor config
        self.time_per_degree = 15
        self.min_time = 1500
        # Index:          [gripper, link5, link4, link3, link2, base]
        self.home_motors = [80, 120, 30, 168, 120, 40]
        self.current_motors = self.home_motors.copy()

    # ─────────────────────────────────────────────────────────────────────────
    # MOTOR COMMAND  (one move per call — do NOT call per-joint inside CCD)
    # ─────────────────────────────────────────────────────────────────────────
    def send_motor_command(self, motors: list[float]):
        """Send one command and block until the arm has had time to arrive."""
        clamped = [
            float(np.clip(motors[i], *MOTOR_LIMITS[i]))
            for i in range(len(motors))
        ]
        max_delta = max(abs(a - b)
                        for a, b in zip(clamped, self.current_motors))
        move_time = max(int(max_delta * self.time_per_degree), self.min_time)

        msg = ArmControl()
        msg.position = clamped
        msg.time = [move_time] * 6
        self.pub.publish(msg)

        time.sleep(move_time / 1000.0)
        self.current_motors = clamped.copy()

    # ─────────────────────────────────────────────────────────────────────────
    # TF LOOKUP
    # ─────────────────────────────────────────────────────────────────────────
    def _lookup_xyz(self, frame: str) -> np.ndarray:
        """Raises TransformException if the frame is not yet available."""
        tf = self.tf_buffer.lookup_transform(
            'base_link', frame, rclpy.time.Time()
        )
        return np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ])

    # ─────────────────────────────────────────────────────────────────────────
    # STEP 1 — capture target once
    # ─────────────────────────────────────────────────────────────────────────
    def _try_capture_target(self):
        """
        Called at 1 Hz until the detector publishes cube_detected1.
        Freezes the position in base_link coordinates and starts CCD.
        After this point the detector keeps running but we never read it again.
        """
        if self.state != "WAIT_FOR_OBJECT":
            return

        try:
            # cube_detected1 is the once-published static TF from your
            # TestTransformPublisher snippet — already in base_link frame
            target = self._lookup_xyz('cube_detected1')
        except TransformException:
            self.get_logger().info("Waiting for cube_detected1 ...")
            return

        self._frozen_target = target.copy()   # ← frozen, never touched again
        self.get_logger().info(
            f"Target frozen at base_link: "
            f"({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}) m"
        )

        self.state = "APPROACH"
        self.timer.cancel()   # stop polling

        self._execute_pickup()

    # ─────────────────────────────────────────────────────────────────────────
    # STEP 2 — full pickup sequence
    # ─────────────────────────────────────────────────────────────────────────
    def _execute_pickup(self):
        target = self._frozen_target

        # Approach 4 cm above the object first
        above = target.copy()
        above[2] += 0.04
        self.get_logger().info("CCD: moving above target")
        self.ccd_move_to_target(above)

        # Descend to object
        self.get_logger().info("CCD: descending to target")
        self.ccd_move_to_target(target)

        # Close gripper
        self._grasp()

        self.state = "DONE"
        self.get_logger().info("Pickup complete — shutting down")
        # Schedule shutdown outside the callback so spin() exits cleanly
        self.create_timer(0.1, self._shutdown_once)

    def _shutdown_once(self):
        self.destroy_node()
        rclpy.shutdown()

    # ─────────────────────────────────────────────────────────────────────────
    # CCD  (uses frozen target — one motor command per iteration)
    # ─────────────────────────────────────────────────────────────────────────
    def ccd_move_to_target(self, target: np.ndarray, max_iter: int = 40):
        for iteration in range(max_iter):

            # Read EE position at the START of this iteration only
            try:
                p_ee = self._lookup_xyz('grasping_frame')
            except TransformException:
                self.get_logger().warn("grasping_frame not available, skipping")
                continue

            error = np.linalg.norm(target - p_ee)
            self.get_logger().info(f"  CCD iter {iteration}  error={error:.4f} m")

            if error < 0.01:
                self.get_logger().info("  → target reached")
                return

            # Build the full proposed motor array for this iteration
            proposed = self.current_motors.copy()

            # Iterate end → base:  motor indices 1 (link5) → 5 (base)
            for motor_idx in [1, 2, 3, 4, 5]:
                link_frame = MOTOR_TO_LINK[motor_idx]
                try:
                    p_joint = self._lookup_xyz(link_frame)
                except TransformException:
                    continue

                v1 = p_ee    - p_joint
                v2 = target  - p_joint

                n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
                if n1 < 1e-6 or n2 < 1e-6:
                    continue

                v1 /= n1
                v2 /= n2

                dot   = float(np.clip(np.dot(v1, v2), -1.0, 1.0))
                angle = np.arccos(dot)

                cross     = np.cross(v1, v2)
                axis      = MOTOR_AXIS[motor_idx]
                direction = np.sign(np.dot(cross, axis))

                step = 0.15 * angle   # damped step
                proposed[motor_idx] += np.degrees(step) * direction

                # Clamp to joint limits immediately
                lo, hi = MOTOR_LIMITS[motor_idx]
                proposed[motor_idx] = float(np.clip(proposed[motor_idx], lo, hi))

            # ONE motor command per full CCD sweep — then wait for the arm
            self.send_motor_command(proposed)

            # After the arm has settled, update p_ee at the top of next loop

        self.get_logger().warn("CCD did not fully converge")

    # ─────────────────────────────────────────────────────────────────────────
    # GRIPPER
    # ─────────────────────────────────────────────────────────────────────────
    def _grasp(self):
        self.get_logger().info("Closing gripper")
        motors = self.current_motors.copy()
        motors[0] = 150   # gripper closed
        self.send_motor_command(motors)
        time.sleep(1.0)


# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = ArmPickupCCD()
    node.send_motor_command(node.home_motors)   # go home first
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()