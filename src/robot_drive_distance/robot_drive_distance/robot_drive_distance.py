#!/usr/bin/env python3
"""
import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
import math

class DriveDistance(Node):
    def __init__(self, target_distance_m):
        super().__init__('drive_distance')

        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.sub = self.create_subscription(Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10)

        self.start_left = None
        self.start_right = None
        self.left_encoder = 0
        self.right_encoder = 0

        # Wheel parameters
        self.left_radius = 0.047  # meters
        self.right_radius = 0.047  # meters
        self.ticks_per_revolution = 2872  # adjust to your encoders

        self.target_distance = target_distance_m
        self.timer = self.create_timer(0.05, self.drive)
        self.get_logger().info(f"Driving forward {self.target_distance} meters...")

    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        if self.start_left is None:
            self.start_left = self.left_encoder
            self.start_right = self.right_encoder

    def ticks_to_meters(self, ticks, radius):
        circumference = 2 * math.pi * radius
        return circumference * (ticks / self.ticks_per_revolution)

    def drive(self):
        if self.start_left is None:
            return

        delta_left = self.left_encoder - self.start_left
        delta_right = self.right_encoder - self.start_right

        # Distance traveled per wheel
        left_dist = self.ticks_to_meters(delta_left, self.left_radius)
        right_dist = self.ticks_to_meters(delta_right, self.right_radius)

        # Average distance
        distance_traveled = (left_dist + right_dist) / 2.0
        self.get_logger().info(f"Distance traveled: {distance_traveled:.3f} m (L: {left_dist:.3f}, R: {right_dist:.3f})")

        msg = DutyCycles()
        if distance_traveled < self.target_distance:
            msg.duty_cycle_left = 0.5
            msg.duty_cycle_right = 0.5
        else:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.get_logger().info("Target distance reached!")
            self.timer.cancel()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveDistance(target_distance_m=2.0)  # drive 1 meter
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
import math

class DriveDistance(Node):
    def __init__(self, target_distance_m):
        super().__init__('drive_distance')

        # Publishers & subscribers
        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.sub = self.create_subscription(Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10)

        # Encoder variables
        self.start_left = None
        self.start_right = None
        self.left_encoder = 0
        self.right_encoder = 0

        # Wheel parameters
        self.left_radius = 0.047  # meters
        self.right_radius = 0.047  # meters
        self.ticks_per_revolution = 2872

        # Target distance
        self.target_distance = target_distance_m

        # Proportional gain for correction
        self.Kp = 1.0  # You can tune this if robot oscillates

        # Timer to drive
        self.timer = self.create_timer(0.05, self.drive)
        self.get_logger().info(f"Driving forward {self.target_distance} meters...")

    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        if self.start_left is None:
            self.start_left = self.left_encoder
            self.start_right = self.right_encoder

    def ticks_to_meters(self, ticks, radius):
        circumference = 2 * math.pi * radius
        return circumference * (ticks / self.ticks_per_revolution)

    def drive(self):
        if self.start_left is None:
            return

        # Distance traveled by each wheel
        delta_left = self.left_encoder - self.start_left
        delta_right = self.right_encoder - self.start_right

        left_dist = self.ticks_to_meters(delta_left, self.left_radius)
        right_dist = self.ticks_to_meters(delta_right, self.right_radius)

        # Average distance traveled
        distance_traveled = (left_dist + right_dist) / 2.0
        self.get_logger().info(f"Distance traveled: {distance_traveled:.3f} m (L: {left_dist:.3f}, R: {right_dist:.3f})")

        # Create motor command
        msg = DutyCycles()
        if distance_traveled < self.target_distance:
            # Simple proportional correction to drive straight
            error = left_dist - right_dist
            msg.duty_cycle_left = 0.5 - self.Kp * error
            msg.duty_cycle_right = 0.5 + self.Kp * error

            # Limit duty cycles between 0 and 1
            msg.duty_cycle_left = max(0.0, min(1.0, msg.duty_cycle_left))
            msg.duty_cycle_right = max(0.0, min(1.0, msg.duty_cycle_right))
        else:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.get_logger().info("Target distance reached!")
            self.timer.cancel()

        # Publish command
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveDistance(target_distance_m=2.0)  # Drive 2 meters
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
from nav_msgs.msg import Odometry
import math


class DriveToCoordinate(Node):
    def __init__(self, target_x, target_y):
        super().__init__('drive_to_coordinate')

        # Publishers & subscribers
        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.enc_sub = self.create_subscription(
            Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Encoder state
        self.left_encoder = 0
        self.right_encoder = 0
        self.start_left = None
        self.start_right = None

        # Odometry state (relative)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Initial odom reference
        self.odom_initialized = False
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0

        # Wheel parameters
        self.left_radius = 0.047
        self.right_radius = 0.047
        self.ticks_per_revolution = 2872

        # Target
        self.target_x = target_x
        self.target_y = target_y

        # Control gains
        self.Kp_distance = 0.6
        self.Kp_angle = 2.5
        self.Kp_straight = 1.0

        # Thresholds
        self.ANGLE_TOL = 0.15     # rad (~8.5 deg)
        self.DIST_TOL = 0.05      # meters

        # Timer
        self.timer = self.create_timer(0.05, self.drive)
        self.get_logger().info(f"Driving to ({self.target_x}, {self.target_y})")


    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        if self.start_left is None:
            self.start_left = self.left_encoder
            self.start_right = self.right_encoder


    def odom_callback(self, msg):
        x_raw = msg.pose.pose.position.x
        y_raw = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta_raw = math.atan2(siny_cosp, cosy_cosp)

        # Set initial pose as (0,0,0)
        if not self.odom_initialized:
            self.x0 = x_raw
            self.y0 = y_raw
            self.theta0 = theta_raw
            self.odom_initialized = True
            self.get_logger().info("Initial pose set to (0,0,0)")
            return

        self.x = x_raw - self.x0
        self.y = y_raw - self.y0
        self.theta = theta_raw - self.theta0
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))


    def ticks_to_meters(self, ticks, radius):
        return 2 * math.pi * radius * (ticks / self.ticks_per_revolution)


    def drive(self):
        if not self.odom_initialized or self.start_left is None:
            return

        # Target vector
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)

        angle_error = target_heading - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        msg = DutyCycles()

        # STOP condition
        if distance < self.DIST_TOL:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.pub.publish(msg)
            self.timer.cancel()
            self.get_logger().info("Reached target coordinate")
            return

        # TURN IN PLACE
        if abs(angle_error) > self.ANGLE_TOL:
            turn = self.Kp_angle * angle_error
            turn = max(-0.5, min(0.5, turn))

            msg.duty_cycle_left = -turn
            msg.duty_cycle_right = turn

        # DRIVE STRAIGHT
        else:
            base_speed = min(0.5, self.Kp_distance * distance)

            delta_left = self.left_encoder - self.start_left
            delta_right = self.right_encoder - self.start_right

            left_dist = self.ticks_to_meters(delta_left, self.left_radius)
            right_dist = self.ticks_to_meters(delta_right, self.right_radius)

            error_straight = left_dist - right_dist
            correction = self.Kp_straight * error_straight

            msg.duty_cycle_left = base_speed - correction
            msg.duty_cycle_right = base_speed + correction

        # Clamp
        msg.duty_cycle_left = max(-1.0, min(1.0, msg.duty_cycle_left))
        msg.duty_cycle_right = max(-1.0, min(1.0, msg.duty_cycle_right))

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveToCoordinate(target_x=2.0, target_y=1.5)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
import math

class DriveToPoint(Node):
    def __init__(self, target_x, target_y):
        super().__init__('drive_to_point')

        # Publishers & subscribers
        self.pub = self.create_publisher(
            DutyCycles, '/phidgets/motor/duty_cycles', 10
        )
        self.sub = self.create_subscription(
            Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10
        )

        # Encoder values
        self.start_left = None
        self.start_right = None
        self.prev_left = None
        self.prev_right = None
        self.left_encoder = 0
        self.right_encoder = 0

        # Robot parameters
        self.r = 0.047                 # wheel radius (m)
        self.L = 0.30                  # wheel separation (m)
        self.ticks_per_rev = 2872

        # Pose (odometry)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Target
        self.x_t = target_x
        self.y_t = target_y

        # Control gains
        self.k_rho = 0.8
        self.k_alpha = 2.0

        # Limits
        self.v_max = 0.4
        self.omega_max = 2.0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f"Driving to target (x={self.x_t}, y={self.y_t})"
        )

    # --------------------------------------------------
    # Encoder callback
    # --------------------------------------------------
    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        if self.start_left is None:
            self.start_left = self.left_encoder
            self.start_right = self.right_encoder
            self.prev_left = self.left_encoder
            self.prev_right = self.right_encoder

    # --------------------------------------------------
    # Utility functions
    # --------------------------------------------------
    def ticks_to_meters(self, ticks):
        return (2 * math.pi * self.r) * (ticks / self.ticks_per_rev)

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # --------------------------------------------------
    # Main control loop
    # --------------------------------------------------
    def control_loop(self):
        if self.start_left is None:
            return

        # Encoder increments
        d_left_ticks = self.left_encoder - self.prev_left
        d_right_ticks = self.right_encoder - self.prev_right

        self.prev_left = self.left_encoder
        self.prev_right = self.right_encoder

        # Distance traveled by wheels
        d_left = self.ticks_to_meters(d_left_ticks)
        d_right = self.ticks_to_meters(d_right_ticks)

        # Forward kinematics (odometry)
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.L

        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta = self.wrap_to_pi(self.theta + d_theta)

        # Distance & heading to target
        dx = self.x_t - self.x
        dy = self.y_t - self.y
        rho = math.sqrt(dx**2 + dy**2)
        theta_d = math.atan2(dy, dx)
        alpha = self.wrap_to_pi(theta_d - self.theta)

        self.get_logger().info(
            f"x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f}, ρ={rho:.2f}"
        )

        msg = DutyCycles()

        # Stop condition
        if rho < 0.05:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.pub.publish(msg)
            self.get_logger().info("Target reached!")
            self.timer.cancel()
            return

        # Control law
        v = self.k_rho * rho
        omega = self.k_alpha * alpha

        # Saturation
        v = max(min(v, self.v_max), -self.v_max)
        omega = max(min(omega, self.omega_max), -self.omega_max)

        # Inverse kinematics
        v_r = v + (omega * self.L / 2)
        v_l = v - (omega * self.L / 2)

        # Convert to duty cycle (simple proportional mapping)
        msg.duty_cycle_right = max(min(v_r / self.v_max, 1.0), -1.0)
        msg.duty_cycle_left = max(min(v_l / self.v_max, 1.0), -1.0)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveToPoint(target_x=2.0, target_y=1.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



