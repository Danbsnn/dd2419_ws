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
"""

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
"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
from nav_msgs.msg import Odometry  # Odometry for x, y, theta
import math

class DriveToCoordinate(Node):
    def __init__(self, target_x, target_y):
        super().__init__('drive_to_coordinate')

        # Publishers & subscribers
        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.enc_sub = self.create_subscription(Encoders, '/phidgets/motor/encoders', self.encoder_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Encoder variables
        self.left_encoder = 0
        self.right_encoder = 0
        self.start_left = None
        self.start_right = None

        # Odometry / pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians

        # Wheel parameters
        self.left_radius = 0.047  # meters
        self.right_radius = 0.047
        self.ticks_per_revolution = 2872

        # Target coordinate
        self.target_x = target_x
        self.target_y = target_y

        # Control gains
        self.Kp_distance = 0.5   # linear correction
        self.Kp_angle = 2.0      # angular correction
        self.Kp_straight = 1.0   # correction for wheel mismatch

        # Timer
        self.timer = self.create_timer(0.05, self.drive)
        self.get_logger().info(f"Driving to coordinate ({self.target_x}, {self.target_y})...")

    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

        if self.start_left is None:
            self.start_left = self.left_encoder
            self.start_right = self.right_encoder

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def ticks_to_meters(self, ticks, radius):
        circumference = 2 * math.pi * radius
        return circumference * (ticks / self.ticks_per_revolution)

    def drive(self):
        if self.start_left is None:
            return

        # Compute vector to target
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        angle_error = target_heading - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # normalize -pi..pi

        msg = DutyCycles()

        # Stop if close to target
        if distance_to_target < 0.05:  # 5 cm tolerance
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0
            self.pub.publish(msg)
            self.get_logger().info("Reached target coordinate!")
            self.timer.cancel()
            return

        # Compute linear and angular components
        linear_command = self.Kp_distance * distance_to_target
        angular_command = self.Kp_angle * angle_error

        # Convert linear & angular into left/right wheel duty cycles
        # Simple differential drive formula
        base_speed = max(0.0, min(1.0, linear_command))  # cap to 0..1
        turn = angular_command  # can be >1; will be capped later

        # Wheel mismatch correction using encoder distances
        delta_left = self.left_encoder - self.start_left
        delta_right = self.right_encoder - self.start_right
        left_dist = self.ticks_to_meters(delta_left, self.left_radius)
        right_dist = self.ticks_to_meters(delta_right, self.right_radius)
        error_straight = left_dist - right_dist

        # Final duty cycles
        msg.duty_cycle_left = base_speed - turn - self.Kp_straight * error_straight
        msg.duty_cycle_right = base_speed + turn + self.Kp_straight * error_straight

        # Clamp between 0..1
        msg.duty_cycle_left = max(0.0, min(1.0, msg.duty_cycle_left))
        msg.duty_cycle_right = max(0.0, min(1.0, msg.duty_cycle_right))

        self.pub.publish(msg)
        self.get_logger().info(f"Distance: {distance_to_target:.3f}, Angle error: {math.degrees(angle_error):.1f} deg")

def main(args=None):
    rclpy.init(args=args)
    node = DriveToCoordinate(target_x=2.0, target_y=1.5)  # example target
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

"""
