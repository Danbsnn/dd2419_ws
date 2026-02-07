#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles, Encoders
from pynput import keyboard
import threading

class MotorTeleop(Node):
    def __init__(self):
        super().__init__('motor_teleop')

        # -------------------------
        # Publisher for motors
        # -------------------------
        self.pub = self.create_publisher(
            DutyCycles,
            '/phidgets/motor/duty_cycles',
            10
        )

        # -------------------------
        # Subscriber for encoders
        # -------------------------
        self.sub_encoders = self.create_subscription(
            Encoders,
            '/phidgets/motor/encoders',
            self.encoder_callback,
            10
        )

        self.left_encoder = 0
        self.right_encoder = 0

        # -------------------------
        # Motor control
        # -------------------------
        self.speed = 0.6
        self.turn = 0.5
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0

        self.lock = threading.Lock()

        # -------------------------
        # Keyboard listener
        # -------------------------
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        # -------------------------
        # Timer to publish motors & print encoders
        # -------------------------
        self.timer = self.create_timer(0.1, self.publish_motor)

        self.get_logger().info(
            "Teleop ready: 8 forward | 2 back | 4 left | 6 right | 5 quit"
        )

    # -------------------------
    # Encoder callback
    # -------------------------
    def encoder_callback(self, msg):
        self.left_encoder = msg.encoder_left
        self.right_encoder = msg.encoder_right

    # -------------------------
    # Convert cmd -> wheels
    # -------------------------
    def update_wheels(self):
        left = self.linear_vel - self.angular_vel
        right = self.linear_vel + self.angular_vel
        self.left_speed = max(-1.0, min(1.0, left))
        self.right_speed = max(-1.0, min(1.0, right))

    # -------------------------
    # Keyboard handling
    # -------------------------
    def on_press(self, key):
        try:
            k = key.char.lower()
            with self.lock:
                if k == '8':
                    self.linear_vel = self.speed
                    self.angular_vel = 0.0
                elif k == '2':
                    self.linear_vel = -self.speed
                    self.angular_vel = 0.0
                elif k == '4':
                    self.linear_vel = 0.0
                    self.angular_vel = self.turn
                elif k == '6':
                    self.linear_vel = 0.0
                    self.angular_vel = -self.turn
                elif k == '5':
                    rclpy.shutdown()
                    return
                self.update_wheels()
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            k = key.char.lower()
            if k in ['8', '2', '4', '6']:
                with self.lock:
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    self.update_wheels()
        except AttributeError:
            pass

    # -------------------------
    # Publish motor commands & print encoder
    # -------------------------
    def publish_motor(self):
        with self.lock:
            msg = DutyCycles()
            msg.duty_cycle_left = float(self.left_speed)
            msg.duty_cycle_right = float(self.right_speed)
            self.pub.publish(msg)

        self.get_logger().info(
            f"[Encoders] Left: {self.left_encoder} | Right: {self.right_encoder}"
        )


# -------------------------
# Main
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MotorTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

