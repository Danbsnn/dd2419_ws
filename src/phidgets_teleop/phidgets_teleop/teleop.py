import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles
import threading
import sys
import termios
import tty
import time

class MotorTeleop(Node):
    def __init__(self):
        super().__init__('motor_teleop')
        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)

        # Current motor speeds
        self.left_speed = 0.0
        self.right_speed = 0.0

        # Target motor speeds
        self.target_left = 0.0
        self.target_right = 0.0

        self.lock = threading.Lock()
        self.keys_pressed = set()

        # Publish timer at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_motor)

        # Start keyboard listener thread
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        # Optional: display active keys
        threading.Thread(target=self.print_active_keys, daemon=True).start()

        self.get_logger().info("Teleop ready: W forward/A left/S back/D right to move, Q to stop, X to exit (smooth ramp)")

    # -------------------------
    # Keyboard input loop
    # -------------------------
    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1).lower()
                with self.lock:
                    if ch in ('w', 'a', 's', 'd'):
                        self.keys_pressed.add(ch)
                        print(f"\n{ch.upper()} pressed")  # Immediate feedback
                    elif ch == 'q':
                        self.keys_pressed.clear()
                        self.target_left = 0.0
                        self.target_right = 0.0
                        print("\nQ pressed → Motors stopped")
                    elif ch == 'x':
                        self.target_left = 0.0
                        self.target_right = 0.0
                        print("\nX pressed → Exiting teleop")
                        rclpy.shutdown()
                        break
                    elif ord(ch) == 3:  # Ctrl+C
                        rclpy.shutdown()
                        break

                    # Update target speeds
                    self.update_target_speeds()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # -------------------------
    # Compute target speeds
    # -------------------------
    def update_target_speeds(self):
        left = 0.0
        right = 0.0
        if 'w' in self.keys_pressed:
            left += 0.5
            right += 0.5
        if 's' in self.keys_pressed:
            left -= 0.5
            right -= 0.5
        if 'a' in self.keys_pressed:
            left -= 0.3
            right += 0.3
        if 'd' in self.keys_pressed:
            left += 0.3
            right -= 0.3

        # Clamp speeds
        self.target_left = max(min(left, 0.7), -0.7)
        self.target_right = max(min(right, 0.7), -0.7)

    # -------------------------
    # Publish motor commands (smooth ramp)
    # -------------------------
    def publish_motor(self):
        with self.lock:
            ramp_rate = 0.05  # change per loop
            self.left_speed += max(min(self.target_left - self.left_speed, ramp_rate), -ramp_rate)
            self.right_speed += max(min(self.target_right - self.right_speed, ramp_rate), -ramp_rate)

            msg = DutyCycles()
            msg.duty_cycle_left = self.left_speed
            msg.duty_cycle_right = self.right_speed
            self.pub.publish(msg)

    # -------------------------
    # Display currently pressed keys
    # -------------------------
    def print_active_keys(self):
        while True:
            with self.lock:
                if self.keys_pressed:
                    keys_str = ''.join(k.upper() for k in self.keys_pressed)
                    print(f"\rActive keys: {keys_str}        ", end='', flush=True)
                else:
                    print(f"\rActive keys: None             ", end='', flush=True)
            time.sleep(0.1)

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

