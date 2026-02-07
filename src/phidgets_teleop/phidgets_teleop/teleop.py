import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles
from pynput import keyboard
import threading


class MotorTeleop(Node):

    def __init__(self):
        super().__init__('motor_teleop')

        # Publisher uses the correct message type
        self.pub = self.create_publisher(
            DutyCycles,
            '/phidgets/motor/duty_cycles',
            10
        )

        self.left_speed = 0.0
        self.right_speed = 0.0
        self.lock = threading.Lock()

        # Keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        # Publish timer at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_motor)

        self.get_logger().info("Teleop ready: W A S D to move, Q to quit")


    # -------------------------
    # Keyboard handling
    # -------------------------
    def on_press(self, key):
        try:
            k = key.char.lower()
            with self.lock:
                if k == '8':
                    self.left_speed = 0.5
                    self.right_speed = 0.5
                   
                    
                elif k == '2':
                    self.left_speed = -0.5
                    self.right_speed = -0.5
                    
                    
                elif k == '4':
                    self.left_speed = -0.3
                    self.right_speed = 0.3
                   
                    
                elif k == '6':
                    self.left_speed = 0.3
                    self.right_speed = -0.3
                 
                    
                elif k == '5':
                    rclpy.shutdown()
        except:
            pass

    def on_release(self, key):
        with self.lock:
            self.left_speed = 0.0
            self.right_speed = 0.0

    # -------------------------
    # Publish motor commands
    # -------------------------
    def publish_motor(self):
        with self.lock:
            msg = DutyCycles()
            msg.duty_cycle_left = self.left_speed
            msg.duty_cycle_right = self.right_speed

        self.pub.publish(msg)


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


