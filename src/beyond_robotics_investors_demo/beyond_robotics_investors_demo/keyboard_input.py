import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select

class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('keyboard_input_node')
        self.publisher = self.create_publisher(String, 'keyboard_input', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        print("Press 's' to start, 'c' to continue, 'q' to stop immediately, 'e' to exit.")

    def timer_callback(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 's':
                self.publisher.publish(String(data='start'))
                self.get_logger().info('Start command sent')
            elif key == 'c':
                self.publisher.publish(String(data='continue'))
                self.get_logger().info('Continue command sent')
            elif key == 'q':
                self.publisher.publish(String(data='stop'))
                self.get_logger().info('Emergency stop command sent')
            elif key == 'e':
                self.get_logger().info('Exiting...')
                sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    
    # Set terminal to raw mode
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        rclpy.spin(node)
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
