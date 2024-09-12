import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from beyond_robotics_interfaces.msg import LaneGuidance
import sys
import tty
import termios
import select

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lane_guidance_subscription = self.create_subscription(
            LaneGuidance,
            'lane_guidance',
            self.lane_guidance_callback,
            10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.movement_enabled = False
        self.current_angular_z = 0.0
        print("Press 'w' to move forward, 's' to stop, 'q' to quit.")

    def timer_callback(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'w':
                self.movement_enabled = True
                print("Movement enabled")
            elif key == 's':
                self.movement_enabled = False
                self.stop_robot()
                print("Robot stopped")
            elif key == 'q':
                self.stop_robot()
                print("Quitting...")
                sys.exit(0)

        if self.movement_enabled:
            twist = Twist()
            twist.linear.x = 0.1  # Forward velocity
            twist.angular.z = self.current_angular_z
            self.cmd_vel_publisher.publish(twist)

    def lane_guidance_callback(self, msg):
        self.current_angular_z = msg.angular_z

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    
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
