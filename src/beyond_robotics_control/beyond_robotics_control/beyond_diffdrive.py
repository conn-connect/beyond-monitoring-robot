import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .zlac8105d_controller import ZLAC8105D_Controller

class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('beyond_diffdrive')
        self.controller = ZLAC8105D_Controller()
        self.controller.enable_motor()
        self.controller.set_mode(self.controller.VEL_CONTROL)
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        wheel_separation = 0.52  # Adjust based on your robot
        wheel_radius = 0.21  # Adjust based on your robot

        left_vel = msg.linear.x - (msg.angular.z * wheel_separation / 2)
        right_vel = msg.linear.x + (msg.angular.z * wheel_separation / 2)

        left_rpm = (left_vel / (2 * 3.14159 * wheel_radius)) * 60
        right_rpm = (right_vel / (2 * 3.14159 * wheel_radius)) * 60

        self.controller.set_rpm(left_rpm, right_rpm)

    def __del__(self):
        self.controller.set_rpm(0, 0)
        self.controller.disable_motor()

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
