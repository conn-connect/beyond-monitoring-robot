import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from beyond_robotics_interfaces.msg import LaneGuidance, QRInfo
import math

class RobotControlNode(Node):
    def __init__(self):        
        super().__init__('robot_control_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        self.lane_guidance_subscription = self.create_subscription(
            LaneGuidance, 'lane_guidance', self.lane_guidance_callback, 10)
        self.qr_code_subscription = self.create_subscription(
            QRInfo, 'qr_code', self.qr_code_callback, 10)
        self.keyboard_input_subscription = self.create_subscription(
            String, 'keyboard_input', self.keyboard_input_callback, 10)

        self.state = 'WAITING_TO_START'
        self.current_qr = ''
        self.current_qr_width = 0
        self.current_qr_x = 0
        self.target_qrs = [['BR-PP-00001', 'BR-PP-00002'], ['BR-PP-00003', 'BR-PP-00004']]
        self.turn_qrs = ['BR-RP-00001', 'BR-RP-00002']
        self.current_side = 0
        self.current_target = 0

        self.wheel_diameter = 0.21  # meters
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.post_qr_distance = 1.2  # 1.2m after QR detection
        self.image_width = 960  # 이미지 너비, 실제 값으로 조정 필요

        self.move_start_time = None
        self.move_duration = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def lane_guidance_callback(self, msg):
        if self.state in ['FOLLOWING_LANE', 'SEARCHING_TURN_QR']:
            twist = Twist()
            if self.state == 'FOLLOWING_LANE':
                twist.linear.x = 0.2  # 일정한 전진 속도
                twist.angular.z = msg.angular_z
            elif self.state == 'SEARCHING_TURN_QR':
                twist.linear.x = 0.15  # 회전 QR 검색 시 느린 속도
                twist.angular.z = 0.0  # QR 코드를 향해 직진
            self.cmd_vel_publisher.publish(twist)

    def qr_code_callback(self, msg):
        self.current_qr = msg.data
        self.current_qr_width = msg.width
        self.current_qr_x = msg.x

        if self.state == 'FOLLOWING_LANE':
            if self.current_qr in self.target_qrs[self.current_side] or self.current_qr in self.turn_qrs:
                qr_position = self.current_qr_x + self.current_qr_width / 2
                if qr_position > self.image_width - 80:  # QR 코드 중심이 이미지의 오른쪽 80% 이상에 위치할 때
                    self.state = 'APPROACHING_STOP'
                    self.get_logger().info(f"Approaching stop point: {self.current_qr}")

    def keyboard_input_callback(self, msg):
        if msg.data == 'start' and self.state == 'WAITING_TO_START':
            self.state = 'FOLLOWING_LANE'
            self.get_logger().info("로봇 이동 시작")
        elif msg.data == 'continue' and self.state == 'STOPPED_AT_POINT':
            if self.current_qr in self.turn_qrs:
                self.state = 'ROTATING'
                self.get_logger().info(f"회전 시작: {self.current_qr}")
            else:
                self.state = 'FOLLOWING_LANE'
                self.current_target += 1
                self.get_logger().info(f"다음 지점으로 이동: {self.target_qrs[self.current_side][self.current_target]}")
        elif msg.data == 'stop':
            self.state = 'EMERGENCY_STOP'
            self.stop_robot()
            self.get_logger().info("긴급 정지 활성화")

    def control_loop(self):
        if self.state == 'APPROACHING_STOP':
            if self.move_start_time is None:
                # Initialize the movement
                self.move_start_time = self.get_clock().now()
                self.move_duration = rclpy.duration.Duration(seconds=self.calculate_move_duration())
                self.get_logger().info(f"Starting {self.post_qr_distance}m forward movement, duration: {self.move_duration.nanoseconds/1e9:.2f} seconds")
            
            current_time = self.get_clock().now()
            if current_time - self.move_start_time < self.move_duration:
                # Continue moving forward
                twist = Twist()
                twist.linear.x = 0.2  # Adjust this speed as needed
                self.cmd_vel_publisher.publish(twist)
            else:
                # Movement complete
                self.stop_robot()
                self.state = 'STOPPED_AT_POINT'
                self.move_start_time = None
                self.move_duration = None
                self.get_logger().info(f"정지 지점에 도착: {self.current_qr}")
        elif self.state == 'ROTATING':
            self.rotate_180()
            self.current_side += 1
            self.current_target = 0
            if self.current_side >= len(self.target_qrs):
                self.state = 'MISSION_COMPLETE'
                self.get_logger().info("임무 완료")
            else:
                self.state = 'FOLLOWING_LANE'
                self.get_logger().info(f"회전 완료. 다음 지점으로 이동: {self.target_qrs[self.current_side][self.current_target]}")
        elif self.state == 'FOLLOWING_LANE':
            # Ensure the robot keeps moving if there's no lane guidance
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_publisher.publish(twist)
        elif self.state == 'EMERGENCY_STOP' or self.state == 'MISSION_COMPLETE':
            self.stop_robot()

        self.publish_state()

    def calculate_move_duration(self):
        # Calculate the time needed to move based on the wheel circumference and desired speed
        desired_speed = 0.2  # m/s, adjust as needed
        return self.post_qr_distance / desired_speed
        
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

    def rotate_180(self):
        # 실제 환경에서는 오도메트리 피드백을 사용하여 구현해야 합니다.
        self.get_logger().info("180도 회전 중")
        twist = Twist()
        twist.angular.z = 3.14/2 # 회전 속도, 필요에 따라 조정
        self.cmd_vel_publisher.publish(twist)
        # 여기서 일정 시간 동안 회전을 수행합니다. 실제로는 오도메트리를 확인해야 합니다.
        rclpy.spin_once(self, timeout_sec=4.2)  # π 초 동안 회전 (예시)
        self.stop_robot()
        self.get_logger().info("회전 완료")

    def publish_state(self):
        self.state_publisher.publish(String(data=self.state))

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
