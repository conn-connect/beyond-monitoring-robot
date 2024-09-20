import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from beyond_robotics_interfaces.msg import LaneGuidance
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class DemoRobotControlNode(Node):
    def __init__(self):
        super().__init__('demo_robot_control_node')
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        self.debug_image_pub = self.create_publisher(Image, 'debug_image', 10)
        self.picking_command_publisher = self.create_publisher(String, 'picking_command', 10)

        # Subscribers
        self.create_subscription(Image, 'camera/front/color/image_raw', self.front_image_callback, 10)
        self.create_subscription(Image, 'camera/front/depth/image_raw', self.front_depth_callback, 10)
        self.create_subscription(CameraInfo, 'camera/front/camera_info', self.front_camera_info_callback, 10)
        self.create_subscription(Image, 'camera/side/color/image_raw', self.side_image_callback, 10)
        self.create_subscription(Image, 'camera/side/depth/image_raw', self.side_depth_callback, 10)
        self.create_subscription(CameraInfo, 'camera/side/camera_info', self.side_camera_info_callback, 10)
        self.create_subscription(String, 'keyboard_input', self.keyboard_input_callback, 10)
        self.create_subscription(LaneGuidance, 'lane_guidance', self.lane_guidance_callback, 10)
        self.create_subscription(String, 'picking_status', self.picking_status_callback, 10)

        # ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Camera intrinsics
        self.front_camera_matrix = None
        self.front_dist_coeffs = None
        self.front_rs_intrinsics = None
        self.front_depth_scale = 0.001
        
        self.side_camera_matrix = None
        self.side_dist_coeffs = None
        self.side_rs_intrinsics = None
        self.side_depth_scale = 0.0001
        
        # Latest color and depth images
        self.front_color_image = None
        self.front_depth_image = None
        self.side_color_image = None
        self.side_depth_image = None
        
        # Robot state
        self.state = 'WAITING_TO_START'
        self.current_aruco_id = None
        self.current_aruco_position = None
        self.target_aruco_ids = [0, 1, 2]
        self.current_target = 0

        self.move_start_time = None
        self.move_duration = None

        # Lane guidance
        self.current_angular_z = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)
    
    def picking_status_callback(self, msg):
        if msg.data == 'picking_started':
            self.get_logger().info('Robot arm has started picking.')
            # If needed, adjust autonomous platform behavior
        elif msg.data == 'picking_completed':
            self.get_logger().info('Robot arm has completed picking.')
            # Proceed to the next waypoint or final destination
            self.state = 'MOVING_TO_NEXT_POSITION'
    
    def send_picking_command(self, command):
        msg = String()
        msg.data = command
        self.picking_command_publisher.publish(msg)
        self.get_logger().info(f'Sent picking command: {command}')

    def front_camera_info_callback(self, msg):
        self.front_camera_matrix = np.array(msg.k).reshape(3, 3)
        self.front_dist_coeffs = np.array(msg.d)
        
        self.front_rs_intrinsics = rs.intrinsics()
        self.front_rs_intrinsics.width = msg.width
        self.front_rs_intrinsics.height = msg.height
        self.front_rs_intrinsics.ppx = self.front_camera_matrix[0, 2]
        self.front_rs_intrinsics.ppy = self.front_camera_matrix[1, 2]
        self.front_rs_intrinsics.fx = self.front_camera_matrix[0, 0]
        self.front_rs_intrinsics.fy = self.front_camera_matrix[1, 1]
        self.front_rs_intrinsics.model = rs.distortion.brown_conrady
        self.front_rs_intrinsics.coeffs = [float(i) for i in msg.d]

    def front_depth_callback(self, msg):
        self.front_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def front_image_callback(self, msg):
        self.front_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.front_depth_image is not None and self.front_rs_intrinsics is not None:
            self.process_front_image()

    def side_camera_info_callback(self, msg):
        self.side_camera_matrix = np.array(msg.k).reshape(3, 3)
        self.side_dist_coeffs = np.array(msg.d)
        
        self.side_rs_intrinsics = rs.intrinsics()
        self.side_rs_intrinsics.width = msg.width
        self.side_rs_intrinsics.height = msg.height
        self.side_rs_intrinsics.ppx = self.side_camera_matrix[0, 2]
        self.side_rs_intrinsics.ppy = self.side_camera_matrix[1, 2]
        self.side_rs_intrinsics.fx = self.side_camera_matrix[0, 0]
        self.side_rs_intrinsics.fy = self.side_camera_matrix[1, 1]
        self.side_rs_intrinsics.model = rs.distortion.brown_conrady
        self.side_rs_intrinsics.coeffs = [float(i) for i in msg.d]

    def side_depth_callback(self, msg):
        self.side_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def side_image_callback(self, msg):
        self.side_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.side_depth_image is not None and self.side_rs_intrinsics is not None:
            self.process_side_image()

    def lane_guidance_callback(self, msg):
        self.current_angular_z = msg.angular_z

    def process_front_image(self):
        if self.state == 'APPROACHING_FINAL_ARUCO':
            gray = cv2.cvtColor(self.front_color_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None and self.target_aruco_ids[2] in ids:
                index = np.where(ids == self.target_aruco_ids[2])[0][0]
                marker_corners = corners[index][0]
                marker_center = np.mean(marker_corners, axis=0).astype(int)
                
                depth = self.get_marker_depth(self.front_depth_image, marker_center)
                if depth is not None:
                    marker_3d = rs.rs2_deproject_pixel_to_point(self.front_rs_intrinsics, marker_center, depth)
                    self.current_aruco_id = self.target_aruco_ids[2]
                    self.current_aruco_position = np.array(marker_3d)
                    
                    distance = np.linalg.norm(self.current_aruco_position)
                    if distance <= 0.8:  # 80cm
                        self.state = 'MISSION_COMPLETE'
                        self.get_logger().info("Final ArUco reached. Mission Complete.")
                    
                    self.visualize_aruco(self.front_color_image, corners, ids)
                    self.publish_debug_image(self.front_color_image)

    def process_side_image(self):
        if self.state in ['FOLLOWING_LANE', 'APPROACHING_SIDE_ARUCO']:
            gray = cv2.cvtColor(self.side_color_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                for i, id in enumerate(ids):
                    if id[0] in self.target_aruco_ids[:2]:
                        marker_corners = corners[i][0]
                        marker_center = np.mean(marker_corners, axis=0).astype(int)
                        
                        depth = self.get_marker_depth(self.side_depth_image, marker_center)
                        if depth is not None:
                            marker_3d = rs.rs2_deproject_pixel_to_point(self.side_rs_intrinsics, marker_center, depth)
                            self.current_aruco_id = id[0]
                            self.current_aruco_position = np.array(marker_3d)
                            
                            if abs(self.current_aruco_position[0]) < 0.05:  # If ArUco is within 5cm of the center
                                self.state = 'STOPPING_AT_SIDE_ARUCO'
                                self.get_logger().info(f"Stopping at Side ArUco: ID {self.current_aruco_id}")
                            
                            self.visualize_aruco(self.side_color_image, corners, ids)
                            self.publish_debug_image(self.side_color_image)
                            break

    def get_marker_depth(self, depth_image, center):
        kernel_size = 5
        depth_region = depth_image[
            max(0, center[1] - kernel_size // 2):min(depth_image.shape[0], center[1] + kernel_size // 2 + 1),
            max(0, center[0] - kernel_size // 2):min(depth_image.shape[1], center[0] + kernel_size // 2 + 1)
        ]
        valid_depths = depth_region[depth_region > 0]
        if len(valid_depths) > 0:
            return np.median(valid_depths) * self.front_depth_scale
        return None

    def visualize_aruco(self, image, corners, ids):
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i, id in enumerate(ids):
            if id[0] in self.target_aruco_ids:
                marker_center = np.mean(corners[i][0], axis=0).astype(int)
                cv2.putText(image, f"ID: {id[0]}", (marker_center[0], marker_center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def keyboard_input_callback(self, msg):
        if msg.data == 'start' and self.state == 'WAITING_TO_START':
            self.state = 'FOLLOWING_LANE'
            self.get_logger().info("Robot movement started")
        elif msg.data == 'continue' and self.state == 'STOPPED_AT_SIDE_ARUCO':
            self.state = 'MOVING_AFTER_SIDE_ARUCO'
            self.move_start_time = self.get_clock().now()
            self.move_duration = rclpy.duration.Duration(seconds=2.5)  # Time to move 50cm at 0.2 m/s
            self.get_logger().info("Continuing after Side ArUco")
        elif msg.data == 'stop':
            self.state = 'EMERGENCY_STOP'
            self.stop_robot()
            self.get_logger().info("Emergency stop activated")

    def control_loop(self):
        twist = Twist()
        
        if self.state == 'FOLLOWING_LANE':
            twist.linear.x = 0.2  # Constant forward speed
            twist.angular.z = self.current_angular_z
        elif self.state == 'STOPPING_AT_SIDE_ARUCO':
            self.stop_robot()
            self.state = 'STOPPED_AT_SIDE_ARUCO'
            self.get_logger().info(f"Stopped at Side ArUco: ID {self.current_aruco_id}")
        elif self.state == 'MOVING_AFTER_SIDE_ARUCO':
            current_time = self.get_clock().now()
            if current_time - self.move_start_time < self.move_duration:
                twist.linear.x = 0.2  # Move forward at 0.2 m/s
            else:
                self.stop_robot()
                self.current_target += 1
                if self.current_target < 2:
                    self.state = 'FOLLOWING_LANE'
                    self.get_logger().info("Resuming lane following")
                else:
                    self.state = 'APPROACHING_FINAL_ARUCO'
                    self.get_logger().info("Approaching final ArUco")
        elif self.state == 'APPROACHING_FINAL_ARUCO':
            twist.linear.x = 0.2  # Slower speed when approaching final ArUco
            twist.angular.z = self.current_angular_z
        elif self.state == 'MISSION_COMPLETE' or self.state == 'EMERGENCY_STOP':
            self.stop_robot()

        self.cmd_vel_publisher.publish(twist)
        self.publish_state()

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

    def publish_state(self):
        self.state_publisher.publish(String(data=self.state))

    def publish_debug_image(self, image):
        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.debug_image_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DemoRobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()