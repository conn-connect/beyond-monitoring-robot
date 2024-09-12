import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import Float32
from beyond_robotics_interfaces.msg import LaneGuidance
from geometry_msgs.msg import Point

class ArUcoPathFollowingNode(Node):
    def __init__(self):
        super().__init__('aruco_path_following_node')
        self.bridge = CvBridge()
                
        # Publishers
        self.debug_image_pub = self.create_publisher(Image, 'debug_image', 10)
        self.lane_guidance_pub = self.create_publisher(LaneGuidance, 'lane_guidance', 10)
        
        # Subscribers
        self.create_subscription(Image, 'camera/front/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, 'camera/front/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, 'camera/front/camera_info', self.camera_info_callback, 10)
        
        # ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rs_intrinsics = None
        self.depth_scale = 0.001
        
        # Latest color and depth images
        self.color_image = None
        self.depth_image = None
        
        # Path parameters
        self.offset_distance = 0.65  # 58cm offset
        self.path_extension = 0.9  # 90cm extension on both ends

        # Control parameters
        self.max_angular_speed = 0.5  # Maximum angular speed (rad/s)
        self.max_distance_error = 0.3  # Maximum distance error to consider (meters)
        self.lookahead_distance = 1.5  # Lookahead distance for path following (meters)

        # PID control parameters
        self.Kp = 3.0  # Proportional gain
        self.Ki = 0.01  # Integral gain
        self.Kd = 1.0  # Derivative gain
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.max_integral = 1.0  # Maximum integral term to prevent windup

        # Target ArUco marker IDs
        self.target_ids = [0, 1]

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        
        self.rs_intrinsics = rs.intrinsics()
        self.rs_intrinsics.width = msg.width
        self.rs_intrinsics.height = msg.height
        self.rs_intrinsics.ppx = self.camera_matrix[0, 2]
        self.rs_intrinsics.ppy = self.camera_matrix[1, 2]
        self.rs_intrinsics.fx = self.camera_matrix[0, 0]
        self.rs_intrinsics.fy = self.camera_matrix[1, 1]
        self.rs_intrinsics.model = rs.distortion.brown_conrady
        self.rs_intrinsics.coeffs = [float(i) for i in msg.d]

        # self.get_logger().info(f"Camera intrinsics updated: {self.rs_intrinsics}")


    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.depth_image is not None and self.rs_intrinsics is not None:
            self.process_image()
            
    def process_image(self):
        if self.color_image is None or self.depth_image is None:
            return

        gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        debug_image = self.color_image.copy()
        angular_z = 0.0  # Default value when no path is detected

        if ids is None:
            self.get_logger().warn("No ArUco markers detected")
            self.publish_debug_image(debug_image)
            self.publish_guidance(angular_z)
            return

        target_corners = []
        target_ids = []
        for corner, id in zip(corners, ids):
            if id[0] in self.target_ids:
                target_corners.append(corner)
                target_ids.append(id)

        target_ids = np.array(target_ids)  # Convert to numpy array

        if len(target_corners) < 2:
            self.get_logger().warn(f"Not enough target markers detected: {len(target_corners)}")
            self.publish_debug_image(debug_image)
            self.publish_guidance(angular_z)
            return

        cv2.aruco.drawDetectedMarkers(debug_image, target_corners, target_ids)

        marker_positions = self.get_marker_positions(target_corners)

        if not marker_positions or len(marker_positions) < 2:
            self.get_logger().warn(f"Not enough valid marker positions: {len(marker_positions) if marker_positions else 0}")
            self.publish_debug_image(debug_image)
            self.publish_guidance(angular_z)
            return

        path_start, path_end, offset_start, offset_end = self.calculate_path(marker_positions)

        if path_start is None or path_end is None:
            self.get_logger().warn("Failed to calculate path")
            self.publish_debug_image(debug_image)
            self.publish_guidance(angular_z)
            return

        angular_z = self.calculate_guidance(offset_start, offset_end)
        self.visualize_markers_and_path(debug_image, marker_positions, target_ids, path_start, path_end, offset_start, offset_end, angular_z)

        # Draw angular_z value on the debug image
        cv2.putText(debug_image, f"Angular Z: {angular_z:.4f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        self.publish_debug_image(debug_image)
        self.publish_guidance(angular_z)

    def calculate_guidance(self, path_start, path_end):
        # Calculate path direction vector in xz plane
        path_vector = path_end - path_start
        path_vector[1] = 0  # Ensure y component is 0
        path_direction = path_vector / np.linalg.norm(path_vector)

        # Project robot's current position (assume it's at the camera position)
        robot_position = np.array([0, 0, 0])  # Assuming camera frame

        # Calculate the closest point on the path to the robot
        t = np.dot(robot_position - path_start, path_direction)
        closest_point = path_start + t * path_direction

        # Calculate lookahead point
        lookahead_point = closest_point + self.lookahead_distance * path_direction

        # Calculate the error vector (perpendicular distance to the path)
        error_vector = closest_point - robot_position
        error_vector[1] = 0  # Ignore y component
        distance_error = np.linalg.norm(error_vector)

        # Calculate the angle between the robot's forward direction and the path direction
        forward_vector = np.array([0, 0, 1])  # Assuming camera's z-axis is forward
        angle_to_lookahead = np.arctan2(lookahead_point[0] - robot_position[0], lookahead_point[2] - robot_position[2])

        # Calculate the desired heading
        desired_heading = angle_to_lookahead

        # Calculate heading error
        heading_error = desired_heading

        # Combine distance and heading errors with increased weight on distance error
        total_error = heading_error + 2.0 * distance_error  # Increased weight on distance error

        # Update integral term
        self.integral_error += total_error
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)

        # Calculate derivative term
        derivative_error = total_error - self.previous_error
        self.previous_error = total_error

        # Adjust PID gains for more aggressive behavior
        Kp = 2.0  # Increased proportional gain
        Ki = 0.03  # Slightly increased integral gain
        Kd = 0.15  # Increased derivative gain

        # Calculate PID control output
        pid_output = (Kp * total_error +
                    Ki * self.integral_error +
                    Kd * derivative_error)

        # Calculate angular velocity
        angular_z = pid_output

        # Increase the maximum angular speed for faster response
        max_angular_speed = 0.5  # Increased from 0.3 to 0.5

        # Limit the angular velocity
        angular_z = -1.0 * np.clip(angular_z, -max_angular_speed, max_angular_speed)

        self.get_logger().info(f"Angular z: {angular_z:.2f}, Heading error: {heading_error:.2f}, Distance error: {distance_error:.2f}")

        return angular_z

    def publish_guidance(self, angular_z):
        # Create and publish LaneGuidance message
        msg = LaneGuidance()
        msg.angular_z = float(angular_z)
        self.lane_guidance_pub.publish(msg)

    def get_marker_positions(self, corners):
        marker_positions = []
        blurred_depth = cv2.medianBlur(self.depth_image, 5)

        for i, corner in enumerate(corners):
            marker_corners = corner[0]
            marker_center = np.mean(marker_corners, axis=0).astype(int)
            
            x_min, y_min = np.min(marker_corners, axis=0).astype(int)
            x_max, y_max = np.max(marker_corners, axis=0).astype(int)
            marker_region = blurred_depth[y_min:y_max, x_min:x_max]

            valid_depths = marker_region[marker_region > 0]

            if len(valid_depths) > 0:
                median_depth = np.median(valid_depths)
                
                kernel_size = 5
                center_region = blurred_depth[
                    max(0, marker_center[1] - kernel_size // 2):min(blurred_depth.shape[0], marker_center[1] + kernel_size // 2 + 1),
                    max(0, marker_center[0] - kernel_size // 2):min(blurred_depth.shape[1], marker_center[0] + kernel_size // 2 + 1)
                ]
                center_valid_depths = center_region[center_region > 0]
                
                if len(center_valid_depths) > 0:
                    depth_value = np.median(center_valid_depths)
                else:
                    depth_value = median_depth

                depth_meters = depth_value * self.depth_scale

                try:
                    marker_3d = rs.rs2_deproject_pixel_to_point(
                        self.rs_intrinsics, 
                        [float(marker_center[0]), float(marker_center[1])],
                        depth_meters
                    )
                    # Set y to 0 to consider only xz plane
                    marker_positions.append(np.array([marker_3d[0], 0, marker_3d[2]]))
                except Exception as e:
                    self.get_logger().error(f"Error in rs2_deproject_pixel_to_point: {e}")
            else:
                self.get_logger().warn(f"Invalid depth for marker {i} at pixel ({marker_center[0]}, {marker_center[1]})")

        if len(marker_positions) > 0:
            return marker_positions
        else:
            self.get_logger().warn("No valid marker positions found")
            return []

    def calculate_path(self, marker_positions):
        if len(marker_positions) == 2:
            marker1, marker2 = marker_positions[:2]
            path_direction = marker2 - marker1
            path_direction[1] = 0  # Ensure y component is 0
            path_direction = path_direction / np.linalg.norm(path_direction)
            path_start = marker1
            path_end = marker2

            offset_direction = np.array([-path_direction[2], 0, path_direction[0]])
            
            offset_start = path_start + offset_direction * self.offset_distance
            offset_end = path_end + offset_direction * self.offset_distance

            # Extend the path by 50cm on both ends
            extended_offset_start = offset_start - path_direction * self.path_extension
            extended_offset_end = offset_end + path_direction * self.path_extension

            return path_start, path_end, extended_offset_start, extended_offset_end
        
        return None, None, None, None

    def visualize_markers_and_path(self, image, marker_positions, marker_ids, path_start, path_end, offset_start, offset_end, angular_z):
        for i, (marker, marker_id) in enumerate(zip(marker_positions, marker_ids)):
            marker_2d = self.project_3d_to_2d(marker)
            cv2.circle(image, tuple(marker_2d.astype(int)), 5, (0, 0, 255), -1)
            
            # Convert 3D coordinates to string
            coords_str = self.coords_to_string(marker)
            
            # Draw marker ID and 3D coordinates
            cv2.putText(image, f"ID: {marker_id}", (int(marker_2d[0]), int(marker_2d[1]) - 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(image, coords_str, (int(marker_2d[0]), int(marker_2d[1]) - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if path_start is not None and path_end is not None:
            # Draw the line between markers (no offset) in green
            start_2d = self.project_3d_to_2d(path_start)
            end_2d = self.project_3d_to_2d(path_end)
            cv2.line(image, tuple(start_2d.astype(int)), tuple(end_2d.astype(int)), (0, 255, 0), 2)
            
            # Draw the extended offset path in blue
            offset_start_2d = self.project_3d_to_2d(offset_start)
            offset_end_2d = self.project_3d_to_2d(offset_end)
            cv2.line(image, tuple(offset_start_2d.astype(int)), tuple(offset_end_2d.astype(int)), (255, 0, 0), 2)
            cv2.arrowedLine(image, tuple(offset_start_2d.astype(int)), tuple(offset_end_2d.astype(int)), (255, 0, 0), 2, tipLength=0.05)
            
            # Calculate line length of the extended offset path
            line_length = np.linalg.norm(offset_end - offset_start)
            length_str = f"{line_length:.2f}m"
            
            # Calculate midpoint for text placement
            mid_point = (offset_start_2d + offset_end_2d) / 2
            
            # Draw length text above the line
            cv2.putText(image, length_str, (int(mid_point[0]), int(mid_point[1]) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        cv2.putText(image, f"Angular Z: {angular_z:.4f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    def coords_to_string(self, coords):
        return f"X: {coords[0]:.2f}, Y: {coords[1]:.2f}, Z: {coords[2]:.2f}"

    def project_3d_to_2d(self, point_3d):
        point_3d = np.array([point_3d], dtype=np.float32)
        point_2d, _ = cv2.projectPoints(point_3d, np.zeros(3), np.zeros(3), self.camera_matrix, self.dist_coeffs)
        return point_2d[0][0]

    def publish_debug_image(self, image):
        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.debug_image_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoPathFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()