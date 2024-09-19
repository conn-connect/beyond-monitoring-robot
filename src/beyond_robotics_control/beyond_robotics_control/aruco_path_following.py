import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import Float32  # Add this import

class ArUcoPathFollowingNode(Node):
    def __init__(self):
        super().__init__('aruco_path_following_node')
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = self.create_publisher(Image, 'debug_image', 10)
        
        # Subscribers for front camera
        self.create_subscription(Image, 'camera/front/color/image_raw', self.front_image_callback, 10)
        self.create_subscription(Image, 'camera/front/depth/image_raw', self.front_depth_callback, 10)
        self.create_subscription(CameraInfo, 'camera/front/camera_info', self.front_camera_info_callback, 10)
        
        # New: Subscribers for side camera
        self.create_subscription(Image, 'camera/side/color/image_raw', self.side_image_callback, 10)
        self.create_subscription(Image, 'camera/side/depth/image_raw', self.side_depth_callback, 10)
        self.create_subscription(CameraInfo, 'camera/side/camera_info', self.side_camera_info_callback, 10)

        # ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Camera intrinsics
<<<<<<< HEAD
        self.front_camera_matrix = None
        self.front_dist_coeffs = None
        self.front_rs_intrinsics = None
        self.front_depth_scale = 0.001
        
        # New: Side camera intrinsics
        self.side_camera_matrix = None
        self.side_dist_coeffs = None
        self.side_rs_intrinsics = None
        self.side_depth_scale = 0.0001
=======
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rs_intrinsics = None
>>>>>>> 3476c19274e5a8000dc06294fed2541c695fb63e
        
        # Latest color and depth images
        self.front_color_image = None
        self.front_depth_image = None
        
        # New: Latest side camera color and depth images
        self.side_color_image = None
        self.side_depth_image = None
        
        # Path parameters
<<<<<<< HEAD
        self.offset_distance = 0.65  # 65cm offset
        self.path_extension = 0.5  # 50cm extension on both ends

        # Control parameters
        self.max_angular_speed = 0.4  # Maximum angular speed (rad/s)
        self.max_distance_error = 0.3  # Maximum distance error to consider (meters)
        self.lookahead_distance = 1.5  # Lookahead distance for path following (meters)

        # PID control parameters
        self.Kp = 3.0  # Proportional gain
        self.Ki = 0.01  # Integral gain
        self.Kd = 1.0  # Derivative gain
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.max_integral = 1.0  # Maximum integral term to prevent windup

        # Initialize PID variables for distance and orientation
        self.integral_error_dist = 0.0
        self.previous_error_dist = 0.0
        self.integral_error_orient = 0.0
        self.previous_error_orient = 0.0

        self.previous_angular_z = 0.0  # 이전의 angular_z 값을 저장하기 위한 변수

=======
        self.offset_distance = 0.0  # 70cm offset
        
>>>>>>> 3476c19274e5a8000dc06294fed2541c695fb63e
        # Target ArUco marker IDs
        self.target_ids = [0, 1]
        self.final_aruco_id = 2

        # State variable
        self.state = 'INITIAL_ALIGNMENT'  # 'INITIAL_ALIGNMENT', 'BLUE_TAPE_FOLLOWING', 'FINAL_ARUCO_APPROACH'

<<<<<<< HEAD
        # Blue tape detection parameters
        self.blue_lower = np.array([100, 50, 50])
        self.blue_upper = np.array([140, 255, 255])
=======
        # Add subscriber for depth scale
        self.depth_scale = 0.001

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
>>>>>>> 3476c19274e5a8000dc06294fed2541c695fb63e
        
        # Target distance for blue tape following (33cm)
        self.target_distance = 0.33

<<<<<<< HEAD
=======
        self.get_logger().info(f"Camera intrinsics updated: {self.rs_intrinsics}")
>>>>>>> 3476c19274e5a8000dc06294fed2541c695fb63e

    # Front camera callbacks
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
            self.process_image()

    # New: Side camera callbacks
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

        # Now, detect ArUco markers in the side image
        gray = cv2.cvtColor(self.side_color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            ids = ids.flatten()
            if self.state == 'INITIAL_ALIGNMENT' and 0 in ids:
                self.state = 'BLUE_TAPE_FOLLOWING'
                self.get_logger().info("Switching to BLUE_TAPE_FOLLOWING state (ArUco ID 0 detected in side camera)")
            elif self.state == 'BLUE_TAPE_FOLLOWING' and 1 in ids:
                # switch state only if id=1 marker is on the right side of the camera
                index = np.where(ids == 1)[0][0]
                marker_center = np.mean(corners[index][0], axis=0)
                if marker_center[0] > self.side_color_image.shape[1] * 2 / 3:
                    self.state = 'FINAL_ARUCO_APPROACH'
                    self.get_logger().info("Switching to FINAL_ARUCO_APPROACH state (ArUco ID 1 detected in side camera)")

        
    # Modified: process_image method
    def process_image(self):
        if self.front_color_image is None or self.front_depth_image is None:
            return

<<<<<<< HEAD
        debug_image = self.front_color_image.copy()
        angular_z = 0.0  # Default value when no guidance is available

        if self.state == 'INITIAL_ALIGNMENT':
            angular_z = self.process_initial_alignment(debug_image)
        elif self.state == 'BLUE_TAPE_FOLLOWING':
            if self.side_color_image is not None and self.side_depth_image is not None:
                debug_image = self.side_color_image.copy()
                angular_z = self.process_blue_tape_following(debug_image)
            else:
                self.get_logger().warn("Side camera images not available for blue tape following")
        elif self.state == 'FINAL_ARUCO_APPROACH':
            angular_z = self.process_final_aruco_approach(debug_image)

        # Draw state and angular_z value on the debug image
        cv2.putText(debug_image, f"State: {self.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        cv2.putText(debug_image, f"Angular Z: {angular_z:.4f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        self.publish_debug_image(debug_image)
        self.publish_guidance(angular_z)

    # New: process_initial_alignment method
    def process_initial_alignment(self, debug_image):
        gray = cv2.cvtColor(self.front_color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            self.get_logger().warn("No ArUco markers detected")
            return 0.0

        target_corners = []
        target_ids = []
        for corner, id in zip(corners, ids):
            if id[0] in self.target_ids:
                target_corners.append(corner)
                target_ids.append(id)

        target_ids = np.array(target_ids)

        if len(target_corners) < 2:
            self.get_logger().warn(f"Not enough target markers detected: {len(target_corners)}")
            return 0.0

        cv2.aruco.drawDetectedMarkers(debug_image, target_corners, target_ids)

        marker_positions = self.get_marker_positions(target_corners)

        if not marker_positions or len(marker_positions) < 2:
            self.get_logger().warn(f"Not enough valid marker positions: {len(marker_positions) if marker_positions else 0}")
            return 0.0

        path_start, path_end, offset_start, offset_end = self.calculate_path(marker_positions)

        if path_start is None or path_end is None:
            self.get_logger().warn("Failed to calculate path")
            return 0.0

        angular_z = self.calculate_guidance(offset_start, offset_end)
        self.visualize_markers_and_path(debug_image, marker_positions, target_ids, path_start, path_end, offset_start, offset_end)

        # Check if we're close enough to the path to switch to blue tape following
        robot_position = np.array([0, 0, 0])  # Assuming camera frame
        distance_to_path = np.linalg.norm(np.cross(offset_end - offset_start, robot_position - offset_start)) / np.linalg.norm(offset_end - offset_start)
        
        # if distance_to_path < 0.1:  # If within 10cm of the path
        #     self.state = 'BLUE_TAPE_FOLLOWING'
        #     self.get_logger().info("Switching to BLUE_TAPE_FOLLOWING state")

        return angular_z
    

    def process_blue_tape_following(self, debug_image):
        hsv = cv2.cvtColor(self.side_color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        # Limit the mask to the vertical region where the tape is expected
        mask[:180, :] = 0  # Zero out rows above pixel 180
        mask[250:, :] = 0  # Zero out rows below pixel 250

        # Find contours of blue tape within the specified region
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Select the largest contour (assumed to be the blue tape)
            largest_contour = max(contours, key=cv2.contourArea)

            # Compute the bounding rectangle with padding
            x, y, w, h = cv2.boundingRect(largest_contour)
            padding = 10  # Adjust padding as needed
            x_start = max(0, x - padding)
            x_end = min(self.side_depth_image.shape[1], x + w + padding)
            y_start = max(0, y - padding)
            y_end = min(self.side_depth_image.shape[0], y + h + padding)
            # Split the rectangle into front and back halves
            mid_x = x_start + (x_end - x_start) // 2

            # Front half (closer to the robot's front)
            depth_region_front = self.side_depth_image[y_start:y_end, x_start:mid_x]
            valid_depths_front = depth_region_front[depth_region_front > 0]

            # Back half (closer to the robot's back)
            depth_region_back = self.side_depth_image[y_start:y_end, mid_x:x_end]
            valid_depths_back = depth_region_back[depth_region_back > 0]

            if len(valid_depths_front) == 0 or len(valid_depths_back) == 0:
                self.get_logger().warn("No valid depth values in one or both halves")
                return 0.0

            # Compute median depths
            depth_front = np.median(valid_depths_front) * self.side_depth_scale
            depth_back = np.median(valid_depths_back) * self.side_depth_scale

            # Calculate the average tape distance
            tape_distance = (depth_front + depth_back) / 2

            # Calculate the depth difference
            depth_difference = (depth_back - depth_front) * 2.0  # Positive if back is further than front

            # Log the values
            self.get_logger().info(f"Depth Front: {depth_front:.2f}m, Depth Back: {depth_back:.2f}m, Depth Difference: {depth_difference:.2f}m")

            # Calculate distance error
            distance_error = self.target_distance - tape_distance

            # Calculate orientation error
            orientation_error = depth_difference  # Use depth difference directly

            # Adjust PID gains or introduce a separate controller for orientation
            angular_z = self.calculate_pid_combined(distance_error, orientation_error)

            # Limit angular_z to prevent excessive values
            max_angular_speed = 0.5  # Adjust as needed
            angular_z = np.clip(angular_z, -max_angular_speed, max_angular_speed)

            # Visualize blue tape detection
            cv2.drawContours(debug_image, [largest_contour], 0, (0, 255, 0), 2)
            cv2.rectangle(debug_image, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)
            cv2.line(debug_image, (mid_x, y_start), (mid_x, y_end), (0, 0, 255), 2)  # Draw the split line
            cv2.putText(debug_image, f"Distance: {tape_distance:.2f}m", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
            cv2.putText(debug_image, f"Angular Z: {angular_z:.4f}", (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

            return angular_z

        self.get_logger().warn("Blue tape not detected")
        return 0.0

    def process_final_aruco_approach(self, debug_image):
        gray = cv2.cvtColor(self.front_color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None and self.final_aruco_id in ids:
            index = np.where(ids == self.final_aruco_id)[0][0]
            marker_center = np.mean(corners[index][0], axis=0)
            
            # 마커 중심과 이미지 중심의 차이로부터 오차 계산
            image_center = self.front_color_image.shape[1] / 2
            error = marker_center[0] - image_center  # 픽셀 단위 오차
            
            # 오차에 비례하여 angular_z 계산
            gain = 0.001  # 필요에 따라 조정
            angular_z = -gain * error
            
            # 스무딩 적용
            alpha = 0.7  # 스무딩 계수 (0과 1 사이, 높을수록 부드러움)
            angular_z = alpha * self.previous_angular_z + (1 - alpha) * angular_z
            self.previous_angular_z = angular_z  # 이전 angular_z 업데이트
            
            # angular_z 제한
            max_angular_speed = 0.3  # 필요에 따라 조정
            angular_z = np.clip(angular_z, -max_angular_speed, max_angular_speed)
            
            # 마커 시각화
            cv2.aruco.drawDetectedMarkers(debug_image, [corners[index]], np.array([[self.final_aruco_id]]))
            cv2.putText(debug_image, f"ID: {self.final_aruco_id}", (int(marker_center[0]), int(marker_center[1]) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            return angular_z
        
        self.get_logger().warn(f"Final ArUco marker (ID {self.final_aruco_id}) not detected")
        return 0.0


    # New: check_end_of_tape method
    def check_end_of_tape(self, mask):
        # Check the number of blue pixels in the bottom part of the image
        bottom_region = mask[int(mask.shape[0]*0.9):, :]
        return np.sum(bottom_region) < 100  # Adjust threshold as needed

    # New: calculate_pid method
    def calculate_pid(self, error):
        self.integral_error += error
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        derivative_error = error - self.previous_error
        self.previous_error = error
        
        return self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error

    def calculate_pid_combined(self, distance_error, orientation_error):
        # Parameters for distance control
        Kp_dist = 2.0
        Ki_dist = 0.05
        Kd_dist = 0.1

        # Parameters for orientation control
        Kp_orient = 3.0
        Ki_orient = 0.05
        Kd_orient = 0.1

        # Calculate PID for distance
        self.integral_error_dist += distance_error
        self.integral_error_dist = np.clip(self.integral_error_dist, -self.max_integral, self.max_integral)
        derivative_error_dist = distance_error - self.previous_error_dist
        self.previous_error_dist = distance_error

        pid_output_dist = Kp_dist * distance_error + Ki_dist * self.integral_error_dist + Kd_dist * derivative_error_dist

        # Calculate PID for orientation
        self.integral_error_orient += orientation_error
        self.integral_error_orient = np.clip(self.integral_error_orient, -self.max_integral, self.max_integral)
        derivative_error_orient = orientation_error - self.previous_error_orient
        self.previous_error_orient = orientation_error

        pid_output_orient = Kp_orient * orientation_error + Ki_orient * self.integral_error_orient + Kd_orient * derivative_error_orient

        # Combine the two control outputs
        total_pid_output = pid_output_dist + pid_output_orient

        # Log the PID terms
        self.get_logger().info(f"PID Distance Output: {pid_output_dist:.2f}, PID Orientation Output: {pid_output_orient:.2f}")

        return total_pid_output


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
        Kd = 0.1  # Increased derivative gain

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

=======
        gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        debug_image = self.color_image.copy()

        if ids is not None:
            # Filter out markers that are not in target_ids
            target_corners = []
            target_ids = []
            for corner, id in zip(corners, ids):
                if id[0] in self.target_ids:
                    target_corners.append(corner)
                    target_ids.append(id[0])
            
            target_ids = np.array(target_ids)
            
            if len(target_corners) > 0:
                cv2.aruco.drawDetectedMarkers(debug_image, target_corners, target_ids)

            if len(target_corners) >= 1:
                marker_positions = self.get_marker_positions(target_corners)
                
                if marker_positions and len(marker_positions) >= 1:
                    path_start, path_end = self.calculate_path(marker_positions)
                    if path_start is not None and path_end is not None:
                        self.visualize_markers_and_path(debug_image, marker_positions, target_ids, path_start, path_end)
                    else:
                        self.get_logger().warn("Failed to calculate path")
                else:
                    self.get_logger().warn(f"Not enough valid marker positions: {len(marker_positions) if marker_positions else 0}")
            else:
                self.get_logger().warn(f"Not enough target markers detected: {len(target_corners)}")
        else:
            self.get_logger().warn("No ArUco markers detected")

        self.publish_debug_image(debug_image)
    # The rest of the methods remain unchanged
    
>>>>>>> 3476c19274e5a8000dc06294fed2541c695fb63e
    def get_marker_positions(self, corners):
        marker_positions = []
        blurred_depth = cv2.medianBlur(self.front_depth_image, 5)

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

<<<<<<< HEAD
                depth_meters = depth_value * self.front_depth_scale

                if self.front_rs_intrinsics is not None:  # Add this check
                    try:
                        marker_3d = rs.rs2_deproject_pixel_to_point(
                            self.front_rs_intrinsics, 
                            [float(marker_center[0]), float(marker_center[1])],
                            depth_meters
                        )
                        # Set y to 0 to consider only xz plane
                        marker_positions.append(np.array([marker_3d[0], 0, marker_3d[2]]))
                    except Exception as e:
                        self.get_logger().error(f"Error in rs2_deproject_pixel_to_point: {e}")
                else:
                    self.get_logger().warn("front_rs_intrinsics is not initialized")
=======
                depth_meters = depth_value * self.depth_scale

                self.get_logger().info(f"Marker {i}: Raw depth: {depth_value}, Depth in meters: {depth_meters}")

                try:
                    marker_3d = rs.rs2_deproject_pixel_to_point(
                        self.rs_intrinsics, 
                        [float(marker_center[0]), float(marker_center[1])],
                        depth_meters
                    )
                    self.get_logger().info(f"Marker {i}: 3D position: {marker_3d}")
                    marker_positions.append(np.array(marker_3d))
                except Exception as e:
                    self.get_logger().error(f"Error in rs2_deproject_pixel_to_point: {e}")
>>>>>>> 3476c19274e5a8000dc06294fed2541c695fb63e
            else:
                self.get_logger().warn(f"Invalid depth for marker {i} at pixel ({marker_center[0]}, {marker_center[1]})")

        if len(marker_positions) > 0:
            return marker_positions
        else:
            self.get_logger().warn("No valid marker positions found")
            return []


    def calculate_path(self, marker_positions):
        if len(marker_positions) == 0:
            return None, None

        if len(marker_positions) == 1:
            marker = marker_positions[0]
            default_direction = np.array([0, 0, 1])
            path_direction = default_direction
            path_start = marker
            path_end = marker + default_direction
        else:
            marker1, marker2 = marker_positions[:2]
            path_direction = (marker2 - marker1) / np.linalg.norm(marker2 - marker1)
            path_start = marker1
            path_end = marker2

        offset_direction = np.array([-path_direction[2], 0, path_direction[0]])
        
        path_start += offset_direction * self.offset_distance
        path_end += offset_direction * self.offset_distance

        return path_start, path_end


<<<<<<< HEAD
    def visualize_markers_and_path(self, image, marker_positions, marker_ids, path_start, path_end, offset_start, offset_end):
=======
    def visualize_markers_and_path(self, image, marker_positions, marker_ids, path_start, path_end):
>>>>>>> 3476c19274e5a8000dc06294fed2541c695fb63e
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
            start_2d = self.project_3d_to_2d(path_start)
            end_2d = self.project_3d_to_2d(path_end)
            cv2.line(image, tuple(start_2d.astype(int)), tuple(end_2d.astype(int)), (0, 255, 0), 2)
            cv2.arrowedLine(image, tuple(start_2d.astype(int)), tuple(end_2d.astype(int)), (0, 255, 0), 2, tipLength=0.05)
            
            # Calculate line length
            line_length = np.linalg.norm(path_end - path_start)
            length_str = f"{line_length:.2f}m"
            
            # Calculate midpoint for text placement
            mid_point = (start_2d + end_2d) / 2
            
            # Draw length text above the line
            cv2.putText(image, length_str, (int(mid_point[0]), int(mid_point[1]) - 10), 
<<<<<<< HEAD
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

=======
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
>>>>>>> 3476c19274e5a8000dc06294fed2541c695fb63e
    def coords_to_string(self, coords):
        return f"X: {coords[0]:.2f}, Y: {coords[1]:.2f}, Z: {coords[2]:.2f}"

    def project_3d_to_2d(self, point_3d):
        point_3d = np.array([point_3d], dtype=np.float32)
        point_2d, _ = cv2.projectPoints(point_3d, np.zeros(3), np.zeros(3), self.front_camera_matrix, self.front_dist_coeffs)
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