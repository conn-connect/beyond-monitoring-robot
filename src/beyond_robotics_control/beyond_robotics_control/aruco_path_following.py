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
        
        # Latest color and depth images
        self.color_image = None
        self.depth_image = None
        
        # Path parameters
        self.offset_distance = 0.0  # 70cm offset
        
        # Target ArUco marker IDs
        self.target_ids = [0, 1]

        # Add subscriber for depth scale
        self.depth_scale = 0.001

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

        self.get_logger().info(f"Camera intrinsics updated: {self.rs_intrinsics}")


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


    def visualize_markers_and_path(self, image, marker_positions, marker_ids, path_start, path_end):
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
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
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