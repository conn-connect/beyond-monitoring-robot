import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from beyond_robotics_interfaces.msg import LaneGuidance
import cv2
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs

def get_color_range(target_h, min_s=100, min_v=100):
    """
    주어진 목표 색상(H), 최소 채도(S)와 명도(V)에 대한 HSV 범위를 반환합니다.
    
    :param target_h: 목표 색상 (0-179)
    :param min_s: 최소 채도 (0-255)
    :param min_v: 최소 명도 (0-255)
    :return: 하한값과 상한값 튜플의 리스트
    """
    if target_h < 0 or target_h > 179:
        raise ValueError("Target hue must be between 0 and 179")

    hue_range = 20  # 색상 범위 (조정 가능)
    
    lower = np.array([max(0, target_h - hue_range), min_s, min_v])
    upper = np.array([min(179, target_h + hue_range), 255, 255])
    
    if target_h - hue_range < 0:
        return [(np.array([0, min_s, min_v]), upper),
                (np.array([180 + (target_h - hue_range), min_s, min_v]), np.array([179, 255, 255]))]
    elif target_h + hue_range > 179:
        return [(lower, np.array([179, 255, 255])),
                (np.array([0, min_s, min_v]), np.array([(target_h + hue_range) % 180, 255, 255]))]
    else:
        return [(lower, upper)]

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        self.lane_guidance_publisher = self.create_publisher(LaneGuidance, 'lane_guidance', 10)
        self.image_publisher = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.mask_publisher = self.create_publisher(Image, 'camera/color/mask', 10)

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Parameters for lane detection
        self.min_line_length = 50
        self.max_line_gap = 25
        self.current_lane_position = None
        self.prev_lane_center = None
        self.lane_width = 320  # Estimated lane width in pixels
        
        self.target_position = None
        
        # New control parameters
        self.Kp = 0.005  # Proportional gain
        self.deadband = 5  # Deadband in pixels

        # 목표 색상 설정
        self.target_color = (60, 80, 80)  # (H, min_S, min_V)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        lanes, processed_image, mask = self.detect_lanes(color_image)
        
        if lanes:
            self.publish_lane_guidance(lanes, processed_image)
        
        img_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        self.image_publisher.publish(img_msg)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.mask_publisher.publish(mask_msg)

    def detect_lanes(self, image):
        height, width = image.shape[:2]
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Get color range using the new function
        color_ranges = get_color_range(*self.target_color)
        
        # Create mask for the target color
        mask = np.zeros((height, width), dtype=np.uint8)
        for lower, upper in color_ranges:
            mask |= cv2.inRange(hsv, lower, upper)
        
        # Apply morphological operations
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Define region of interest (ROI)
        roi_vertices = [(0, height), (0, height/2), (width, height/2), (width, height)]
        roi_mask = np.zeros_like(mask)
        cv2.fillPoly(roi_mask, np.array([roi_vertices], dtype=np.int32), 255)
        masked = cv2.bitwise_and(mask, roi_mask)
        
        # Find contours
        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter and fit lines to contours
        lanes = []
        for contour in contours:
            if cv2.contourArea(contour) > 1000:  # Adjust this threshold as needed
                rect = cv2.minAreaRect(contour)
                (x, y), (w, h), angle = rect
                aspect_ratio = min(w, h) / max(w, h)
                
                # Filter out contours that are too square-like (potential boxes)
                if aspect_ratio < 0.2:  # Adjust this threshold as needed
                    vx, vy, x, y = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
                    slope = vy / vx if vx != 0 else float('inf')
                    intercept = y - slope * x
                    if 0.1 < abs(slope) < 10:
                        lanes.append((slope[0], intercept[0]))
        
        return lanes, image, masked

    def publish_lane_guidance(self, lanes, image):
        if not lanes:
            return
        
        height, width = image.shape[:2]
        
        # Separate lanes based on slope
        left_lanes = [lane for lane in lanes if lane[0] > 0]  # Positive slope
        right_lanes = [lane for lane in lanes if lane[0] < 0]  # Negative slope
        
        # Sort left lanes from right to left, and right lanes from left to right
        left_lanes.sort(key=lambda lane: -lane[1]/lane[0] if lane[0] != 0 else float('inf'), reverse=True)
        right_lanes.sort(key=lambda lane: -lane[1]/lane[0] if lane[0] != 0 else float('inf'))
        
        left_lane = left_lanes[0] if left_lanes else None
        right_lane = right_lanes[0] if right_lanes else None
        
        # Calculate target position (center of the lane)
        if left_lane and right_lane:
            left_x = int((height - left_lane[1]) / left_lane[0])
            right_x = int((height - right_lane[1]) / right_lane[0])
            self.target_position = (left_x + right_x) // 2
        elif left_lane:
            left_x = int((height - left_lane[1]) / left_lane[0])
            self.target_position = left_x - self.lane_width // 2
        elif right_lane:
            right_x = int((height - right_lane[1]) / right_lane[0])
            self.target_position = right_x + self.lane_width // 2
        else:
            self.target_position = width // 2

        # Calculate current position
        current_position = self.current_lane_position or width // 2
        
        # Calculate error
        error = self.target_position - current_position
        
        # Apply deadband and proportional control
        if abs(error) > self.deadband:
            angular_z = self.Kp * (error - np.sign(error) * self.deadband)
        else:
            angular_z = 0.0
        
        # Limit the maximum angular velocity
        angular_z = max(min(angular_z, 0.3), -0.3)
        
        msg = LaneGuidance()
        msg.angular_z = -angular_z  # Negative because right is positive
        
        self.lane_guidance_publisher.publish(msg)
        
        # Update current lane position (with smoothing)
        if self.prev_lane_center is not None:
            current_position = int(0.7 * self.prev_lane_center + 0.3 * current_position)
        self.prev_lane_center = current_position
        self.current_lane_position = current_position
        
        # Visualize detected lanes and guidance
        if left_lane:
            self.draw_lane(image, left_lane, (255, 0, 0))  # Blue for left lane
        if right_lane:
            self.draw_lane(image, right_lane, (0, 0, 255))  # Red for right lane
        
        # Draw center indicator
        cv2.circle(image, (width // 2, height - 20), 7, (100, 100, 100), -1)  # Gray center circle
        
        # Draw current position indicator
        cv2.circle(image, (current_position, height - 20), 5, (0, 255, 0), -1)  # Green circle
        
        # Draw target position indicator
        cv2.circle(image, (self.target_position, height - 30), 5, (0, 255, 255), -1)  # Yellow circle
        
        # Draw text showing angular_z
        cv2.putText(image, f"Angular Z: {angular_z:.4f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    def draw_lane(self, image, lane, color):
        height, width = image.shape[:2]
        slope, intercept = lane
        y1 = height
        y2 = int(height / 2)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        cv2.line(image, (x1, y1), (x2, y2), color, 2)
        
    def __del__(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()