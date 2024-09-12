import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from beyond_robotics_interfaces.msg import LaneGuidance
import cv2
from cv_bridge import CvBridge
import numpy as np

def get_color_range(target_h, min_s=100, min_v=100):
    if target_h < 0 or target_h > 179:
        raise ValueError("Target hue must be between 0 and 179")

    hue_range = 20
    
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
        self.processed_image_publisher = self.create_publisher(Image, 'processed_image', 10)
        self.mask_publisher = self.create_publisher(Image, 'lane_mask', 10)

        # Subscribe to the front camera topic
        self.subscription = self.create_subscription(
            Image,
            'camera/front/image_raw',
            self.image_callback,
            10)
        
        self.min_line_length = 50
        self.max_line_gap = 25
        self.current_lane_position = None
        self.prev_lane_center = None
        self.lane_width = 1100
        
        self.target_position = None
        
        self.Kp = 0.01
        self.deadband = 3

        self.target_color = (102, 200, 150)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        lanes, processed_image, mask = self.detect_lanes(cv_image)
        
        if lanes:
            self.publish_lane_guidance(lanes, processed_image)
        
        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        self.processed_image_publisher.publish(processed_msg)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.mask_publisher.publish(mask_msg)

    def detect_lanes(self, image):
        height, width = image.shape[:2]
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        color_ranges = get_color_range(*self.target_color)
        
        mask = np.zeros((height, width), dtype=np.uint8)
        for lower, upper in color_ranges:
            mask |= cv2.inRange(hsv, lower, upper)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        roi_vertices = [(0, height), (0, height/2), (width, height/2), (width, height)]
        roi_mask = np.zeros_like(mask)
        cv2.fillPoly(roi_mask, np.array([roi_vertices], dtype=np.int32), 255)
        masked = cv2.bitwise_and(mask, roi_mask)
        
        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        lanes = []
        for contour in contours:
            if cv2.contourArea(contour) > 1000:
                rect = cv2.minAreaRect(contour)
                (x, y), (w, h), angle = rect
                aspect_ratio = min(w, h) / max(w, h)
                
                if aspect_ratio < 0.3:
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
        
        left_lanes = [lane for lane in lanes if lane[0] > 0]
        right_lanes = [lane for lane in lanes if lane[0] < 0]
        
        left_lanes.sort(key=lambda lane: -lane[1]/lane[0] if lane[0] != 0 else float('inf'), reverse=True)
        right_lanes.sort(key=lambda lane: -lane[1]/lane[0] if lane[0] != 0 else float('inf'))
        
        left_lane = left_lanes[0] if left_lanes else None
        right_lane = right_lanes[0] if right_lanes else None
        
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

        current_position = self.current_lane_position or width // 2
        
        error = self.target_position - current_position
        
        if abs(error) > self.deadband:
            angular_z = self.Kp * (error - np.sign(error) * self.deadband)
        else:
            angular_z = 0.0
        
        angular_z = max(min(angular_z, 0.3), -0.3)
        
        msg = LaneGuidance()
        msg.angular_z = -angular_z
        
        self.lane_guidance_publisher.publish(msg)
        
        if self.prev_lane_center is not None:
            current_position = int(0.7 * self.prev_lane_center + 0.3 * current_position)
        self.prev_lane_center = current_position
        self.current_lane_position = current_position
        
        if left_lane:
            self.draw_lane(image, left_lane, (255, 0, 0))
        if right_lane:
            self.draw_lane(image, right_lane, (0, 0, 255))
        
        cv2.circle(image, (width // 2, height - 20), 7, (100, 100, 100), -1)
        
        cv2.circle(image, (current_position, height - 20), 5, (0, 255, 0), -1)
        
        cv2.circle(image, (self.target_position, height - 30), 5, (0, 255, 255), -1)
        
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

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()