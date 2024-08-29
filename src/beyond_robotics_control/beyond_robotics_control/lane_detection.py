import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from beyond_robotics_interfaces.msg import LaneGuidance
import cv2
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        self.lane_guidance_publisher = self.create_publisher(LaneGuidance, 'lane_guidance', 10)
        self.image_publisher = self.create_publisher(Image, 'camera/color/image_raw', 10)

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        lane_center = self.detect_lane(color_image)
        
        if lane_center is not None:
            self.publish_lane_guidance(lane_center, color_image.shape[1])
        
        img_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        self.image_publisher.publish(img_msg)

    def detect_lane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.erode(mask, kernel, iterations = 1)
        mask = cv2.dilate(mask, kernel, iterations = 1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) < 2:
            return None
        
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
        
        centers = []
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                centers.append(cx)
        
        if len(centers) == 2:
            lane_center = sum(centers) // 2
            cv2.circle(image, (lane_center, image.shape[0] // 2), 10, (0, 0, 255), -1)
            return lane_center
        
        return None

    def publish_lane_guidance(self, lane_center, image_width):
        error = lane_center - (image_width // 2)
        
        Kp = 0.01
        angular_z = Kp * error
        
        msg = LaneGuidance()
        msg.angular_z = -angular_z  # Negative because right is positive
        
        self.lane_guidance_publisher.publish(msg)

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
