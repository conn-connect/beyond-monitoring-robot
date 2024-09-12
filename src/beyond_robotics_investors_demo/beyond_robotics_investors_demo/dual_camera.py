import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class DualCameraNode(Node):
    def __init__(self):
        super().__init__('dual_camera_node')
        self.bridge = CvBridge()
        
        # Publishers for front and side cameras
        self.front_publisher = self.create_publisher(Image, 'camera/front/image_raw', 10)
        self.side_publisher = self.create_publisher(Image, 'camera/side/image_raw', 10)
        
        # Initialize RealSense pipelines
        self.front_pipeline = rs.pipeline()
        self.side_pipeline = rs.pipeline()
        
        # Configure front camera (D435)
        front_config = rs.config()
        front_config.enable_device('233522071373')  # Replace with actual serial number
        front_config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
        
        # Configure side camera (D405)
        side_config = rs.config()
        side_config.enable_device('218622277577')  # Replace with actual serial number
        side_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        
        # Start the pipelines
        self.front_pipeline.start(front_config)
        self.side_pipeline.start(side_config)
        
        # Set up timer for capturing and publishing images
        self.timer = self.create_timer(1.0/30, self.timer_callback)  # 30 Hz

    def timer_callback(self):
        # Capture and publish front camera image
        front_frames = self.front_pipeline.wait_for_frames()
        front_color_frame = front_frames.get_color_frame()
        if front_color_frame:
            front_image = np.asanyarray(front_color_frame.get_data())
            front_msg = self.bridge.cv2_to_imgmsg(front_image, encoding="bgr8")
            self.front_publisher.publish(front_msg)
        
        # Capture and publish side camera image
        side_frames = self.side_pipeline.wait_for_frames()
        side_color_frame = side_frames.get_color_frame()
        if side_color_frame:
            side_image = np.asanyarray(side_color_frame.get_data())
            side_msg = self.bridge.cv2_to_imgmsg(side_image, encoding="bgr8")
            self.side_publisher.publish(side_msg)

    def __del__(self):
        # Stop the pipelines when the node is destroyed
        self.front_pipeline.stop()
        self.side_pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()