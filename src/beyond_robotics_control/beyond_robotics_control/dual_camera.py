import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class DualCameraNode(Node):
    def __init__(self):
        super().__init__('dual_camera_node')
        self.bridge = CvBridge()

        # Publishers for front camera
        self.front_color_publisher = self.create_publisher(Image, 'camera/front/color/image_raw', 10)
        self.front_aligned_depth_publisher = self.create_publisher(Image, 'camera/front/depth/image_raw', 10)
        self.front_info_publisher = self.create_publisher(CameraInfo, 'camera/front/camera_info', 10)

        # Publishers for side camera
        self.side_color_publisher = self.create_publisher(Image, 'camera/side/color/image_raw', 10)
        self.side_aligned_depth_publisher = self.create_publisher(Image, 'camera/side/depth/image_raw', 10)
        self.side_info_publisher = self.create_publisher(CameraInfo, 'camera/side/camera_info', 10)

        # Initialize RealSense pipelines
        self.front_pipeline = rs.pipeline()
        self.side_pipeline = rs.pipeline()

        # Configure front camera (D435)
        front_config = rs.config()
        front_config.enable_device('233522071373')  # Replace with actual serial number
        front_config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
        front_config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)

        # Configure side camera (D405)
        side_config = rs.config()
        side_config.enable_device('218622277577')  # Replace with actual serial number
        side_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        side_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

        # Start the pipelines
        self.front_profile = self.front_pipeline.start(front_config)
        self.side_profile = self.side_pipeline.start(side_config)

        # Create align objects
        self.front_align = rs.align(rs.stream.color)
        self.side_align = rs.align(rs.stream.color)

        # Get depth scale for each camera
        self.front_depth_scale = self.front_profile.get_device().first_depth_sensor().get_depth_scale()
        self.side_depth_scale = self.side_profile.get_device().first_depth_sensor().get_depth_scale()

        # Set up timer for capturing and publishing images
        self.timer = self.create_timer(1.0/30, self.timer_callback)  # 30 Hz

    def timer_callback(self):
        # Process and publish front camera images
        front_frames = self.front_align.process(self.front_pipeline.wait_for_frames())
        front_color_frame = front_frames.get_color_frame()
        front_depth_frame = front_frames.get_depth_frame()
        if front_color_frame and front_depth_frame:
            self.process_and_publish_frames(front_color_frame, front_depth_frame, 
                                            self.front_color_publisher, self.front_aligned_depth_publisher, 
                                            self.front_info_publisher, "front_camera_link")

        # Process and publish side camera images
        side_frames = self.side_align.process(self.side_pipeline.wait_for_frames())
        side_color_frame = side_frames.get_color_frame()
        side_depth_frame = side_frames.get_depth_frame()
        if side_color_frame and side_depth_frame:
            self.process_and_publish_frames(side_color_frame, side_depth_frame, 
                                            self.side_color_publisher, self.side_aligned_depth_publisher, 
                                            self.side_info_publisher, "side_camera_link")

    def process_and_publish_frames(self, color_frame, depth_frame, color_publisher, depth_publisher, info_publisher, frame_id):
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
        
        timestamp = self.get_clock().now().to_msg()
        color_msg.header.stamp = timestamp
        depth_msg.header.stamp = timestamp
        color_msg.header.frame_id = frame_id
        depth_msg.header.frame_id = frame_id
        
        color_publisher.publish(color_msg)
        depth_publisher.publish(depth_msg)
        
        # Publish camera info
        info = self.get_camera_info(color_frame, timestamp, frame_id)
        info_publisher.publish(info)

    def get_camera_info(self, frame, timestamp, frame_id):
        info = CameraInfo()
        intr = frame.profile.as_video_stream_profile().intrinsics
        
        info.header.stamp = timestamp
        info.header.frame_id = frame_id
        info.width = intr.width
        info.height = intr.height
        info.k = [float(intr.fx), 0.0, float(intr.ppx),
                  0.0, float(intr.fy), float(intr.ppy),
                  0.0, 0.0, 1.0]
        info.d = [float(c) for c in intr.coeffs]
        info.distortion_model = 'plumb_bob'
        
        # Set up the projection matrix
        info.p = [float(intr.fx), 0.0, float(intr.ppx), 0.0,
                  0.0, float(intr.fy), float(intr.ppy), 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        # Set up the rectification matrix
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        
        return info

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