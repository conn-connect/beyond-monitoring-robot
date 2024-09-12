import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from beyond_robotics_interfaces.msg import QRInfo

class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        print(f"OpenCV version: {cv2.__version__}")
        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(QRInfo, 'marker_info', 10)
        self.image_publisher = self.create_publisher(Image, 'marker_detected_image', 10)
        
        # ArUco dictionary
        board_type = cv2.aruco.DICT_4X4_250
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(board_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # Detect ArUco markers
        corners, ids, _ = self.detector.detectMarkers(blurred)
        
        # If ArUco markers are detected
        if ids is not None:
            for i in range(len(ids)):
                corner = corners[i][0]
                (topLeft, topRight, bottomRight, bottomLeft) = corner
                
                # Calculate center point
                x = int(corner[:, 0].mean())
                y = int(corner[:, 1].mean())
                
                # Draw marker corners and ID
                blue_BGR = (255, 0, 0)
                cv2.circle(cv_image, tuple(topLeft.astype(int)), 4, blue_BGR, -1)
                cv2.circle(cv_image, tuple(topRight.astype(int)), 4, blue_BGR, -1)
                cv2.circle(cv_image, tuple(bottomRight.astype(int)), 4, blue_BGR, -1)
                cv2.circle(cv_image, tuple(bottomLeft.astype(int)), 4, blue_BGR, -1)
                cv2.putText(cv_image, f"ID: {ids[i][0]}", (x+10, y+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Publish marker info
                marker_info = QRInfo()
                marker_info.data = str(ids[i][0])
                marker_info.width = int(np.linalg.norm(topRight - topLeft))
                marker_info.x = x
                marker_info.y = y
                self.marker_publisher.publish(marker_info)
        
        # Publish processed image
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.image_publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()