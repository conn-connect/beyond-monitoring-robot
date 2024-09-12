import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar
from std_msgs.msg import String
from beyond_robotics_interfaces.msg import QRInfo

class QRDetectionNode(Node):
    def __init__(self):
        super().__init__('qr_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.qr_publisher = self.create_publisher(QRInfo, 'qr_code', 10)
        self.image_publisher = self.create_publisher(Image, 'qr_detected_image', 10)
        self.threshold_value = 130
        self.roi_percentage = 0.6

    def binary_thresholding(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.threshold_value, 255, cv2.THRESH_BINARY)
        return binary

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        height, width = cv_image.shape[:2]
        roi_height = int(height * self.roi_percentage)
        roi = cv_image[:roi_height, :]
        
        binary_image = self.binary_thresholding(roi)
        
        barcodes = pyzbar.decode(binary_image)
        
        full_binary = np.zeros((height, width), dtype=np.uint8)
        full_binary[:roi_height, :] = binary_image
        color_binary = cv2.cvtColor(full_binary, cv2.COLOR_GRAY2BGR)
        
        cv2.line(color_binary, (0, roi_height), (width, roi_height), (0, 0, 255), 2)
        
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            barcode_data = barcode.data.decode('utf-8')
            
            cv2.rectangle(color_binary, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            text = f"{barcode_data[-5:]} (X: {x}, W: {w})"
            cv2.putText(color_binary, text, (x, y + h + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            qr_info = QRInfo()
            qr_info.data = barcode_data
            qr_info.width = w
            qr_info.x = int(x + w / 2)
            qr_info.y = int(y + h / 2)
            self.qr_publisher.publish(qr_info)
            
            position = "right" if x + w > width - 30 else "center"
            self.get_logger().info(f'QR Code detected: {barcode_data}, Width: {w}, X: {x}')
        
        img_msg = self.bridge.cv2_to_imgmsg(color_binary, encoding="bgr8")
        self.image_publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()