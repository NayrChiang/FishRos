import rclpy
from rclpy.node import Node
from ch4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge
import time

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.service_ = self.create_service(FaceDetector, 'face_detector', self.detect_face_callback)
        self.bridge = CvBridge()
        self.number_of_times_to_upsample = 1
        self.model = 'hog'
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/default.jpg')
        self.get_logger().info("Face Detector Node Initialized!")
        
    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info(f"Failed to load image, use default image")
            
        start_time = time.time()
        self.get_logger().info(f"Image loading completed, starts to detect.")
        face_locations = face_recognition.face_locations(
            cv_image, number_of_times_to_upsample=self.number_of_times_to_upsample, model=self.model)
        
        response.use_time = time.time() - start_time
        response.number = len(face_locations)
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
            
        self.get_logger().info("Face Detection Completed!")
        
        return response
    
def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()