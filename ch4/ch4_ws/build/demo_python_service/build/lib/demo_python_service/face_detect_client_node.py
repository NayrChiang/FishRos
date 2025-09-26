import rclpy
from rclpy.node import Node
from ch4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge
import time

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/test1.jpg')
        self.get_logger().info("Face Detector Client Node Initialized!")
        self.client = self.create_client(FaceDetector, 'face_detector')
        self.image = cv2.imread(self.default_image_path)
        
    def send_request(self):
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info("Wait for service client to start!")
            
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        future = self.client.call_async(request)
        
        def result_callback(result_future):
            response = result_future.result()
            self.get_logger().info("Received response.")
            self.show_response(response)
            
        future.add_done_callback(result_callback)
    
    def show_response(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 4)
            
        cv2.imshow('Face Detect Result', self.image)
        cv2.waitKey(0)
      
      
def main():
    rclpy.init()
    node = FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()