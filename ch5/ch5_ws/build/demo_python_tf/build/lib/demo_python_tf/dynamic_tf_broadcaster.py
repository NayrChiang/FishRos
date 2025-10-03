# Ch 5.2.2
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math

class TFBroadcast (Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster_ = TransformBroadcaster(self)
        self.timer_ = self.create_timer(0.01, self.publish_tf)
        self.publish_tf()
        
    def publish_tf(self):
        transform = TransformStamped()
        transform.header.frame_id = 'camera_link'
        transform.child_frame_id = 'bottle_link'
        transform.header.stamp = self.get_clock().now().to_msg()
        
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.5
        
        x, y, z, w = quaternion_from_euler(0, 0, 0)
        
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        
        self.broadcaster_.sendTransform(transform)
        self.get_logger().info(f'Publish TF: {transform}')
        
def main():
    rclpy.init()
    node = TFBroadcast()
    rclpy.spin(node)
    rclpy.shutdown()
        