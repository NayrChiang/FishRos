# Ch 5.2.1
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math

class StaticTFBroadcast (Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.publish_static_tf()
        
    def publish_static_tf(self):
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.child_frame_id = 'camera_link'
        
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6
        
        x, y, z, w = quaternion_from_euler(math.radians(180), 0, 0)
        
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        
        self.static_broadcaster_.sendTransform(transform)
        self.get_logger().info(f'Publish Static TF: {transform}')
        
def main():
    rclpy.init()
    node = StaticTFBroadcast()
    rclpy.spin(node)
    rclpy.shutdown()
        