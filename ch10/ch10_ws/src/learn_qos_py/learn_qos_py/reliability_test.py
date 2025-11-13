import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.qos import QoSProfile

class OdomPublisherSubscriber(Node):
    def __init__(self):
        super().__init__('odom_publisher_subscriber')

        qos_profile = qos.QoSProfile(depth=10,
                                     reliability=qos.ReliabilityPolicy.BEST_EFFORT,
                                     durability=qos.DurabilityPolicy.VOLATILE)
        
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', qos.qos_profile_sensor_data)
        self.odom_subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile)
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def odom_callback(self, msg):
        self.get_logger().info(f'Received odom: {msg}')
        # self.odom_publisher_.publish(msg)

    def timer_callback(self):
        odom_msg = Odometry()
        self.get_logger().info('Publishing odom')
        self.odom_publisher_.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisherSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
