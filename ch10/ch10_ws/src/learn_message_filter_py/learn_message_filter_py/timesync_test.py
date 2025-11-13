import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer

class TimeSyncTestNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        # 1. Subscribe to imu topic and register callback to print timestamp
        self.imu_sub = Subscriber(self, Imu, 'imu')
        self.imu_sub.registerCallback(self.imu_callback)
        # 2. Subscribe to odom topic and register callback to print timestamp
        self.odom_sub = Subscriber(self, Odometry, 'odom')
        self.odom_sub.registerCallback(self.odom_callback)
        # 3. Create synchronizer with corresponding policy to sync two topics and register callback to print data
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.imu_sub, self.odom_sub], 10,
            slop=0.01,  # slop represents time window in seconds
        )
        self.synchronizer.registerCallback(self.result_callback)

    def imu_callback(self, imu_msg):
        self.get_logger().info(
            f'imu({imu_msg.header.stamp.sec},{imu_msg.header.stamp.nanosec})')

    def odom_callback(self, odom_msg):
        self.get_logger().info(
            f'odom({odom_msg.header.stamp.sec},{odom_msg.header.stamp.nanosec})')

    def result_callback(self, imu_msg, odom_msg):
        self.get_logger().info(
            f'imu({imu_msg.header.stamp.sec},{imu_msg.header.stamp.nanosec}),odom({odom_msg.header.stamp.sec},{odom_msg.header.stamp.nanosec})')


def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncTestNode()
    rclpy.spin(node)
    rclpy.shutdown()