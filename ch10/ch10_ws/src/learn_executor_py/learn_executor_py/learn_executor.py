import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import threading
import time


class LearnExecutorNode(Node):
    def __init__(self):
        super().__init__('learn_executor')
        self.publisher = self.create_publisher(String, 'string_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        my_callback_group = ReentrantCallbackGroup()
        self.service = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_two_ints_callback,
            callback_group=my_callback_group)
        # self.service = self.create_service(
        #     AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Topic published, Thread ID: {threading.get_ident()}, Total threads: {threading.active_count()}'
        self.get_logger().info(msg.data)
        self.publisher.publish(msg)

    def add_two_ints_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        self.get_logger().info(f'Processing service, Thread ID: {threading.get_ident()}')
        time.sleep(10)  # Simulate processing delay
        response.sum = request.a + request.b
        self.get_logger().info(f'Processing completed, Thread ID: {threading.get_ident()}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LearnExecutorNode()
    # executor = SingleThreadedExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()