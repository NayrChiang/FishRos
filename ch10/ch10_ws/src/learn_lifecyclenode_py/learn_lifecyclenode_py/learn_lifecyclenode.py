import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class LearnLifeCycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecyclenode')
        self.timer_period = 0
        self.timer_ = None
        self.get_logger().info(f'{self.get_name()}: Created')

    def timer_callback(self):
        self.get_logger().info('Timer printing in progress...')

    def on_configure(self, state):
        self.timer_period = 1.0  # Set timer period
        self.get_logger().info('on_configure(): Configure timer_period')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('on_activate(): Handle activate command, create timer')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.destroy_timer(self.timer_) # Destroy timer
        self.get_logger().info('on_deactivate(): Handle deactivate command, stop timer')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.timer_ = None
        self.timer_period = 0
        self.get_logger().info('on_cleanup(): Handle cleanup command')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        # Destroy timer if not already destroyed
        if self.timer_: self.destroy_timer(self.timer_)
        self.get_logger().info('on_shutdown(): Handle shutdown command')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        # Directly call parent class handler
        return super().on_error(state)

def main():
    rclpy.init()
    node = LearnLifeCycleNode()
    rclpy.spin(node)
    rclpy.shutdown()