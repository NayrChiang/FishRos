# Ch 7.4.3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import time

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # nav.get_logger().info(f'Navigation feedback: {feedback}')
        nav.get_logger().info(f'Remaining distance: {feedback.distance_remaining:.2f} meters')
        # nav.cancelTask()
        time.sleep(0.5)

    result = nav.getResult()
    nav.get_logger().info(f'Navigation result: {result}')

    rclpy.spin(nav)
    rclpy.shutdown()