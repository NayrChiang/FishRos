# Ch 7.4.3
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Wait until Nav2 is active (map_server + amcl etc.)
    nav.waitUntilNav2Active()

    # Build waypoints
    def mk_pose(x, y, yaw_w=1.0):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = nav.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.w = yaw_w  # yaw = 0
        return p

    goal_poses = [
        mk_pose( 3.0,  1.0),
        mk_pose( 5.0,  1.0),
        mk_pose( 4.0,  1.0),
        mk_pose( 2.0,  1.0),
    ]

    # Send waypoints
    nav.followWaypoints(goal_poses)

    last_idx = None
    last_wait_log = 0.0  # throttle "waiting for feedback" logs
    # try:
    #     while not nav.isTaskComplete():
    #         feedback = nav.getFeedback()  # may be None
    #         if feedback is not None:
    #             idx = feedback.current_waypoint  # 0-based
    #             if idx != last_idx:
    #                 nav.get_logger().info(f'Current waypoint: {idx}')
    #                 last_idx = idx
    #         time.sleep(0.5)  # 2 Hz logging
    # except KeyboardInterrupt:
    #     nav.cancelTask()
    #     nav.get_logger().info('Canceled by user.')

    # # Result
    # result = nav.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     nav.get_logger().info('All waypoints reached!')
    # elif result == TaskResult.CANCELED:
    #     nav.get_logger().info('Waypoint following canceled.')
    # elif result == TaskResult.FAILED:
    #     # Simple Commander doesn’t expose missed_waypoints directly via TaskResult;
    #     # check logs from the FollowWaypoints server if needed.
    #     nav.get_logger().info('Waypoint following FAILED.')
    try:
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()  # may be None
            if feedback is not None:
                idx = getattr(feedback, 'current_waypoint', None)  # 0-based
                dist = getattr(feedback, 'distance_remaining', None)
                if idx is not None and idx != last_idx:
                    if dist is not None:
                        nav.get_logger().info(
                            f'Current waypoint: {idx}, distance remaining: {dist:.2f} m')
                    else:
                        nav.get_logger().info(f'Current waypoint: {idx}')
                    last_idx = idx
            else:
                now = time.monotonic()
                if now - last_wait_log >= 2.0:  # every 2s
                    nav.get_logger().info('Waiting for feedback...')
                    last_wait_log = now
            time.sleep(0.5)  # 2 Hz logging
        time.sleep(0.5) 
    except KeyboardInterrupt:
        nav.cancelTask()
        nav.get_logger().info('Canceled by user.')
    result = nav.getResult() 
    nav.get_logger().info(f'Navigation result: {result}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
