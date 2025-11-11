# Ch 7.4.2
import rclpy
import rclpy.time
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import math

def euler_from_quaternion(quaternion):
    """Convert quaternion [x, y, z, w] to Euler angles (roll, pitch, yaw)"""
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]

class GetRobotPose (Node):
    def __init__(self):
        super().__init__('get_robot_pose')
        self.buffer_ = Buffer()
        self.broadcaster_ = TransformListener(self.buffer_, self)
        self.timer_ = self.create_timer(1.0, self.get_transform)
        self.get_transform()
        
    def get_transform(self):
        try:
            tf = self.buffer_.lookup_transform('map', 'base_footprint', 
                                                   rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            transform = tf.transform
            self.get_logger().info(f"Translation: {transform.translation}")
            self.get_logger().info(f"Rotation: {transform.rotation}")
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            self.get_logger().info(f'Rotation RPY (roll, pitch, yaw): {rotation_euler}')
        except Exception as e:
            self.get_logger().warn(f"Failed to obtain TF, cause: {str(e)}")
        
def main():
    rclpy.init()
    node = GetRobotPose()
    rclpy.spin(node)
    rclpy.shutdown()
        