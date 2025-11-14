# Chapter 9: Real Robot Implementation

## Overview

This chapter covers ROS 2 concepts and considerations for implementing a real robot system. While the actual hardware components will be purchased later, this chapter documents the ROS 2 concepts, architecture, and best practices needed for real robot deployment.

## Key Concepts

### Hardware Interfaces

Real robots require hardware interfaces to communicate with physical components:
- **Sensors**: LIDAR, cameras, IMU, encoders
- **Actuators**: Motors, servos, grippers
- **Communication**: Serial, USB, CAN bus, Ethernet
- **ROS 2 Hardware Interface**: Bridge between ROS 2 and hardware

### ROS 2 Control for Real Robots

ROS 2 Control provides hardware abstraction:
- **Hardware Interface**: Connects to physical hardware
- **Controllers**: Control algorithms (same as simulation)
- **Controller Manager**: Manages real-time control loops
- **Benefits**: Same code works in simulation and on real robot

### Sensor Integration

Integrating physical sensors:
- **Driver Nodes**: Interface with sensor hardware
- **Message Publishing**: Convert sensor data to ROS messages
- **Calibration**: Sensor calibration and transformation
- **Timing**: Synchronization and timestamping

### Actuator Control

Controlling physical actuators:
- **Motor Drivers**: Interface with motor controllers
- **Velocity/Position Control**: Command interfaces
- **Feedback**: Encoder readings and state feedback
- **Safety**: Emergency stops and limits

### Real-Time Considerations

Real robot systems have timing requirements:
- **Control Loops**: High-frequency control (100+ Hz)
- **Latency**: Minimize communication delays
- **Determinism**: Predictable execution times
- **Priority**: Real-time scheduling

### Network Configuration

Robot network setup:
- **Robot Computer**: Onboard computer running ROS 2
- **Remote Access**: SSH, remote visualization
- **Topics/Services**: Network communication
- **Bandwidth**: Managing data rates

## Code Examples

### Example 1: Hardware Interface Structure

```cpp
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

class MyRobotHardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override {
        // Initialize hardware connections
        // Parse hardware info
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override {
        // Enable hardware
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    hardware_interface::return_type read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override {
        // Read sensor data from hardware
        // Update state interfaces
        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override {
        // Write commands to actuators
        // Use command interfaces
        return hardware_interface::return_type::OK;
    }
};
```

**Key Points:**
- Inherit from `hardware_interface::SystemInterface`
- Implement hardware communication
- Read sensors, write actuators
- Handle lifecycle states

### Example 2: Sensor Driver Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

class LidarDriverNode(Node):
    def __init__(self):
        super().__init__('lidar_driver')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        
        # Connect to hardware
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        
        # Create timer for reading sensor
        self.timer_ = self.create_timer(0.1, self.read_sensor)
    
    def read_sensor(self):
        # Read data from hardware
        raw_data = self.serial_port.readline()
        
        # Convert to ROS message
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        # ... populate message from raw_data ...
        
        self.publisher_.publish(msg)
```

**Key Points:**
- Interface with hardware (serial, USB, etc.)
- Convert hardware data to ROS messages
- Publish on standard topics
- Handle hardware errors gracefully

### Example 3: Actuator Control Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO  # Example for Raspberry Pi

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.subscription_ = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Initialize GPIO for motor control
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)  # Motor pin
        self.pwm = GPIO.PWM(18, 1000)  # PWM frequency
        self.pwm.start(0)
    
    def cmd_vel_callback(self, msg):
        # Convert Twist to motor commands
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Calculate left/right wheel speeds
        left_speed = linear_vel - angular_vel
        right_speed = linear_vel + angular_vel
        
        # Send commands to motors
        self.set_motor_speed('left', left_speed)
        self.set_motor_speed('right', right_speed)
    
    def set_motor_speed(self, motor, speed):
        # Convert speed to PWM duty cycle
        duty_cycle = abs(speed) * 100
        self.pwm.ChangeDutyCycle(duty_cycle)
```

**Key Points:**
- Subscribe to control commands
- Convert ROS messages to hardware commands
- Interface with motor controllers
- Handle safety limits

### Example 4: ROS 2 Control Hardware Interface

```cpp
// In URDF/Xacro
<ros2_control name="MyRobotHardware" type="system">
  <hardware>
    <plugin>my_robot_hardware/MyRobotHardware</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
  </hardware>
  
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

**Key Points:**
- Define hardware in URDF
- Specify command and state interfaces
- Hardware plugin handles communication
- Same controllers work as simulation

## Key Takeaways

1. **Hardware Abstraction**: Use ROS 2 Control to abstract hardware. Same controllers work in sim and real robot.

2. **Driver Nodes**: Create driver nodes for sensors/actuators that don't fit ROS 2 Control model.

3. **Safety First**: 
   - Implement emergency stops
   - Set velocity/acceleration limits
   - Monitor system health
   - Handle hardware failures gracefully

4. **Calibration**: Sensors need calibration:
   - Camera intrinsics/extrinsics
   - IMU bias and noise
   - LIDAR mounting position
   - Wheel odometry parameters

5. **Timing**: Real robots need:
   - High-frequency control loops
   - Low-latency communication
   - Synchronized sensor data
   - Predictable execution

6. **Network Setup**:
   - Onboard computer for robot
   - Remote access for development
   - Configure ROS 2 domain IDs
   - Manage bandwidth

7. **Testing Strategy**:
   - Test in simulation first
   - Use hardware-in-the-loop when possible
   - Start with low speeds
   - Have emergency stop ready

8. **Documentation**: Document:
   - Hardware connections
   - Pin assignments
   - Calibration procedures
   - Troubleshooting steps

## Hardware Components (To Be Purchased)

### Sensors
- **LIDAR**: For mapping and navigation
- **Camera**: For vision tasks
- **IMU**: For orientation and motion
- **Encoders**: For wheel odometry

### Actuators
- **Motors**: For robot movement
- **Motor Controllers**: For motor control
- **Servos**: For manipulators (if needed)

### Computing
- **Onboard Computer**: Raspberry Pi, Jetson, or similar
- **Power Management**: Battery, power distribution
- **Communication**: WiFi, Ethernet

### Mechanical
- **Chassis**: Robot base
- **Wheels**: Differential drive or other
- **Mounts**: For sensors and components

## Related Topics

- **Chapter 6**: URDF/Xacro (robot description)
- **Chapter 7**: Navigation2 (autonomous navigation)
- **Chapter 8**: ROS 2 Control (hardware interfaces)
- **Chapter 10**: Advanced features (QoS, executors)

## Resources

- [ROS 2 Control Documentation](https://control.ros.org/)
- [Hardware Interface Tutorial](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_component.html)
- [Real Robot Setup Guide](https://docs.ros.org/en/humble/Guides/Real-Robot-Setup.html)
- [ROS 2 Hardware Acceleration](https://github.com/ros-acceleration)

