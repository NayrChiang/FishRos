# Chapter 6: Robot Description (URDF/Xacro) and TF

## Overview

This chapter covers **URDF** (Unified Robot Description Format) and **Xacro** (XML Macros) for describing robot models, and **TF** (Transform) for managing coordinate frames. You'll learn how to create robot models, define links and joints, and use TF to track relationships between coordinate frames.

## Key Concepts

### URDF vs Xacro

**URDF (Unified Robot Description Format):**
- XML-based format for robot description
- Static, verbose, repetitive
- Good for simple robots
- Direct XML representation

**Xacro (XML Macros):**
- Extension of URDF with macro capabilities
- Supports parameters, includes, and math expressions
- Reduces repetition and improves maintainability
- Preprocessed into URDF before use

### Robot Model Components

**Links:**
- Represent physical parts of the robot
- Have visual, collision, and inertial properties
- Define geometry and appearance

**Joints:**
- Connect links together
- Define relationships between links
- Types: fixed, revolute, continuous, prismatic, floating, planar

**Properties:**
- **Visual**: Appearance in visualization (RViz)
- **Collision**: Geometry for collision detection
- **Inertial**: Mass and inertia for physics simulation

### TF (Transform)

TF manages coordinate frame relationships:
- **Static Transforms**: Fixed relationships (e.g., camera to base)
- **Dynamic Transforms**: Changing relationships (e.g., joint positions)
- **Frame Tree**: Hierarchical structure of frames
- **Robot State Publisher**: Publishes transforms from URDF/joint states

## Code Examples

### Example 1: Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <!-- Link definition -->
    <link name="base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <cylinder radius="0.1" length="0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <cylinder radius="0.1" length="0.2"/>
            </geometry>
        </collision>
    </link>
    
    <!-- Joint definition -->
    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="0.5 0 0.3" rpy="0 0 0"/>
    </joint>
</robot>
```

**Key Points:**
- `<link>` defines a physical component
- `<joint>` connects links
- `visual` for appearance, `collision` for physics
- `origin` specifies position and orientation

### Example 2: Xacro Macros

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="first_robot">
    
    <!-- Macro definition with parameters -->
    <xacro:macro name="base_link" params="length radius">
        <link name="base_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <cylinder radius="${radius}" length="${length}"/>
                </geometry>
                <material name="white">
                    <color rgba="1.0 1.0 1.0 0.5"/>
                </material>
            </visual>
        </link>
    </xacro:macro>
    
    <!-- Macro with multiple parameters -->
    <xacro:macro name="imu_link" params="imu_name xyz">
        <link name="${imu_name}_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.02 0.02 0.02"/>
                </geometry>
            </visual>
        </link>
        <joint name="${imu_name}_joint" type="fixed">
            <parent link="base_link"/>
            <child link="${imu_name}_link"/>
            <origin xyz="${xyz}" rpy="0.0 0.0 0.0"/>
        </joint>
    </xacro:macro>
    
    <!-- Use macros -->
    <xacro:base_link length="0.12" radius="0.1"/>
    <xacro:imu_link imu_name="imu_up" xyz="0.0 0.0 0.03"/>
    <xacro:imu_link imu_name="imu_down" xyz="0.0 0.0 -0.03"/>
    
</robot>
```

**Key Points:**
- `xacro:macro` defines reusable components
- `${variable}` for parameter substitution
- `xacro:include` to include other files
- Reduces code duplication

### Example 3: Xacro Includes and Modular Design

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- Include common definitions -->
    <xacro:include filename="$(find package_name)/urdf/common_inertia.xacro"/>
    
    <!-- Include base -->
    <xacro:include filename="$(find package_name)/urdf/base.urdf.xacro"/>
    <xacro:base_xacro length="0.2" radius="0.15"/>
    
    <!-- Include sensors -->
    <xacro:include filename="$(find package_name)/urdf/sensor/camera.urdf.xacro"/>
    <xacro:camera name="front_camera" xyz="0.3 0 0.1"/>
    
    <!-- Include actuators -->
    <xacro:include filename="$(find package_name)/urdf/actuator/wheel.urdf.xacro"/>
    <xacro:wheel name="left_wheel" xyz="0 0.2 0"/>
</robot>
```

**Key Points:**
- `xacro:include` for modular design
- `$(find package_name)` resolves package paths
- Separate files for different components
- Main file composes everything

### Example 4: Inertial Properties

```xml
<xacro:macro name="cylinder_inertia" params="m r h">
    <inertial>
        <mass value="${m}"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia 
            ixx="${(1.0/12.0)*m*(3*r*r+h*h)}"
            iyy="${(1.0/12.0)*m*(3*r*r+h*h)}"
            izz="${0.5*m*r*r}"
            ixy="0" ixz="0" iyz="0"/>
    </inertial>
</xacro:macro>

<!-- Use in link -->
<link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <xacro:cylinder_inertia m="1.0" r="0.1" h="0.2"/>
</link>
```

**Key Points:**
- Inertial properties needed for physics simulation
- Mass and inertia matrix required
- Formulas depend on geometry (box, cylinder, sphere)

### Example 5: Static TF Broadcaster (Python)

```python
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math

class StaticTFBroadcast(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.publish_static_tf()
        
    def publish_static_tf(self):
        transform = TransformStamped()
        
        # Set frame IDs
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'
        transform.header.stamp = self.get_clock().now().to_msg()
        
        # Set translation
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6
        
        # Set rotation (convert Euler to quaternion)
        x, y, z, w = quaternion_from_euler(
            math.radians(180), 0, 0)
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        
        # Publish transform
        self.static_broadcaster_.sendTransform(transform)
```

**Key Points:**
- `StaticTransformBroadcaster` for fixed transforms
- `TransformStamped` message structure
- `header.frame_id` is parent frame
- `child_frame_id` is child frame
- Use quaternions for rotation

### Example 6: TF Listener (Python)

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Look up transform
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',      # target frame
                'camera_link',    # source frame
                rclpy.time.Time() # time (0 = latest)
            )
            self.get_logger().info(f'Transform: {transform}')
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
```

**Key Points:**
- `TransformListener` subscribes to TF topics
- `Buffer` stores transform history
- `lookup_transform(target, source, time)` gets transform
- Handles frame relationships

## Key Takeaways

1. **URDF Structure**: Links define parts, joints connect them. Always include visual, collision, and inertial properties.

2. **Xacro Benefits**: 
   - Macros reduce repetition
   - Parameters make models configurable
   - Includes enable modular design
   - Math expressions simplify calculations

3. **Frame Naming**: Use descriptive names (e.g., `base_link`, `camera_link`). Follow ROS conventions.

4. **TF Tree**: Must form a tree structure (no cycles). One root frame (typically `odom` or `map`).

5. **Robot State Publisher**: Automatically publishes transforms from URDF and joint states. Essential for visualization.

6. **Static vs Dynamic TF**:
   - Static: Fixed relationships (cameras, sensors)
   - Dynamic: Changing relationships (joints, moving parts)

7. **Visualization**: Use RViz to visualize robot model and TF tree. Check frame relationships.

8. **Common Frames**:
   - `base_link` or `base_footprint`: Robot base
   - `odom`: Odometry frame
   - `map`: Map frame (for navigation)

## Related Topics

- **Chapter 5**: Launch files (loading URDF in launch files)
- **Chapter 7**: Navigation2 (uses TF for localization)
- **Chapter 9**: Real robot (URDF needed for hardware)

## Resources

- [URDF Documentation](http://wiki.ros.org/urdf)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [TF2 Documentation](https://docs.ros.org/en/humble/Concepts/About-Tf2.html)
- [Robot State Publisher](https://github.com/ros/robot_state_publisher)
- [RViz for Visualization](https://github.com/ros2/rviz)

