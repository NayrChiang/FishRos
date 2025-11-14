# Chapter 8: Advanced Topics

## Overview

This chapter covers advanced ROS 2 topics including custom Navigation2 plugins, ROS 2 Control framework, plugin systems, and extending Navigation2 with custom behaviors. You'll learn how to create reusable components and integrate them into the ROS 2 ecosystem.

## Key Concepts

### Custom Navigation2 Plugins

Navigation2 uses a plugin-based architecture that allows you to:
- Create custom controllers for path following
- Implement custom planners for path planning
- Add custom behaviors for recovery actions
- Extend Navigation2 functionality without modifying core code

**Plugin Benefits:**
- Modular and reusable components
- Easy to swap implementations
- Follows ROS 2 plugin interface standards
- Loaded dynamically at runtime

### ROS 2 Control (ros2_control)

ROS 2 Control provides a framework for robot control:
- **Hardware Interface**: Abstraction layer for robot hardware
- **Controllers**: Control algorithms (e.g., diff_drive_controller)
- **Controller Manager**: Manages multiple controllers
- **Joint Interfaces**: Command and state interfaces for joints

**Key Components:**
- Hardware abstraction
- Controller plugins
- Real-time control loops
- Unified control interface

### Plugin System (pluginlib)

pluginlib enables dynamic loading of classes:
- Register classes as plugins
- Load plugins at runtime
- No need to modify main code
- Used throughout ROS 2 (Navigation2, MoveIt, etc.)

### Motion Control System

Custom motion control plugins:
- Define motion controller interface
- Implement specific controllers (e.g., spin, move)
- Load via plugin system
- Reusable across different robots

## Code Examples

### Example 1: Custom Navigation2 Controller

```cpp
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_custom_controller {

class CustomController : public nav2_core::Controller {
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
        
        node_ = parent.lock();
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        plugin_name_ = name;
        
        // Declare parameters
        nav2_util::declare_parameter_if_not_declared(
            node_, plugin_name_ + ".max_linear_speed",
            rclcpp::ParameterValue(0.1));
        node_->get_parameter(
            plugin_name_ + ".max_linear_speed", max_linear_speed_);
    }
    
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker * goal_checker) override {
        
        // Calculate velocities based on path
        geometry_msgs::msg::TwistStamped cmd_vel;
        // ... path following logic ...
        return cmd_vel;
    }
    
    void setPlan(const nav_msgs::msg::Path &path) override {
        global_plan_ = path;
    }
    
private:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav_msgs::msg::Path global_plan_;
    double max_linear_speed_;
    double max_angular_speed_;
};

} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    nav2_custom_controller::CustomController,
    nav2_core::Controller)
```

**Key Points:**
- Inherit from `nav2_core::Controller`
- Implement required methods: `configure()`, `computeVelocityCommands()`, `setPlan()`
- Use `PLUGINLIB_EXPORT_CLASS` to register plugin
- Lifecycle-aware (activate/deactivate/cleanup)

### Example 2: Plugin XML Registration

**custom_controller_plugins.xml:**
```xml
<library path="nav2_custom_controller">
  <class name="nav2_custom_controller::CustomController"
         type="nav2_custom_controller::CustomController"
         base_class_type="nav2_core::Controller">
    <description>Custom path following controller</description>
  </class>
</library>
```

**CMakeLists.txt:**
```cmake
pluginlib_export_plugin_description_file(
    nav2_core custom_controller_plugins.xml)
```

**Key Points:**
- XML file describes plugin classes
- Links library path to class name
- Specifies base class type
- Description for documentation

### Example 3: ROS 2 Control Configuration

**URDF/Xacro (fishbot.ros2_control.xacro):**
```xml
<ros2_control name="FishBotGazeboSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-100</param>
      <param name="max">100</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

**Controller YAML (fishbot_ros2_controller.yaml):**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    fishbot_diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

fishbot_diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.2
    wheel_radius: 0.032
    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
```

**Key Points:**
- `ros2_control` block in URDF defines hardware
- Hardware plugin interfaces with Gazebo/simulator
- Controllers configured via YAML
- Controller manager loads and manages controllers

### Example 4: Custom Motion Control Plugin

```cpp
#include "motion_control_system/motion_control_interface.hpp"

namespace motion_control_system {

class SpinMotionController : public MotionController {
public:
    void start() override {
        std::cout << "SpinMotionController::start" << std::endl;
        // Start spinning motion
    }
    
    void stop() override {
        std::cout << "SpinMotionController::stop" << std::endl;
        // Stop motion
    }
};

} // namespace motion_control_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    motion_control_system::SpinMotionController,
    motion_control_system::MotionController)
```

**Key Points:**
- Define interface/base class
- Implement interface methods
- Register with pluginlib
- Load dynamically at runtime

### Example 5: Loading Plugins in Navigation2

**nav2_params.yaml:**
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_custom_controller::CustomController"
      max_linear_speed: 0.5
      max_angular_speed: 1.0

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_custom_planner::CustomPlanner"
```

**Key Points:**
- Plugins specified in parameter files
- Plugin name maps to class name
- Parameters passed to plugin via configure()
- Multiple plugins can be loaded

## Key Takeaways

1. **Plugin Architecture**: ROS 2 uses plugins extensively. Understanding pluginlib is essential for extending functionality.

2. **Navigation2 Extensibility**: Navigation2 is highly extensible through plugins. You can customize every component.

3. **ROS 2 Control**: Provides unified interface for robot control. Abstracts hardware differences.

4. **Lifecycle Management**: Plugins often use lifecycle nodes for proper initialization and cleanup.

5. **Interface Design**: Define clear interfaces for plugins. Makes them reusable and testable.

6. **Parameter Configuration**: Plugins are configured via parameters. Makes them flexible without recompilation.

7. **Plugin Registration**: 
   - Export class with `PLUGINLIB_EXPORT_CLASS`
   - Create plugin XML file
   - Export in CMakeLists.txt

8. **Hardware Abstraction**: ROS 2 Control separates hardware from control algorithms. Enables simulation and real robot with same controllers.

## Related Topics

- **Chapter 7**: Navigation2 basics (prerequisite)
- **Chapter 6**: URDF/Xacro (needed for ros2_control)
- **Chapter 10**: Advanced ROS 2 features (plugin loading, composition)

## Resources

- [Navigation2 Plugin Tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html)
- [ROS 2 Control Documentation](https://control.ros.org/)
- [pluginlib Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Pluginlib.html)
- [Creating a Controller Plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html)

