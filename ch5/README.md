# Chapter 5: Parameters and Launch Files

## Overview

This chapter covers ROS 2 **parameters** for runtime configuration and **launch files** for starting multiple nodes and configuring the system. Parameters allow nodes to be configured without code changes, while launch files provide a way to start and configure multiple nodes simultaneously.

## Key Concepts

### ROS 2 Parameters

Parameters are key-value pairs that can be set at runtime to configure node behavior. They provide a way to:
- Configure nodes without recompiling
- Change behavior at runtime
- Load configurations from files
- Update values dynamically

**Parameter Types:**
- `bool`, `int`, `float`, `string`
- Arrays of the above types
- Nested parameter structures

### Launch Files

Launch files are Python scripts that:
- Start multiple nodes simultaneously
- Configure node parameters
- Set up the ROS 2 environment
- Organize complex robot systems

**Launch File Benefits:**
- Reproducible system startup
- Centralized configuration
- Conditional execution
- Parameter passing

## Code Examples

### Example 1: Declaring Parameters (Python)

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameters with default values
        self.declare_parameter('number_of_times_to_upsample', 1)
        self.declare_parameter('model', 'hog')
        
        # Get parameter values
        self.number_of_times_to_upsample = self.get_parameter(
            'number_of_times_to_upsample').value
        self.model = self.get_parameter('model').value
        
        # Set up parameter change callback
        self.add_on_set_parameters_callback(self.param_callback)
    
    def param_callback(self, parameters):
        from rcl_interfaces.msg import SetParametersResult
        
        for parameter in parameters:
            self.get_logger().info(
                f"Parameter {parameter.name} changed to {parameter.value}")
            
            # Update internal variables based on parameter changes
            if parameter.name == 'model':
                self.model = parameter.value
        
        return SetParametersResult(successful=True)
```

**Key Points:**
- `declare_parameter(name, default_value)` declares a parameter
- `get_parameter(name)` retrieves parameter value
- `add_on_set_parameters_callback()` handles runtime parameter changes
- Callback must return `SetParametersResult`

### Example 2: Declaring Parameters (C++)

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        // Declare parameters
        this->declare_parameter("k", 1.0);
        this->declare_parameter("max_speed", 1.0);
        
        // Get parameter values
        this->get_parameter("k", k_);
        this->get_parameter("max_speed", max_speed_);
        
        // Set parameter callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) 
            -> rcl_interfaces::msg::SetParametersResult {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                
                for (const auto &parameter : parameters) {
                    if (parameter.get_name() == "k") {
                        k_ = parameter.as_double();
                    }
                }
                return result;
            });
    }

private:
    double k_;
    double max_speed_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr 
        parameter_callback_handle_;
};
```

**Key Points:**
- `declare_parameter(name, default_value)` declares parameter
- `get_parameter(name, variable)` gets value into variable
- Lambda functions or member functions can be used for callbacks

### Example 3: Basic Launch File

```python
import launch
import launch_ros

def generate_launch_description():
    # Create node action
    turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen'
    )
    
    # Create another node
    control_node = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control_node',
        output='screen'
    )
    
    return launch.LaunchDescription([
        turtlesim_node,
        control_node
    ])
```

**Key Points:**
- `generate_launch_description()` must return `LaunchDescription`
- `launch_ros.actions.Node()` creates a node action
- `output='screen'` prints to console (options: 'screen', 'log', 'both')

### Example 4: Launch File with Parameters

```python
import launch
import launch_ros

def generate_launch_description():
    # Declare launch argument
    background_g_arg = launch.actions.DeclareLaunchArgument(
        'launch_arg_bg_g', 
        default_value='150'
    )
    
    # Create node with parameters
    turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[{
            'background_g': launch.substitutions.LaunchConfiguration(
                'launch_arg_bg_g', 
                default='150'
            )
        }],
        output='screen'
    )
    
    return launch.LaunchDescription([
        background_g_arg,
        turtlesim_node
    ])
```

**Key Points:**
- `DeclareLaunchArgument()` creates a launch argument
- `LaunchConfiguration()` references launch arguments
- Parameters can be passed as dictionaries to nodes

### Example 5: Launch File with YAML Parameters

```python
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to parameter file
    param_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )
    
    node = launch_ros.actions.Node(
        package='my_package',
        executable='my_node',
        parameters=[param_file],  # Load from YAML file
        output='screen'
    )
    
    return launch.LaunchDescription([node])
```

**YAML Parameter File (params.yaml):**
```yaml
my_node:
  ros__parameters:
    k: 2.0
    max_speed: 3.0
    model: 'cnn'
    enabled: true
```

### Example 6: Launch File with Conditions

```python
import launch
import launch_ros

def generate_launch_description():
    # Declare argument
    use_gui_arg = launch.actions.DeclareLaunchArgument(
        'use_gui',
        default_value='false'
    )
    
    # Conditional node
    gui_node = launch_ros.actions.Node(
        package='rqt',
        executable='rqt',
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('use_gui')
        )
    )
    
    return launch.LaunchDescription([
        use_gui_arg,
        gui_node
    ])
```

## Key Takeaways

1. **Parameter Declaration**: Always declare parameters before using them. This makes them visible and configurable.

2. **Parameter Sources** (priority order):
   - Command line (`--ros-args -p param:=value`)
   - Launch file parameters
   - Parameter files (YAML)
   - Default values in code

3. **Parameter Updates**: Use `set_parameters` service or `ros2 param set` command to update parameters at runtime.

4. **Launch File Structure**:
   - Import `launch` and `launch_ros`
   - Define `generate_launch_description()` function
   - Return `LaunchDescription` with actions

5. **Launch Arguments**: Use `DeclareLaunchArgument` and `LaunchConfiguration` to make launch files configurable.

6. **Parameter Files**: YAML files provide clean separation of configuration from code.

7. **Launch File Execution**: Run with `ros2 launch package_name launch_file.py [args]`

8. **Parameter Inspection**: Use `ros2 param list` and `ros2 param get` to inspect node parameters.

## Related Topics

- **Chapter 2**: Package structure (needed for launch files)
- **Chapter 4**: Services (parameters can be updated via services)
- **Chapter 6**: Robot description (launch files often load URDF)

## Resources

- [ROS 2 Parameters Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html)
- [Launch Files Documentation](https://docs.ros.org/en/humble/How-To-Guides/Launch-files-migration-guide.html)
- [Creating a Launch File](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Using Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)

