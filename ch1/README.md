# Chapter 1: ROS Basics and CMake Introduction

## Overview

This chapter introduces the fundamentals of ROS 2 and CMake. It covers creating basic ROS 2 nodes in both Python and C++, understanding the node lifecycle, and using CMake for building C++ programs.

## Key Concepts

### ROS 2 Node Structure

A ROS 2 node is the fundamental unit of computation in ROS. Every node performs a specific function and can communicate with other nodes through topics, services, or actions.

**Python Node Components:**
- `rclpy.init()` - Initialize ROS 2 Python client library
- `Node("node_name")` - Create a node instance with a name
- `get_logger()` - Get the node's logger for outputting messages
- `rclpy.spin(node)` - Keep the node alive and process callbacks
- `rclpy.shutdown()` - Clean shutdown of ROS 2

### CMake Basics

CMake is a cross-platform build system generator. In ROS 2, CMake is used to build C++ packages.

**Basic CMake Structure:**
- `cmake_minimum_required(VERSION 3.8)` - Specify minimum CMake version
- `project(ProjectName)` - Define the project name
- `add_executable(target_name source_files)` - Create an executable from source files

## Code Examples

### Example 1: Basic Python ROS 2 Node

```python
import rclpy 
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("python_node")
    node.get_logger().info('Hi Python Node')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

**Explanation:**
- `rclpy.init()` initializes the ROS 2 Python client library
- `Node("python_node")` creates a node named "python_node"
- `get_logger().info()` logs an informational message
- `rclpy.spin(node)` keeps the node running and processes callbacks
- `rclpy.shutdown()` properly shuts down the ROS 2 system

### Example 2: Basic C++ Program

```cpp
#include "iostream"

int main()
{
    std::cout << "Hello World C++" << std::endl;
    return 0;
}
```

**Explanation:**
- Standard C++ program demonstrating basic I/O
- This is a non-ROS program, used to learn CMake basics

### Example 3: Basic CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(HelloWorld)
add_executable(learn_cmake hello_world.cpp)
```

**Explanation:**
- Sets minimum CMake version to 3.8
- Defines project name as "HelloWorld"
- Creates executable "learn_cmake" from source file "hello_world.cpp"

## Key Takeaways

1. **Node Lifecycle**: Every ROS 2 node follows a pattern: initialize → create node → use node → spin → shutdown

2. **Logger Usage**: The logger is the primary way to output information from a node. Use `get_logger().info()`, `get_logger().warn()`, `get_logger().error()` for different log levels.

3. **Node Spinning**: `rclpy.spin()` is essential - it keeps the node alive and allows it to process callbacks. Without it, the node would exit immediately.

4. **CMake for ROS**: While this chapter shows basic CMake, ROS 2 packages use more complex CMakeLists.txt files that will be covered in later chapters.

5. **Python vs C++**: Python nodes are simpler to write and good for prototyping, while C++ nodes offer better performance for computationally intensive tasks.

## Related Topics

- **Chapter 2**: Learn about ROS 2 workspaces and packages, which build upon these basic node concepts
- **Chapter 3**: Discover how nodes communicate using topics
- **Chapter 4**: Learn about services and actions for request-response communication

## Resources

- [ROS 2 Python Client Library (rclpy) Documentation](https://docs.ros2.org/latest/api/rclpy/index.html)
- [CMake Tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
- [ROS 2 Node Documentation](https://docs.ros.org/en/humble/Concepts/About-Nodes.html)

