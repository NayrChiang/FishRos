# Chapter 3: Topics and Communication

## Overview

This chapter introduces the topic-based communication mechanism in ROS 2. Topics enable nodes to communicate asynchronously through a publish-subscribe pattern. You'll learn how to create publishers and subscribers in both C++ and Python, work with different message types, and understand the fundamentals of ROS 2 communication.

## Key Concepts

### Topic Communication Pattern

Topics implement a **publish-subscribe** communication pattern:
- **Publisher**: Node that sends messages to a topic
- **Subscriber**: Node that receives messages from a topic
- **Topic**: Named bus where messages are exchanged
- **Message Type**: Data structure defining the message format

### Key Characteristics

- **Asynchronous**: Publishers and subscribers operate independently
- **Many-to-Many**: Multiple publishers and subscribers can use the same topic
- **Decoupled**: Publishers don't need to know about subscribers and vice versa
- **Queue-based**: Messages are buffered in queues

### Message Types

ROS 2 provides standard message types in packages like:
- `std_msgs` - Basic types (String, Int32, Float64, etc.)
- `geometry_msgs` - Geometry types (Twist, Pose, Point, etc.)
- `sensor_msgs` - Sensor data (LaserScan, Image, etc.)
- `nav_msgs` - Navigation messages (Odometry, Path, etc.)

## Code Examples

### Example 1: C++ Publisher

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

public:
    TurtleControlNode() : Node("turtle_control_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
    }

    void publish_command()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 1.0;
        msg.angular.z = 0.5;
        publisher_->publish(msg);
    }
};
```

**Key Points:**
- `create_publisher<MessageType>(topic_name, queue_size)` creates a publisher
- Queue size determines how many messages to buffer
- `publish(msg)` sends the message to the topic

### Example 2: C++ Subscriber

```cpp
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;

public:
    TurtleControlNode() : Node("turtle_control_node")
    {
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleControlNode::on_pose_received, this, 
                     std::placeholders::_1));
    }

    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose)
    {
        RCLCPP_INFO(get_logger(), "Pose x: %f, y: %f", pose->x, pose->y);
    }
};
```

**Key Points:**
- `create_subscription<MessageType>(topic_name, queue_size, callback)` creates a subscriber
- Callback function is called when a message is received
- Use `std::bind` or lambda functions for callbacks

### Example 3: Python Publisher

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novel_publisher_ = self.create_publisher(String, 'novel', 10)
        self.create_timer(5, self.timer_callback)
        
    def timer_callback(self):
        msg = String()
        msg.data = "Hello from publisher"
        self.novel_publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
```

**Key Points:**
- `create_publisher(MessageType, topic_name, queue_size)` creates a publisher
- Create message instance and set data fields
- `publish(msg)` sends the message

### Example 4: Python Subscriber

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novel_subscriber_ = self.create_subscription(
            String, 'novel', self.novel_callback, 10)
        
    def novel_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

**Key Points:**
- `create_subscription(MessageType, topic_name, callback, queue_size)` creates a subscriber
- Callback function receives the message as parameter
- Process received data in the callback

## Key Takeaways

1. **Topic Naming**: Use descriptive names with forward slashes (e.g., `/turtle1/cmd_vel`). Names are case-sensitive.

2. **Queue Size**: Determines how many messages to buffer. Too small may cause message loss, too large may cause memory issues. Common values: 10-100.

3. **Message Creation**: 
   - C++: Create message object, set fields, then publish
   - Python: Create message instance, set data attributes, then publish

4. **Callback Functions**: 
   - Called automatically when messages arrive
   - Should process quickly to avoid blocking
   - Can use threading for time-consuming operations

5. **Topic Inspection**: Use command-line tools:
   - `ros2 topic list` - List all topics
   - `ros2 topic echo /topic_name` - Print messages from a topic
   - `ros2 topic info /topic_name` - Get topic information
   - `ros2 topic hz /topic_name` - Check publishing rate

6. **Message Types**: Always use the correct message type. Mismatched types will prevent communication.

7. **Asynchronous Nature**: Publishers and subscribers don't need to be running simultaneously. Messages are queued.

## Related Topics

- **Chapter 2**: Package structure (prerequisite for creating topic nodes)
- **Chapter 4**: Services and actions (alternative communication patterns)
- **Chapter 5**: Launch files (running multiple nodes with topics)
- **Chapter 10**: QoS policies (advanced topic configuration)

## Resources

- [ROS 2 Topics Documentation](https://docs.ros.org/en/humble/Concepts/About-Topics.html)
- [Standard Message Types](https://github.com/ros2/common_interfaces)
- [Creating a Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Topic Command Line Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

