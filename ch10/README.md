# Chapter 10: Advanced ROS 2 Features

## Overview

This chapter covers advanced ROS 2 features that enable sophisticated robot systems: Quality of Service (QoS) policies, executors and callback groups, lifecycle nodes, message filters for time synchronization, DDS configuration, and component composition. These features provide fine-grained control over system behavior, performance, and reliability.

## Key Concepts

### Quality of Service (QoS)

QoS policies control how ROS 2 handles message delivery:
- **Reliability**: Guaranteed delivery vs best-effort
- **Durability**: Transient (keep last) vs volatile
- **History**: Keep last N messages vs keep all
- **Deadline**: Maximum time between messages
- **Lifespan**: Maximum age of messages
- **Liveliness**: Publisher liveliness lease duration

### Executors

Executors manage callback execution:
- **SingleThreadedExecutor**: Sequential callback execution
- **MultiThreadedExecutor**: Parallel callback execution
- **StaticSingleThreadedExecutor**: Optimized single-threaded
- **Callback Groups**: Control which callbacks can run in parallel

### Lifecycle Nodes

Lifecycle nodes have managed states:
- **Unconfigured**: Initial state
- **Inactive**: Configured but not active
- **Active**: Running and processing
- **Finalized**: Shutdown state
- **State Transitions**: Controlled activation/deactivation

### Message Filters

Message filters synchronize messages from multiple topics:
- **Time Synchronization**: Align messages by timestamp
- **Policies**: Exact time, approximate time, latest time
- **Use Cases**: Sensor fusion, multi-sensor processing

### DDS Configuration

DDS (Data Distribution Service) is the middleware:
- **Transport**: Shared memory, UDP, TCP
- **Discovery**: How nodes find each other
- **Performance Tuning**: Optimize for specific use cases
- **Configuration Files**: XML-based DDS profiles

### Component Composition

Component composition enables:
- **Intra-process Communication**: Zero-copy message passing
- **Dynamic Loading**: Load components at runtime
- **Resource Efficiency**: Reduced overhead
- **Modular Design**: Reusable components

## Code Examples

### Example 1: QoS Configuration

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomNode : public rclcpp::Node {
public:
    OdomNode() : Node("odom_node") {
        // Create custom QoS profile
        rclcpp::QoS qos_profile(10);  // Queue depth
        
        // Set reliability policy
        qos_profile.reliability(
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        
        // Set durability policy
        qos_profile.durability(
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        
        // Set history policy
        qos_profile.history(
            RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        // Set deadline (1 second)
        qos_profile.deadline(rclcpp::Duration(1, 0));
        
        // Use custom QoS for publisher
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", qos_profile);
        
        // Use predefined QoS for subscriber
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "Received odometry");
            });
    }
};
```

**Key Points:**
- Customize QoS for specific use cases
- Predefined profiles: `SensorDataQoS()`, `ServicesQoS()`, etc.
- Publisher and subscriber QoS must be compatible
- Different policies for different scenarios

### Example 2: Executors and Callback Groups

```cpp
#include <rclcpp/rclcpp.hpp>

class LearnExecutorNode : public rclcpp::Node {
public:
    LearnExecutorNode() : Node("learn_executor") {
        // Create mutually exclusive callback group
        service_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // Create reentrant callback group (allows parallel execution)
        // auto reentrant_group = this->create_callback_group(
        //     rclcpp::CallbackGroupType::Reentrant);
        
        // Service with callback group
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&LearnExecutorNode::callback, this,
                     std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_callback_group_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LearnExecutorNode>();
    
    // Single-threaded executor (sequential callbacks)
    // auto executor = rclcpp::executors::SingleThreadedExecutor();
    
    // Multi-threaded executor (parallel callbacks)
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
```

**Key Points:**
- **MutuallyExclusive**: Callbacks in group cannot run in parallel
- **Reentrant**: Callbacks in group can run in parallel
- **SingleThreadedExecutor**: All callbacks sequential
- **MultiThreadedExecutor**: Callbacks can run in parallel (respects callback groups)

### Example 3: Lifecycle Node

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class LearnLifeCycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    LearnLifeCycleNode() : LifecycleNode("lifecyclenode") {
        timer_period_ = 1.0;
        timer_ = nullptr;
    }
    
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override {
        // Configure node (load parameters, etc.)
        timer_period_ = 1.0;
        RCLCPP_INFO(get_logger(), "on_configure(): Configure timer_period");
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
        // Activate node (start timers, publishers, etc.)
        timer_ = create_wall_timer(
            std::chrono::seconds(static_cast<int>(timer_period_)),
            [this]() { RCLCPP_INFO(get_logger(), "Timer active"); });
        RCLCPP_INFO(get_logger(), "on_activate(): Handle activate command");
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override {
        // Deactivate node (stop timers, etc.)
        timer_.reset();
        RCLCPP_INFO(get_logger(), "on_deactivate(): Handle deactivate command");
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override {
        // Cleanup resources
        timer_.reset();
        RCLCPP_INFO(get_logger(), "on_shutdown(): Handle shutdown command");
        return CallbackReturn::SUCCESS;
    }
    
private:
    rclcpp::TimerBase::SharedPtr timer_;
    double timer_period_;
};
```

**Key Points:**
- Inherit from `rclcpp_lifecycle::LifecycleNode`
- Implement state transition callbacks
- States: Unconfigured → Inactive → Active → Inactive → Finalized
- Controlled activation/deactivation

### Example 4: Message Filters and Time Synchronization

```cpp
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

using Imu = sensor_msgs::msg::Imu;
using Odometry = nav_msgs::msg::Odometry;
using namespace message_filters;

// Choose synchronization policy
// ExactTime: Messages must have exact same timestamp
// ApproximateTime: Messages within time window
// LatestTime: Use latest available messages
using MySyncPolicy = sync_policies::ApproximateTime<Imu, Odometry>;

class TimeSyncTestNode : public rclcpp::Node {
public:
    TimeSyncTestNode() : Node("sync_node") {
        // Create subscribers
        imu_sub_ = std::make_shared<Subscriber<Imu>>(this, "imu");
        odom_sub_ = std::make_shared<Subscriber<Odometry>>(this, "odom");
        
        // Create synchronizer
        synchronizer_ = std::make_shared<Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10),  // Queue size
            *imu_sub_, *odom_sub_);
        
        // Register callback for synchronized messages
        synchronizer_->registerCallback(
            std::bind(&TimeSyncTestNode::result_callback, this,
                     std::placeholders::_1, std::placeholders::_2));
    }
    
private:
    void result_callback(const Imu::ConstSharedPtr imu_msg,
                        const Odometry::ConstSharedPtr odom_msg) {
        // Process synchronized messages
        RCLCPP_INFO(get_logger(), "Synchronized: imu and odom");
    }
    
    std::shared_ptr<Subscriber<Imu>> imu_sub_;
    std::shared_ptr<Subscriber<Odometry>> odom_sub_;
    std::shared_ptr<Synchronizer<MySyncPolicy>> synchronizer_;
};
```

**Key Points:**
- Synchronize messages from multiple topics
- Different policies for different use cases
- Callback receives synchronized messages
- Useful for sensor fusion

### Example 5: Intra-Process Communication

```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    
    // Enable intra-process communication
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    
    auto talker = std::make_shared<Talker>(options);
    auto listener = std::make_shared<Listener>(options);
    
    executor.add_node(talker);
    executor.add_node(listener);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
```

**Key Points:**
- Zero-copy message passing within process
- Use `use_intra_process_comms(true)` in NodeOptions
- More efficient than inter-process communication
- Requires both nodes in same process

### Example 6: Loaned Messages

```cpp
#include <rclcpp/loaned_message.hpp>

class LoanedMessagePublisher : public rclcpp::Node {
public:
    LoanedMessagePublisher() : Node("loaned_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            "topic", 10);
        
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [&]() {
            // Borrow message from publisher (zero-copy)
            auto message = publisher_->borrow_loaned_message();
            
            // Modify message directly
            message.get().data = count_++;
            
            // Publish (move semantics)
            publisher_->publish(std::move(message));
        });
    }
};
```

**Key Points:**
- `borrow_loaned_message()` gets pre-allocated message
- Zero-copy for intra-process communication
- Use `std::move()` when publishing
- More efficient than creating new messages

### Example 7: DDS Configuration

**topic_sub_limit.xml:**
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <!-- Default publisher configuration -->
    <publisher profile_name="default_publisher" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </publisher>
    
    <!-- Default subscriber configuration -->
    <subscriber profile_name="default_subscriber" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </subscriber>
    
    <!-- Topic-specific configuration -->
    <publisher profile_name="/chatter">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <matchedSubscribersAllocation>
            <initial>0</initial>
            <maximum>1</maximum>
            <increment>1</increment>
        </matchedSubscribersAllocation>
    </publisher>
</profiles>
```

**Usage:**
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/topic_sub_limit.xml
ros2 run package node
```

**Key Points:**
- Configure DDS behavior via XML
- Control memory allocation
- Limit subscribers per topic
- Optimize for specific use cases

## Key Takeaways

1. **QoS Policies**: Match publisher and subscriber QoS. Incompatible QoS prevents communication.

2. **Reliability**:
   - **RELIABLE**: Guaranteed delivery (slower, more overhead)
   - **BEST_EFFORT**: Faster, may drop messages (good for sensors)

3. **Durability**:
   - **VOLATILE**: New subscribers don't get old messages
   - **TRANSIENT_LOCAL**: New subscribers get last N messages

4. **Executors**:
   - Single-threaded: Simple, predictable
   - Multi-threaded: Better performance, need callback groups
   - Use callback groups to control parallelism

5. **Lifecycle Nodes**: Use for nodes that need controlled startup/shutdown (sensors, controllers).

6. **Message Filters**: Essential for sensor fusion. Choose policy based on timing requirements.

7. **Intra-Process Communication**: Use when nodes are in same process. Zero-copy, more efficient.

8. **DDS Configuration**: Tune for performance. Shared memory for local, UDP/TCP for network.

9. **Loaned Messages**: Use for high-frequency topics. Reduces memory allocations.

10. **Component Composition**: Load components dynamically. Better resource usage.

## Related Topics

- **Chapter 3**: Topics (QoS affects topic communication)
- **Chapter 4**: Services (QoS affects service reliability)
- **Chapter 7**: Navigation2 (uses lifecycle nodes)
- **Chapter 8**: Plugins (component composition)

## Resources

- [ROS 2 QoS Documentation](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [Executors Documentation](https://docs.ros.org/en/humble/Guides/Executors.html)
- [Lifecycle Nodes](https://docs.ros.org/en/humble/Tutorials/Intermediate/Lifecycle/Lifecycle-Nodes.html)
- [Message Filters](https://github.com/ros2/message_filters)
- [DDS Configuration](https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html)
- [Component Composition](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)

