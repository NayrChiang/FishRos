# Chapter 4: Services and Actions

## Overview

This chapter introduces two additional communication patterns in ROS 2: **Services** and **Actions**. Unlike topics (which are asynchronous and one-way), services provide synchronous request-response communication, and actions provide asynchronous request-response with feedback. You'll learn when to use each communication pattern and how to implement service servers/clients and action servers/clients.

## Key Concepts

### Service Communication Pattern

Services implement a **request-response** communication pattern:
- **Service Server**: Node that provides a service and responds to requests
- **Service Client**: Node that sends requests and receives responses
- **Service**: Named endpoint for request-response communication
- **Service Type**: Defines the request and response message structure (`.srv` file)

**Characteristics:**
- **Synchronous**: Client waits for response (blocking or async)
- **One-to-One**: One client, one server per request
- **Request-Response**: Client sends request, server processes and responds
- **Use Cases**: Configuration, queries, one-time operations

### Action Communication Pattern

Actions implement an **asynchronous request-response with feedback** pattern:
- **Action Server**: Node that executes long-running tasks and provides feedback
- **Action Client**: Node that sends goals and receives feedback/result
- **Action**: Named endpoint for goal-feedback-result communication
- **Action Type**: Defines goal, feedback, and result messages (`.action` file)

**Characteristics:**
- **Asynchronous**: Client doesn't block waiting for completion
- **Feedback**: Server can send periodic updates during execution
- **Cancellable**: Goals can be cancelled mid-execution
- **Use Cases**: Long-running tasks, navigation, manipulation

### When to Use Each Pattern

| Pattern | Use When | Example |
|---------|----------|---------|
| **Topics** | Continuous data stream, many-to-many | Sensor data, robot state |
| **Services** | Quick request-response, configuration | Get robot status, set parameters |
| **Actions** | Long-running tasks with feedback | Navigate to goal, pick and place |

## Code Examples

### Example 1: Service Definition (.srv file)

```srv
# FaceDetector.srv
sensor_msgs/Image image 
---
int16 number
float32 use_time
int32[] top
int32[] bottom
int32[] right
int32[] left
```

**Structure:**
- Lines above `---` define the **request** message
- Lines below `---` define the **response** message
- Can use any ROS message type

### Example 2: Python Service Server

```python
import rclpy
from rclpy.node import Node
from ch4_interfaces.srv import FaceDetector

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.service_ = self.create_service(
            FaceDetector, 'face_detector', self.detect_face_callback)
        
    def detect_face_callback(self, request, response):
        # Process request
        # request.image contains the input image
        
        # Perform face detection
        face_locations = face_recognition.face_locations(request.image)
        
        # Fill response
        response.number = len(face_locations)
        response.use_time = processing_time
        
        return response
```

**Key Points:**
- `create_service(ServiceType, service_name, callback)` creates a service server
- Callback receives `request` and `response` objects
- Modify `response` and return it
- Callback should be fast (blocking for other clients)

### Example 3: Python Service Client

```python
import rclpy
from rclpy.node import Node
from ch4_interfaces.srv import FaceDetector

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.client = self.create_client(FaceDetector, 'face_detector')
        
    def send_request(self):
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        # Create request
        request = FaceDetector.Request()
        request.image = image_msg
        
        # Send request (async)
        future = self.client.call_async(request)
        
        # Process response when ready
        def result_callback(result_future):
            response = result_future.result()
            self.get_logger().info(f'Found {response.number} faces')
        
        future.add_done_callback(result_callback)
```

**Key Points:**
- `create_client(ServiceType, service_name)` creates a service client
- `wait_for_service()` checks if server is available
- `call_async(request)` sends request asynchronously
- Use callback or `spin_until_future_complete()` to get response

### Example 4: C++ Service Client

```cpp
#include "rclcpp/rclcpp.hpp"
#include "ch4_interfaces/srv/patrol.hpp"

class PatrolClientNode : public rclcpp::Node
{
private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;

public:
    PatrolClientNode() : Node("patrol_client_node")
    {
        patrol_client_ = this->create_client<Patrol>("patrol");
    }

    void send_request()
    {
        // Wait for service
        while (!patrol_client_->wait_for_service(1.0s)) {
            RCLCPP_INFO(get_logger(), "Waiting for service...");
        }
        
        // Create request
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = 5.0;
        request->target_y = 3.0;
        
        // Send async request
        auto future = patrol_client_->async_send_request(
            request,
            [this](rclcpp::Client<Patrol>::SharedFuture result_future) {
                auto response = result_future.get();
                if (response->result == Patrol::Response::SUCCESS) {
                    RCLCPP_INFO(get_logger(), "Patrol successful!");
                }
            });
    }
};
```

**Key Points:**
- `create_client<ServiceType>(service_name)` creates client
- `wait_for_service()` blocks until service available
- `async_send_request()` with lambda callback for async handling
- Use `future.get()` in callback to access response

### Example 5: Actions (Conceptual)

Actions are similar to services but support:
- **Goal**: Initial request (like service request)
- **Feedback**: Periodic updates during execution
- **Result**: Final outcome (like service response)

**Action Server Flow:**
1. Receive goal from client
2. Accept/reject goal
3. Execute task
4. Send periodic feedback
5. Send final result

**Action Client Flow:**
1. Send goal to server
2. Receive feedback (optional)
3. Receive result when complete
4. Can cancel goal if needed

## Key Takeaways

1. **Service vs Topic**: Services are for request-response, topics are for continuous data streams.

2. **Blocking vs Non-blocking**: 
   - `call()` - Synchronous, blocks until response
   - `call_async()` - Asynchronous, returns future

3. **Service Availability**: Always check `wait_for_service()` before sending requests.

4. **Service Callbacks**: Should be fast. Long operations should use actions instead.

5. **Custom Service Types**: Define in `.srv` files in an interfaces package, build to generate code.

6. **Actions for Long Tasks**: Use actions when:
   - Task takes significant time (>1 second)
   - Need progress feedback
   - Task might need cancellation

7. **Parameter Updates via Services**: ROS 2 nodes expose `set_parameters` service for runtime parameter updates.

## Related Topics

- **Chapter 3**: Topics (alternative communication pattern)
- **Chapter 5**: Launch files (starting service nodes)
- **Chapter 10**: QoS policies (affect service reliability)

## Resources

- [ROS 2 Services Documentation](https://docs.ros.org/en/humble/Concepts/About-Services.html)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Concepts/About-Actions.html)
- [Creating a Service](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Creating an Action](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

