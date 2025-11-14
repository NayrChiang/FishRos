# FishRos: ROS 2 Learning Journey

A comprehensive ROS 2 (Robot Operating System 2) learning project covering fundamental to advanced concepts through hands-on implementation. This repository documents a complete learning path from basic ROS 2 nodes to autonomous navigation and advanced system features.

## Overview

This project is a structured learning journey through ROS 2, organized into 10 chapters. Each chapter builds upon previous concepts, progressing from basic node creation to advanced topics like Navigation2, custom plugins, and real robot implementation.

**ROS 2 Distribution**: Humble Hawksbill  
**Ubuntu Version**: 22.04 LTS  
**Simulation**: Gazebo Fortress/Garden

## Learning Path

### Chapter 1: ROS Basics and CMake Introduction
**Topics**: Basic ROS 2 node structure, CMake fundamentals, node lifecycle  
**Key Concepts**: Node initialization, logger usage, CMake build system  
[View Chapter 1 Notes →](ch1/README.md)

### Chapter 2: ROS 2 Packages and Workspaces
**Topics**: Workspace structure, package creation, colcon build system  
**Key Concepts**: C++ and Python packages, package.xml, CMakeLists.txt, setup.py  
[View Chapter 2 Notes →](ch2/README.md)

### Chapter 3: Topics and Communication
**Topics**: Publisher-subscriber pattern, message types, topic communication  
**Key Concepts**: Topics, publishers, subscribers, message types, asynchronous communication  
[View Chapter 3 Notes →](ch3/README.md)

### Chapter 4: Services and Actions
**Topics**: Request-response communication, service clients/servers, action framework  
**Key Concepts**: Services, actions, when to use each communication pattern  
[View Chapter 4 Notes →](ch4/README.md)

### Chapter 5: Parameters and Launch Files
**Topics**: Runtime configuration, parameter system, launch file composition  
**Key Concepts**: Parameters, launch files, node configuration  
[View Chapter 5 Notes →](ch5/README.md)

### Chapter 6: Robot Description (URDF/Xacro) and TF
**Topics**: Robot modeling, URDF/Xacro, coordinate transforms  
**Key Concepts**: Links, joints, visual/collision properties, TF tree, static/dynamic transforms  
[View Chapter 6 Notes →](ch6/README.md)

### Chapter 7: Navigation2 and Autonomous Patrol
**Topics**: Autonomous navigation, localization, path planning, costmaps  
**Key Concepts**: Navigation2 stack, AMCL, path planning, autonomous waypoint navigation  
[View Chapter 7 Notes →](ch7/README.md)

### Chapter 8: Advanced Topics
**Topics**: Custom Navigation2 plugins, ROS 2 Control, plugin systems  
**Key Concepts**: Plugin architecture, custom controllers/planners, ROS 2 Control framework  
[View Chapter 8 Notes →](ch8/README.md)

### Chapter 9: Real Robot Implementation
**Topics**: Hardware interfaces, sensor integration, actuator control  
**Key Concepts**: ROS 2 Control for real robots, hardware abstraction, sensor/actuator drivers  
**Note**: Hardware components to be purchased later - focuses on ROS concepts  
[View Chapter 9 Notes →](ch9/README.md)

### Chapter 10: Advanced ROS 2 Features
**Topics**: QoS policies, executors, lifecycle nodes, message filters, DDS, composition  
**Key Concepts**: Quality of Service, callback groups, time synchronization, intra-process communication  
[View Chapter 10 Notes →](ch10/README.md)

## Project Structure

```
FishRos/
├── ch1/          # ROS basics and CMake
├── ch2/          # Packages and workspaces
├── ch3/          # Topics and communication
├── ch4/          # Services and actions
├── ch5/          # Parameters and launch files
├── ch6/          # URDF/Xacro and TF
├── ch7/          # Navigation2 and autonomous navigation
├── ch8/          # Advanced topics (plugins, ROS 2 Control)
├── ch9/          # Real robot implementation concepts
├── ch10/         # Advanced ROS 2 features (QoS, executors, etc.)
├── PLAN.md       # Project cleanup and documentation plan
└── README.md     # This file
```

Each chapter contains:
- Source code examples (C++ and/or Python)
- Package configurations (`package.xml`, `CMakeLists.txt`, `setup.py`)
- Comprehensive learning notes (`README.md`)
- Launch files and configuration files

## Prerequisites

### System Requirements
- **Ubuntu**: 22.04 LTS (Jammy) or compatible
- **ROS 2**: Humble Hawksbill Desktop
- **Gazebo**: Fortress or Garden (for simulation chapters)
- **Architecture**: ARM64 (Apple Silicon) or x86_64

### Required ROS 2 Packages

```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    python3-colcon-common-extensions
```

### Python Dependencies

```bash
pip3 install opencv-python cv-bridge numpy transforms3d
```

## Quick Start

### 1. Clone the Repository

```bash
git clone <repository-url>
cd FishRos
```

### 2. Build a Chapter Workspace

```bash
cd ch2/ch2_ws  # Example: Chapter 2 workspace
colcon build --symlink-install
source install/setup.bash
```

### 3. Run Examples

```bash
# Example: Run a node from Chapter 1
ros2 run demo_cpp_pkg cpp_node

# Example: Launch Navigation2 (Chapter 7)
ros2 launch fishbot_description gazebo_sim.launch.py
ros2 launch fishbot_navigation2 navigation2.launch.py
```

## Learning Journey Summary

### Foundation (Chapters 1-2)
Learn the basics of ROS 2: creating nodes, understanding packages, and building workspaces. These chapters establish the fundamental concepts needed for all subsequent learning.

### Communication (Chapters 3-5)
Master ROS 2 communication mechanisms: topics for continuous data streams, services for request-response, actions for long-running tasks, parameters for configuration, and launch files for system orchestration.

### Robot Modeling (Chapter 6)
Learn to describe robots using URDF and Xacro, understand coordinate frames and transforms (TF), and visualize robot models in RViz.

### Autonomous Navigation (Chapters 7-8)
Implement autonomous navigation using Navigation2, create custom plugins, and understand ROS 2 Control for robot motion control.

### Real-World Application (Chapters 9-10)
Prepare for real robot deployment and master advanced ROS 2 features for building sophisticated robotic systems.

## Key Learning Outcomes

After completing this learning journey, you will understand:

✅ **ROS 2 Architecture**: Nodes, topics, services, actions, parameters  
✅ **Build Systems**: CMake, colcon, Python setup tools  
✅ **Robot Description**: URDF/Xacro, TF, coordinate frames  
✅ **Navigation**: Navigation2 stack, AMCL, path planning, costmaps  
✅ **Advanced Features**: QoS, executors, lifecycle nodes, message filters  
✅ **System Design**: Plugin architecture, component composition, hardware abstraction  
✅ **Best Practices**: Code organization, launch file design, parameter management

## Resources

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [Gazebo Documentation](https://gazebosim.org/docs)

### Tutorials
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Navigation2 Tutorials](https://navigation.ros.org/tutorials/)

### Community
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)

## Project Status

✅ **Completed**: All 10 chapters with comprehensive documentation  
✅ **Code**: Source code examples in C++ and Python  
✅ **Documentation**: Detailed learning notes for each chapter  
✅ **Cleanup**: Build artifacts removed, ready for GitHub

## Contributing

This is a personal learning project. Feel free to use it as a reference or starting point for your own ROS 2 learning journey.

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.

## Author

**nayr** (ryan881028@gmail.com)

---

## Chapter Quick Reference

| Chapter | Focus Area | Key Topics |
|---------|------------|------------|
| [Ch1](ch1/README.md) | Basics | Nodes, CMake, Logger |
| [Ch2](ch2/README.md) | Workspaces | Packages, colcon, Build system |
| [Ch3](ch3/README.md) | Communication | Topics, Publishers, Subscribers |
| [Ch4](ch4/README.md) | Services | Services, Actions, Request-Response |
| [Ch5](ch5/README.md) | Configuration | Parameters, Launch files |
| [Ch6](ch6/README.md) | Robot Model | URDF, Xacro, TF |
| [Ch7](ch7/README.md) | Navigation | Navigation2, AMCL, Path Planning |
| [Ch8](ch8/README.md) | Advanced | Plugins, ROS 2 Control |
| [Ch9](ch9/README.md) | Real Robot | Hardware, Sensors, Actuators |
| [Ch10](ch10/README.md) | Advanced ROS 2 | QoS, Executors, Lifecycle, Filters |

---

**Happy Learning! 🤖**
