# Chapter 7: Navigation2 and Autonomous Patrol

This chapter demonstrates autonomous navigation using ROS 2 Navigation2 stack with Gazebo simulation. The workspace includes a complete setup for running an autonomous patrol robot that can navigate through waypoints, save images, and perform patrol tasks.

## Overview

This workspace contains:
- **FishBot robot model** with sensors (LIDAR, IMU, camera)
- **Navigation2 configuration** for autonomous navigation
- **Autonomous patrol system** that navigates through predefined waypoints
- **Gazebo simulation** environment for testing

## Package Structure

```
ch7_ws/src/
├── fishbot_description/      # Robot URDF model, launch files, and Gazebo world
├── fishbot_navigation2/      # Navigation2 configuration and maps
├── autopatrol_robot/         # Autonomous patrol node and speaker service
└── autopatrol_interfaces/    # Custom service interfaces
```

## Prerequisites

### System Requirements
- Ubuntu 22.04 (Jammy) or compatible
- ROS 2 Humble Desktop
- Gazebo/Ignition Fortress (Garden is also supported)

### Required ROS 2 Packages

Install the following ROS 2 packages:

```bash
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    python3-pip \
    python3-colcon-common-extensions
```

### Python Dependencies

```bash
pip3 install opencv-python cv-bridge numpy
```

### Additional Dependencies

If you encounter issues with `tf_transformations`, install:

```bash
pip3 install transforms3d
```

Or use the ROS 2 version:

```bash
sudo apt install ros-humble-tf-transformations
```

## Installation

1. **Clone the repository** (if not already cloned):
   ```bash
   git clone <repository-url>
   cd FishRos/ch7
   ```

2. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Build the workspace**:
   ```bash
   cd ch7_ws
   colcon build --symlink-install
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Usage

### 1. Gazebo Simulation (`gazebo_sim.launch.py`)

This launch file starts the Gazebo simulation with the FishBot robot model, sensors, and ROS-Gazebo bridges.

**Purpose:**
- Launch Gazebo Fortress/Garden with the custom room world
- Spawn the FishBot robot with sensors (LIDAR, IMU, camera)
- Set up ROS-Gazebo bridges for sensor data and control
- Start Robot State Publisher for TF transforms

**Usage:**

```bash
# Basic usage
ros2 launch fishbot_description gazebo_sim.launch.py

# With custom URDF model
ros2 launch fishbot_description gazebo_sim.launch.py model:=/path/to/your/model.urdf.xacro

# Launch with RViz (if enabled in launch file)
ros2 launch fishbot_description gazebo_sim.launch.py
```

**What it does:**
1. Launches Gazebo with the `custom_room.world` environment
2. Spawns the FishBot robot at position (0, 0, 0.02)
3. Creates ROS-Gazebo bridges for:
   - `/cmd_vel` - Robot velocity commands
   - `/odom` - Odometry data
   - `/scan` - LIDAR scan data
   - `/imu` - IMU data
   - `/camera/image_raw` - Camera images
   - `/joint_states` - Joint state information
   - `/tf` - Transform tree
   - `/clock` - Simulation time

**Topics available:**
- `/cmd_vel` (geometry_msgs/Twist) - Control the robot
- `/scan` (sensor_msgs/LaserScan) - LIDAR data
- `/odom` (nav_msgs/Odometry) - Odometry
- `/camera/image_raw` (sensor_msgs/Image) - Camera feed
- `/imu` (sensor_msgs/Imu) - IMU data

### 2. Navigation2 Launch (`navigation2.launch.py`)

This launch file starts the Navigation2 stack with AMCL localization and path planning.

**Purpose:**
- Launch Navigation2 bringup with map server
- Configure AMCL for robot localization
- Set up path planning and navigation controllers
- Load navigation parameters and map

**Usage:**

```bash
# Basic usage (uses default map)
ros2 launch fishbot_navigation2 navigation2.launch.py

# With custom map
ros2 launch fishbot_navigation2 navigation2.launch.py map:=/path/to/your/map.yaml

# With custom parameters
ros2 launch fishbot_navigation2 navigation2.launch.py params_file:=/path/to/nav2_params.yaml

# Disable simulation time (for real robot)
ros2 launch fishbot_navigation2 navigation2.launch.py use_sim_time:=false
```

**Launch Arguments:**
- `use_sim_time` (default: `true`) - Use simulation time from Gazebo
- `map` (default: `maps/room.yaml`) - Path to map YAML file
- `params_file` (default: `config/nav2_params.yaml`) - Navigation2 parameters file

**What it does:**
1. Launches Navigation2 bringup with:
   - Map server loading the room map
   - AMCL for localization
   - Nav2 planner, controller, and recovery behaviors
   - Behavior tree navigator
2. Opens RViz2 with Navigation2 default view
3. Configures all nodes to use simulation time

**Prerequisites:**
- Gazebo simulation must be running (from `gazebo_sim.launch.py`)
- Robot must be spawned in Gazebo
- Map file must exist at the specified path

**Important Notes:**
- The robot's initial pose should be set using `init_robot_pose` node before navigation
- The map file (`room.yaml`) should reference the corresponding PGM file (`room.pgm`)

### 3. Autonomous Patrol Launch (`autopatrol.launch.py`)

This launch file starts the complete autonomous patrol system, including simulation, navigation, and patrol nodes.

**Purpose:**
- Launch the complete autonomous patrol system
- Start Gazebo simulation and Navigation2
- Run the patrol node that navigates through waypoints
- Enable speaker service for status announcements

**Usage:**

```bash
# Complete autonomous patrol system
ros2 launch autopatrol_robot autopatrol.launch.py
```

**What it includes:**
1. **Gazebo Simulation** (from `gazebo_sim.launch.py`)
2. **Navigation2 Stack** (from `navigation2.launch.py`)
3. **Patrol Node** - Autonomous waypoint navigation
4. **Speaker Node** - Text-to-speech service for announcements

**Configuration:**

The patrol behavior is configured in `autopatrol_robot/config/patrol_config.yaml`:

```yaml
/patrol_node:
  ros__parameters:
    initial_point: [0.0, 0.0, 0.0]  # Initial robot pose [x, y, yaw]
    target_points: [                # Waypoints [x1, y1, yaw1, x2, y2, yaw2, ...]
      0.0,  0.0,   0.0,
      1.0,  2.0,   3.14,
     -4.5,  1.5,   1.57,
     -8.0, -5.0,   1.57,
      1.0, -5.0,   3.14
    ]
    use_sim_time: false
```

**Patrol Node Behavior:**
1. Initializes robot pose in the map
2. Navigates to each waypoint in sequence
3. Saves camera images at each waypoint (if configured)
4. Announces status via speaker service
5. Repeats the patrol cycle

**Note:** Currently, the launch file has Gazebo and Navigation2 launches commented out. To run the full system, uncomment lines 44-45 in `autopatrol.launch.py`:

```python
gazebo_sim_launch,
navigation2_launch,
```

## Running the Complete System

### Option 1: Run All Components Separately

**Terminal 1 - Gazebo Simulation:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

**Terminal 2 - Navigation2:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

**Terminal 3 - Initialize Robot Pose:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 run fishbot_application init_robot_pose
```

**Terminal 4 - Autonomous Patrol:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

### Option 2: Run Individual Navigation Commands

**Terminal 1 - Gazebo + Navigation2:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 launch fishbot_description gazebo_sim.launch.py
# In another terminal:
ros2 launch fishbot_navigation2 navigation2.launch.py
```

**Terminal 2 - Initialize Pose:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 run fishbot_application init_robot_pose
```

**Terminal 3 - Navigate to Single Pose:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 run fishbot_application nav2pose --ros-args -p x:=2.0 -p y:=1.0 -p yaw:=1.57
```

**Terminal 4 - Follow Waypoints:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 run fishbot_application waypoint_follower
```

**Terminal 5 - Get Current Robot Pose:**
```bash
cd ch7/ch7_ws
source install/setup.bash
ros2 run fishbot_application get_robot_pose
```

## Troubleshooting

### Gazebo Not Starting
- Ensure Gazebo/Ignition is installed: `sudo apt install gazebo-fortress` or `gazebo-garden`
- Check that `ros_gz_sim` package is installed
- Verify world file exists: `ch7_ws/src/fishbot_description/world/custom_room.world`

### Navigation2 Not Localizing
- Make sure robot pose is initialized using `init_robot_pose`
- Check that map file exists and is valid
- Verify `/scan` topic is publishing LIDAR data
- Check RViz2 to see if robot appears on the map

### Patrol Node Not Working
- Ensure Navigation2 is fully active (wait for "Nav2 is ready" message)
- Check that robot pose is initialized before starting patrol
- Verify waypoints are within map bounds
- Check speaker service is running if using text-to-speech

### Build Errors
- Make sure all dependencies are installed
- Clean build: `rm -rf build install log` then `colcon build --symlink-install`
- Check Python version compatibility (Python 3.10 recommended for Humble)

### TF Errors
- Ensure `robot_state_publisher` is running
- Check that `/tf` and `/tf_static` topics are publishing
- Verify base frame is `base_footprint` in navigation parameters

## Useful Commands

**View all topics:**
```bash
ros2 topic list
```

**Monitor robot pose:**
```bash
ros2 topic echo /amcl_pose
```

**Monitor navigation goal:**
```bash
ros2 topic echo /navigate_to_pose/_action/feedback
```

**Check Navigation2 status:**
```bash
ros2 service list | grep nav2
```

**View robot transforms:**
```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

**Save current robot pose:**
```bash
ros2 run fishbot_application get_robot_pose
```

## Configuration Files

- **Navigation Parameters**: `fishbot_navigation2/config/nav2_params.yaml`
- **Patrol Configuration**: `autopatrol_robot/config/patrol_config.yaml`
- **Robot Controller**: `fishbot_description/config/fishbot_ros2_controller.yaml`
- **RViz Config**: `fishbot_description/config/fishbot_config.rviz`

## Map Files

The default map is located at:
- Map YAML: `fishbot_navigation2/maps/room.yaml`
- Map Image: `fishbot_navigation2/maps/room.pgm`

To use a custom map, provide the path to the YAML file when launching Navigation2.

## License

This project is licensed under Apache-2.0 License.

## Maintainer

- nayr (ryan881028@gmail.com)

