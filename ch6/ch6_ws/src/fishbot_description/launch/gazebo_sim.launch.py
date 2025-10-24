# Ch 6.4
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    # URDF path - using Fortress compatible version
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'fishbot/fishbot.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')

    # 1) CLI arg for model path
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description='Model File Path'
    )

    # 2) Process xacro -> URDF string
    robot_description_cmd = launch.substitutions.Command(
        ['xacro ', LaunchConfiguration('model')]
    )

    # 3) Robot State Publisher consumes the param (and sim time)
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_cmd,
            'use_sim_time': True
        }],
        output='screen'
    )

    # 4) Launch Gazebo Fortress
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']
        ),
        launch_arguments=[
            ('gz_args', ['-v 4 -r ', default_gazebo_world_path])
        ]
    )

    # 5) Spawn robot in Gazebo from the *parameter* (not topic)
    action_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fishbot',
            '-param', 'robot_description',     # <-- changed from -topic
            # Optional pose at spawn:
            # '-x', '0', '-y', '0', '-z', '0.02'
        ],
        parameters=[{
            'robot_description': robot_description_cmd
        }],
        output='screen'
    )

    # 6) ROS 2 <-> Gazebo bridges
    action_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/fishbot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fishbot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fishbot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
        ],
        remappings=[
            ('/model/fishbot/cmd_vel', '/cmd_vel'),
            ('/model/fishbot/odometry', '/odom'),
            ('/model/fishbot/tf', '/tf')
        ],
        output='screen'
    )

    # 7) Small delay so Gazebo is up before spawning/bridging (optional)
    delayed_spawn = TimerAction(period=2.0, actions=[action_spawn_entity, action_ros_gz_bridge])

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        delayed_spawn
    ])