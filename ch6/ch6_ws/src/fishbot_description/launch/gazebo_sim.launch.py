# Ch 6.4
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # URDF path - using Fortress compatible version
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'fishbot/fishbot.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')
    
    # Declare modifiable URDF path parameter
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=str(default_xacro_path), 
        description='Model File Path'
    )
    
    # Get robot description information using file path
    command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result, value_type=str)
    
    # Robot state publisher
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )
    
    # Launch Gazebo Fortress
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']
        ),
        launch_arguments=[
            ('world', default_gazebo_world_path),
            ('verbose', 'true')
        ]
    )
    
    # Spawn robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'fishbot',
        ],
        output='screen'
    )
    
    return launch.LaunchDescription([
        action_declare_arg_model_path, 
        action_robot_state_publisher,
        action_launch_gazebo,
        spawn
    ])
