# Ch 6.2.2 - Fishbot Robot Display
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Fishbot URDF path
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'fishbot/fishbot.urdf.xacro')
    default_rviz_path = os.path.join(urdf_package_path, 'config', 'fishbot_config.rviz')
    
    # Declare a urdf path parameter to modify
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=str(default_urdf_path), 
        description='Fishbot Model File Path'
    )
    
    # Process xacro -> URDF string
    robot_description_cmd = launch.substitutions.Command(
        ['xacro ', LaunchConfiguration('model')]
    )
    robot_description_value = ParameterValue(robot_description_cmd, value_type=str)
    
    # Robot State Publisher
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_value,
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # Joint State Publisher (for manual joint control)
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # RViz2 for robot visualization
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_path],
        output='screen'
    )
    
    return launch.LaunchDescription([
        action_declare_arg_model_path, 
        action_robot_state_publisher, 
        action_joint_state_publisher,
        action_rviz_node
    ])