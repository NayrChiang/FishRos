import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    default_patrol_config = os.path.join(get_package_share_directory('autopatrol_robot'), 'config', 'patrol_config.yaml')
    # Get package paths
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    
    # Include Gazebo simulation launch file
    gazebo_sim_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_description_dir, '/launch', '/gazebo_sim.launch.py']
        )
    )
    
    # Include Navigation2 launch file
    navigation2_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_navigation2_dir, '/launch', '/navigation2.launch.py']
        )
    )
    
    # Patrol node
    patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_node',
        name='patrol_node',
        parameters=[default_patrol_config]
    )

    speaker_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='speaker',
        name='speaker',
    )
    
    return launch.LaunchDescription([
        patrol_node,
        speaker_node,
    ])