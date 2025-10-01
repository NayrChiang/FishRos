import launch 
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    action_declare_startup_rqt = launch.actions.DeclareLaunchArgument(
        'startup_rqt', default_value = "False")
    
    startup_rqt = launch.substitutions.LaunchConfiguration('startup_rqt', default="False")
    
    # Initialize other Launch
    multisim_path = [get_package_share_directory('turtlesim'), '/launch', '/multisim.launch.py']
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_path
        )
    )
    
    action_log_info = launch.actions.LogInfo(msg=str(multisim_path))
    
    action_topic_list = launch.actions.ExecuteProcess(
        condition = launch.conditions.IfCondition(startup_rqt),
        cmd = ['rqt']
    )
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=2.0, actions=[action_include_launch]),
        launch.actions.TimerAction(period=4.0, actions=[action_topic_list])        
    ])
    
    return launch.LaunchDescription([
        action_declare_startup_rqt,
        action_log_info,
        action_group
    ])