import launch 
import launch_ros

def generate_launch_description():
    action_node_turtlesim_node = launch_ros.actions.Node(
        package = 'turtlesim',
        executable = 'turtlesim_node',
        output = 'screen'
    )
    
    action_node_patrol_node = launch_ros.actions.Node(
        package = 'demo_cpp_service',
        executable = 'patrol_client_node',
        output = 'screen'
    )
    
    action_node_turtlesim_node = launch_ros.actions.Node(
        package = 'demo_cpp_service',
        executable = 'turtle_control_node',
        output = 'screen'
    )