#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node named "cpp_node"
    auto node = std::make_shared<rclcpp::Node>("cpp_node");

    // Use ROS 2 logger to output message
    RCLCPP_INFO(node->get_logger(), "Hi C++ Node");

    // spin - keep the node running until terminated
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
