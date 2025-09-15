#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 建立一個 node，名稱為 "cpp_node"
    auto node = std::make_shared<rclcpp::Node>("cpp_node");

    // 使用 ROS 2 logger 輸出訊息
    RCLCPP_INFO(node->get_logger(), "Hi C++ Node");

    // spin → 讓節點保持運行直到被終止
    rclcpp::spin(node);

    // 關閉 ROS 2
    rclcpp::shutdown();
    return 0;
}
