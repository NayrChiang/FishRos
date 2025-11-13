#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LearnLifeCycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  LearnLifeCycleNode()
      : rclcpp_lifecycle::LifecycleNode("lifecyclenode") {
    timer_period_ = 1.0;
    timer_ = nullptr;
    RCLCPP_INFO(get_logger(), "%s: Created", get_name());
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override {
    (void)state;
    timer_period_ = 1.0;
    RCLCPP_INFO(get_logger(), "on_configure(): Configure timer_period");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
    (void)state;
    timer_ = create_wall_timer(
        std::chrono::seconds(static_cast<int>(timer_period_)),
        [this]() { RCLCPP_INFO(get_logger(), "Timer printing in progress..."); });
    RCLCPP_INFO(get_logger(), "on_activate(): Handle activate command, create timer");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override {
    (void)state;
    timer_.reset();
    RCLCPP_INFO(get_logger(), "on_deactivate(): Handle deactivate command, stop timer");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override {
    (void)state;
    timer_.reset();
    RCLCPP_INFO(get_logger(), "on_shutdown(): Handle shutdown command");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double timer_period_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LearnLifeCycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}