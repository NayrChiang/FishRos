#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>
#include "ch4_interfaces/srv/patrol.hpp"

using Patrol = ch4_interfaces::srv::Patrol;
using namespace std::chrono_literals;

class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Service<Patrol>::SharedPtr patrol_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};
    double max_speed_{3.0};

public:
    explicit TurtleControlNode(const std::string &node_name) : Node(node_name)
    {
        patrol_service_ = this->create_service<Patrol>("patrol", [&](const Patrol::Request::SharedPtr request, Patrol::Response::SharedPtr response) -> void
                                                       {
            if(
                (0<request->target_x&&request->target_x<12.0f)&&
                (0<request->target_y&&request->target_y<12.0f)
            ){
                this->target_x_ = request->target_x;
                this->target_y_ = request->target_y;
                response->result = Patrol::Response::SUCCESS;
            }else{
                response->result = Patrol::Response::FAIL;
            } });
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10,
                                                                      std::bind(&TurtleControlNode::on_pose_received_, this, std::placeholders::_1));
    };

    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto current_x = pose->x;
        auto current_y = pose->y;

        RCLCPP_INFO(get_logger(), "Current x: %f, y: %f", current_x, current_y);

        auto dist = std::sqrt((target_x_ - current_x) * (target_x_ - current_x) + (target_y_ - current_y) * (target_y_ - current_y));

        auto angle = std::atan2((target_y_ - current_y), (target_x_ - current_x)) - pose->theta;

        auto msg = geometry_msgs::msg::Twist();
        if (dist > 0.1)
        {
            if (fabs(angle) > 0.2)
            {
                msg.angular.z = fabs(angle);
            }
            else
            {
                msg.linear.x = k_ * dist;
            };
        };

        if (msg.linear.x > max_speed_)
        {
            msg.linear.x = max_speed_;
        };

        publisher_->publish(msg);
    };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>("turtle_control_node");
    RCLCPP_INFO(node->get_logger(), "Turtle Control Node Initialized.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};