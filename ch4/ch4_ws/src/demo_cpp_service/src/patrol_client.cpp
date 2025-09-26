#include "rclcpp/rclcpp.hpp"
#include "ch4_interfaces/srv/patrol.hpp"
#include <chrono>

using Patrol = ch4_interfaces::srv::Patrol;
using namespace std::chrono_literals;

class PatrolClientNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;

public:
    PatrolClientNode(const std::string &node_name) : Node(node_name)
    {
        srand(time(NULL));
        patrol_client_ = this->create_client<Patrol>("patrol");
        timer_ = this->create_wall_timer(10.0s, [&]() -> void
                                         {
            while (!this->patrol_client_ ->wait_for_service(1.0s)){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Waiting for service to start, rclcpp terminated.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Waiting for service to start...");

            } 
            auto request = std::make_shared<Patrol::Request>();
            request ->target_x = rand() %15;
            request ->target_y = rand() %15; 
            RCLCPP_INFO(this->get_logger(), "Random Point Set (X, Y): %f, %f", request->target_x, request->target_y);
            this->patrol_client_->async_send_request(request, [&](rclcpp::Client<Patrol>::SharedFuture result_future)->void{
                    auto response = result_future.get();
                    if(response->result==Patrol::Response::SUCCESS){
                        RCLCPP_INFO(this->get_logger(), "Successfully requested for patrol target!");
                    }
                    if(response->result==Patrol::Response::FAIL){
                        RCLCPP_INFO(this->get_logger(), "Failed to request for patrol target!");
                    }
            }); });
    };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClientNode>("patrol_client_node");
    RCLCPP_INFO(node->get_logger(), "Patrol Client Node Initialized.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};