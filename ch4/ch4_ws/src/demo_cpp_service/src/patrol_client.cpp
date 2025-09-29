#include "rclcpp/rclcpp.hpp"
#include "ch4_interfaces/srv/patrol.hpp"
#include <chrono>
#include <ctime>
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using Patrol = ch4_interfaces::srv::Patrol;
using SetP = rcl_interfaces::srv::SetParameters;
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

    // Send request via client and request result
    SetP::Response::SharedPtr call_set_parameter(const rcl_interfaces::msg::Parameter &param)
    {
        auto param_client = this->create_client<SetP>("/turtle_control_node/set_parameters");
        while (!param_client->wait_for_service(1.0s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Waiting for service to start, rclcpp terminated.");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to start...");
        }
        auto request = std::make_shared<SetP::Request>();
        request->parameters.push_back(param);
        auto future = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        return response;
    }

    void update_server_param_k(double k)
    {
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;
        auto response = this->call_set_parameter(param);
        if (response == NULL)
        {
            RCLCPP_INFO(this->get_logger(), "Failed to Update Parameter.");
        }
        for (auto result : response->results)
        {
            if (result.successful == false)
            {
                RCLCPP_INFO(this->get_logger(), "Failed to Update Parameter. Cause: %s", result.reason.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Successfully Updated Parameter.");
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClientNode>("patrol_client_node");
    RCLCPP_INFO(node->get_logger(), "Patrol Client Node Initialized.");
    node->update_server_param_k(1.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};