// Ch 5.3.3
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;

public:
    TFListener() : Node("tf_listener")
    {
        this->buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        timer_ = this->create_wall_timer(1.0s, std::bind(&TFListener::getTransform, this));
    }

    void getTransform()
    {
        try
        {
            const auto transform = buffer_->lookupTransform("base_link", "target_point",
                                                            this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0f));
            auto translation = transform.transform.translation;
            auto rotation = transform.transform.rotation;
            double y, p, r;
            tf2::getEulerYPR(rotation, y, p, r);
            RCLCPP_INFO(get_logger(), "Translation: %f, %f, %f", translation.x, translation.y, translation.z);
            RCLCPP_INFO(get_logger(), "Rotation: %f, %f, %f", y, p, r);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "%s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}