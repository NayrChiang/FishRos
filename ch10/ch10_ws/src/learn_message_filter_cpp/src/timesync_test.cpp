#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/latest_time.h"
#include "message_filters/time_synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using Imu = sensor_msgs::msg::Imu;
using Odometry = nav_msgs::msg::Odometry;
using namespace message_filters;

// Synchronization policy: Strict time alignment policy
// using MySyncPolicy = sync_policies::ExactTime<Imu, Odometry>;
// Synchronization policy: Approximate time alignment policy
// using MySyncPolicy = sync_policies::ApproximateTime<Imu, Odometry>;
// Synchronization policy: Latest time alignment policy
using MySyncPolicy = sync_policies::LatestTime<Imu, Odometry>;


class TimeSyncTestNode : public rclcpp::Node {
public:
  TimeSyncTestNode() : Node("sync_node") {
    // 1. Subscribe to imu topic and register callback to print timestamp
    imu_sub_ = std::make_shared<Subscriber<Imu>>(this, "imu");
    imu_sub_->registerCallback<Imu::SharedPtr>(
        [&](const Imu::SharedPtr &imu_msg) {
          RCLCPP_INFO(get_logger(), "imu(%u,%u)", imu_msg->header.stamp.sec,
                      imu_msg->header.stamp.nanosec);
        });
    // 2. Subscribe to odom topic and register callback to print timestamp
    odom_sub_ = std::make_shared<Subscriber<Odometry>>(this, "odom");
    odom_sub_->registerCallback<Odometry::SharedPtr>(
        [&](const Odometry::SharedPtr &odom_msg) {
          RCLCPP_INFO(get_logger(), "odom(%u,%u)", odom_msg->header.stamp.sec,
                      odom_msg->header.stamp.nanosec);
        });
    // 3. Create synchronizer with corresponding policy to sync two topics and register callback to print data
    // synchronizer_ = std::make_shared<Synchronizer<MySyncPolicy>>(
    //     MySyncPolicy(10), *imu_sub_, *odom_sub_);
    
    // 3. Create synchronizer with corresponding policy to sync two topics and register callback to print data
    synchronizer_ = std::make_shared<Synchronizer<MySyncPolicy>>(
        MySyncPolicy(), *imu_sub_, *odom_sub_);
    synchronizer_->registerCallback(
        std::bind(&TimeSyncTestNode::result_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  void result_callback(const Imu::ConstSharedPtr imu_msg,
                       const Odometry::ConstSharedPtr odom_msg) {
    RCLCPP_INFO(get_logger(), "imu(%u,%u),odom(%u,%u))",
                imu_msg->header.stamp.sec, imu_msg->header.stamp.nanosec,
                odom_msg->header.stamp.sec, odom_msg->header.stamp.nanosec);
  }

  std::shared_ptr<Subscriber<Imu>> imu_sub_;
  std::shared_ptr<Subscriber<Odometry>> odom_sub_;
  std::shared_ptr<Synchronizer<MySyncPolicy>> synchronizer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeSyncTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}