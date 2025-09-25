#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <status_interfaces/msg/system_status.hpp>

using SystemStatus = status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;
    QLabel *label_;

public:
    SysStatusDisplay() : Node("sys_status_display")
    {
        label_ = new QLabel();

        subscriber_ = this->create_subscription<SystemStatus>("sys_status", 10,
                                                              [&](const SystemStatus::SharedPtr msg) -> void
                                                              {
                                                                  label_->setText(get_qstr_from_msg(msg));
                                                              });
        label_->setText(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->show();
    };

    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg)
    {
        std::stringstream show_str;
        show_str
            << "========== Status Display Tool ==========\n"
            << "Timestamp: \t\t" << msg->stamp.sec << "\ts\n"
            << "CPU Name: \t\t" << msg->host_name << "\t\t\n"
            << "CPU Percent: \t\t" << msg->cpu_percent << "\t\t%\n"
            << "Memory Percent: \t\t" << msg->memory_percent << "\t\t%\n"
            << "Memory Total: \t\t" << msg->memory_total << "\t\tMB\n"
            << "Memory Available: \t" << msg->memory_available << "\t\tMB\n"
            << "Net Sent: \t\t" << msg->net_sent << "\t\tMB\n"
            << "Net Received: \t\t" << msg->net_recv << "\t\tMB\n"
            << "=========================================";
        return QString::fromStdString(show_str.str());
    };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&]() -> void
                            {
                                rclcpp::spin(node);
                            });
    spin_thread.detach();
    return app.exec();
}
