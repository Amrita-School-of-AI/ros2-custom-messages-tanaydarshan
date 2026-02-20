#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_msgs/msg/robot_status.hpp"

using RobotStatus = ros2_custom_msgs::msg::RobotStatus;

class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher() : Node("status_publisher"),
                        battery_level_(100.0),
                        mission_count_(0)
    {
        publisher_ = this->create_publisher<RobotStatus>("/robot_status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&StatusPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        RobotStatus msg;
        msg.robot_name = "Explorer1";
        msg.battery_level = battery_level_;
        msg.is_active = true;
        msg.mission_count = mission_count_;

        publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: robot=%s, battery=%.1f, active=%s, missions=%d",
            msg.robot_name.c_str(),
            msg.battery_level,
            msg.is_active ? "true" : "false",
            msg.mission_count
        );

        battery_level_ -= 0.5;
        mission_count_++;
    }

    rclcpp::Publisher<RobotStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double battery_level_;
    int mission_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
