#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>

using namespace std::chrono_literals;

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
        : Node("joint_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&JointPublisher::set_joint_positions, this));
    }

private:
    void set_joint_positions()
    {
        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = this->now();
        joint_state_msg->name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};  // Replace with your joint names
        joint_state_msg->position = {1.0, 0.5, -0.3, 0.0, 0.0, 0.0};  // Replace with desired joint angles in radians

        publisher_->publish(*joint_state_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing joint positions");
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}

