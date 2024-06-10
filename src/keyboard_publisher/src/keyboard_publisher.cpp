#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/msg/arm_cmd.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<interfaces::msg::ArmCmd>("arm_base_commands", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));//*/
    }

  private:
    void timer_callback()
    {
      interfaces::msg::ArmCmd poseCmd = []{
				interfaces::msg::ArmCmd msg;
				msg.pose.position.x = 0;
				msg.pose.position.y = 0;
				msg.pose.position.z = 0;
				msg.pose.orientation.x = 0;
				msg.pose.orientation.y = 0;
				msg.pose.orientation.z = 0;
				msg.pose.orientation.w = 0;
				msg.speed = 10;
				msg.estop = false;
				msg.reset = false;
				return msg;
			}();
      char c;
      std::cin>>c;
      if (c == 'w')
      {
        poseCmd.pose.position.x = 1;
      }
      else if (c == 's')
      {
        poseCmd.pose.position.x = -1;
      }
      else if (c == 'a')
      {
        poseCmd.pose.position.y = 1;
      }
      else if (c == 'd')
      {
        poseCmd.pose.position.y = -1;
      }
      else if (c == 'z')
      {
        poseCmd.pose.position.z = 1;
      }
      else if (c == 'x')
      {
        poseCmd.pose.position.z = -1;
      }
      else if (c == 'r')
      {
        poseCmd.pose.orientation.x = 1;
      }
      else if (c == 't')
      {
        poseCmd.pose.orientation.x = -1;
      }
      else if (c == 'f')
      {
        poseCmd.pose.orientation.y = 1;
      }
      else if (c == 'g')
      {
        poseCmd.pose.orientation.y = -1;
      }
      else if (c == 'c')
      {
        poseCmd.pose.orientation.z = 1;
      }
      else if (c == 'v')
      {
        poseCmd.pose.orientation.z = -1;
      }
      //auto message = std_msgs::msg::String();
      //geometry_msgs::msg::Pose message;
      //message.data = cmd;
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(poseCmd);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::ArmCmd>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  
  /*rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<MinimalPublisher>();
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });
  spinner.join();*/
  rclcpp::shutdown();
  return 0;
}
