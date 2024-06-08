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

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("arm_base_commands", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));//*/
    }

  private:
    void timer_callback()
    {
      std::string cmd = "0000000";
      geometry_msgs::msg::Pose poseCmd = []{
				geometry_msgs::msg::Pose msg;
				msg.position.x = 0;
				msg.position.y = 0;
				msg.position.z = 0;
				msg.orientation.x = 0;
				msg.orientation.y = 0;
				msg.orientation.z = 0;
				msg.orientation.w = 0;
				return msg;
			}();
      char c;
      std::cin>>c;
      if (c == 'w')
      {
        poseCmd.position.x = 1;
      }
      else if (c == 's')
      {
        poseCmd.position.x = -1;
      }
      else if (c == 'a')
      {
        poseCmd.position.y = 1;
      }
      else if (c == 'd')
      {
        poseCmd.position.y = -1;
      }
      else if (c == 'z')
      {
        poseCmd.position.z = 1;
      }
      else if (c == 'x')
      {
        poseCmd.position.z = -1;
      }
      else if (c == 'r')
      {
        poseCmd.orientation.x = 1;
      }
      else if (c == 't')
      {
        poseCmd.orientation.x = -1;
      }
      else if (c == 'f')
      {
        poseCmd.orientation.y = 1;
      }
      else if (c == 'g')
      {
        poseCmd.orientation.y = -1;
      }
      else if (c == 'c')
      {
        poseCmd.orientation.z = 1;
      }
      else if (c == 'v')
      {
        poseCmd.orientation.z = -1;
      }
      //auto message = std_msgs::msg::String();
      //geometry_msgs::msg::Pose message;
      //message.data = cmd;
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(poseCmd);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
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
