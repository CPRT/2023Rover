#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

bool isEqual(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b)
{
  return a == b;
}

class JoystickReader : public rclcpp::Node
{
  public:
    JoystickReader()
    : Node("JoystickReader")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickReader::topic_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("arm_base_commands", 10);
      timer_ = this->create_wall_timer(
      600ms, std::bind(&JoystickReader::publish_message, this));//*/
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    geometry_msgs::msg::Pose oldCmd;
    rclcpp::TimerBase::SharedPtr timer_;
    bool shouldPub = false;
		
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->axes[1]);
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
      if (msg->axes[0] > 0.4)
      {
        poseCmd.position.y = 1;
      }
      if (msg->axes[0] < -0.4)
      {
        poseCmd.position.y = -1;
      }
      if (msg->axes[1] > 0.4)
      {
        poseCmd.position.x = 1;
      }
      if (msg->axes[1] < -0.4)
      {
        poseCmd.position.x = -1;
      }
      if (msg->buttons[9] == 1)
      {
        poseCmd.position.z = 1;
      }
      if (msg->buttons[10] == 1)
      {
        poseCmd.position.z = -1;
      }
      if (msg->axes[2] > 0.4)
      {
        poseCmd.orientation.x = -1;
      }
      if (msg->axes[2] < -0.4)
      {
        poseCmd.orientation.x = 1;
      }
      if (msg->axes[3] > 0.4)
      {
        poseCmd.orientation.y = 1;
      }
      if (msg->axes[3] < -0.4)
      {
        poseCmd.orientation.y = -1;
      }
      if (msg->axes[4] == -1)
      {
        poseCmd.orientation.z = 1;
      }
      if (msg->axes[5] == -1)
      {
        poseCmd.orientation.z = -1;
      }
      if (msg->buttons[3] == 1)
      {
        poseCmd.orientation.w = 1;
      }
      if (msg->buttons[1] == 1)
      {
        poseCmd = []{
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
        shouldPub = true;
      }
      if (!isEqual(poseCmd, oldCmd))
      {
        oldCmd = poseCmd;
        shouldPub = true;
		    /*auto message = std_msgs::msg::String();
		    message.data = cmd;
		    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		    publisher_->publish(message);*/
		  }
    }
    
    void publish_message()
    {
      if (shouldPub)
      {
		    shouldPub = false;
		    RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
					oldCmd.position.x,
					oldCmd.position.y,
					oldCmd.position.z,
					oldCmd.orientation.x,
					oldCmd.orientation.y,
					oldCmd.orientation.z,
					oldCmd.orientation.w);//*/
			  publisher_->publish(oldCmd);
			}
    }
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickReader>());
  rclcpp::shutdown();
  return 0;
}
