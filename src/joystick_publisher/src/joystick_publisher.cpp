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
#include "interfaces/msg/arm_cmd.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

bool isEqual(interfaces::msg::ArmCmd a, interfaces::msg::ArmCmd b)
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
      publisher_ = this->create_publisher<interfaces::msg::ArmCmd>("arm_base_commands", 10);
      timer_ = this->create_wall_timer(
      600ms, std::bind(&JoystickReader::publish_message, this));//*/
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<interfaces::msg::ArmCmd>::SharedPtr publisher_;
    interfaces::msg::ArmCmd oldCmd;
    rclcpp::TimerBase::SharedPtr timer_;
    bool shouldPub = false;
		
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->axes[1]);
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
      if (msg->axes[0] > 0.4)
      {
        poseCmd.pose.position.y = 1;
      }
      if (msg->axes[0] < -0.4)
      {
        poseCmd.pose.position.y = -1;
      }
      if (msg->axes[1] > 0.4)
      {
        poseCmd.pose.position.x = 1;
      }
      if (msg->axes[1] < -0.4)
      {
        poseCmd.pose.position.x = -1;
      }
      if (msg->buttons[9] == 1)
      {
        poseCmd.pose.position.z = 1;
      }
      if (msg->buttons[10] == 1)
      {
        poseCmd.pose.position.z = -1;
      }
      if (msg->axes[2] > 0.4)
      {
        poseCmd.pose.orientation.x = -1;
      }
      if (msg->axes[2] < -0.4)
      {
        poseCmd.pose.orientation.x = 1;
      }
      if (msg->axes[3] > 0.4)
      {
        poseCmd.pose.orientation.y = 1;
      }
      if (msg->axes[3] < -0.4)
      {
        poseCmd.pose.orientation.y = -1;
      }
      if (msg->axes[4] == -1)
      {
        poseCmd.pose.orientation.z = 1;
      }
      if (msg->axes[5] == -1)
      {
        poseCmd.pose.orientation.z = -1;
      }
      if (msg->buttons[3] == 1)
      {
        poseCmd.pose.orientation.w = 1;
      }
      if (msg->buttons[1] == 1)
      {
        poseCmd = []{
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
					msg.reset = true;
					return msg;
				}();
        //shouldPub = true;
      }
      if (!isEqual(poseCmd, oldCmd))
      {
        /*RCLCPP_INFO(this->get_logger(), "A pose: %f %f %f %f %f %f %f %f %i %i",
					oldCmd.pose.position.x,
					oldCmd.pose.position.y,
					oldCmd.pose.position.z,
					oldCmd.pose.orientation.x,
					oldCmd.pose.orientation.y,
					oldCmd.pose.orientation.z,
					oldCmd.pose.orientation.w,
					oldCmd.speed,
					int(oldCmd.estop),
					int(oldCmd.reset));//
				RCLCPP_INFO(this->get_logger(), "B pose: %f %f %f %f %f %f %f %f %i %i",
					poseCmd.pose.position.x,
					poseCmd.pose.position.y,
					poseCmd.pose.position.z,
					poseCmd.pose.orientation.x,
					poseCmd.pose.orientation.y,
					poseCmd.pose.orientation.z,
					poseCmd.pose.orientation.w,
					poseCmd.speed,
					int(poseCmd.estop),
					int(poseCmd.reset));//*/
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
		    RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f %f %i %i",
					oldCmd.pose.position.x,
					oldCmd.pose.position.y,
					oldCmd.pose.position.z,
					oldCmd.pose.orientation.x,
					oldCmd.pose.orientation.y,
					oldCmd.pose.orientation.z,
					oldCmd.pose.orientation.w,
					oldCmd.speed,
					int(oldCmd.estop),
					int(oldCmd.reset));//*/
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
