#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

bool isEqual(std::string a, std::string b)
{
  if (a.length() != b.length())
  {
    return false;
  }
  for (int i = 0; i < int(a.length()); i++)
  {
    if (a[i] != b[i])
    {
      return false;
    }
  }
  return true;
}

class JoystickReader : public rclcpp::Node
{
  public:
    JoystickReader()
    : Node("JoystickReader")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickReader::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("arm_base_commands", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&JoystickReader::publish_message, this));//*/
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string oldCmd = "0000000";
    rclcpp::TimerBase::SharedPtr timer_;
    bool shouldPub = false;
    
    void setStr(const std::string a)
		{
			if (a.length() != oldCmd.length())
			{
				return;
			}
			for (int i = 0; i < int(a.length()); i++)
			{
				oldCmd[i] = a[i];
			}
			
		}
		
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->axes[1]);
      std::string cmd = "0000000";
      if (msg->axes[0] > 0.4)
      {
        cmd[1] = '0'+1;
      }
      if (msg->axes[0] < -0.4)
      {
        cmd[1] = '0'-1;
      }
      if (msg->axes[1] > 0.4)
      {
        cmd[0] = '0'+1;
      }
      if (msg->axes[1] < -0.4)
      {
        cmd[0] = '0'-1;
      }
      if (msg->buttons[9] == 1)
      {
        cmd[2] = '0'+1;
      }
      if (msg->buttons[10] == 1)
      {
        cmd[2] = '0'-1;
      }
      if (msg->axes[2] > 0.4)
      {
        cmd[3] = '0'-1;
      }
      if (msg->axes[2] < -0.4)
      {
        cmd[3] = '0'+1;
      }
      if (msg->axes[3] > 0.4)
      {
        cmd[4] = '0'+1;
      }
      if (msg->axes[3] < -0.4)
      {
        cmd[4] = '0'-1;
      }
      if (msg->axes[4] == -1)
      {
        cmd[5] = '0'+1;
      }
      if (msg->axes[5] == -1)
      {
        cmd[5] = '0'-1;
      }
      if (msg->buttons[3] == 1)
      {
        cmd[6] = '0'+1;
      }
      if (!isEqual(cmd, oldCmd))
      {
        setStr(cmd);
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
		    auto message = std_msgs::msg::String();
			  message.data = oldCmd;
			  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
			  publisher_->publish(message);
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
