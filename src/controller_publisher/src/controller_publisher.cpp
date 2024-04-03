#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      std::string cmd = "0000000";
      char c;
      std::cin>>c;
      if (c == 'w')
      {
        cmd[0] = '0'+1;
      }
      else if (c == 's')
      {
        cmd[0] = '0'-1;
      }
      else if (c == 'a')
      {
        cmd[1] = '0'+1;
      }
      else if (c == 'd')
      {
        cmd[1] = '0'-1;
      }
      else if (c == 'z')
      {
        cmd[2] = '0'+1;
      }
      else if (c == 'x')
      {
        cmd[2] = '0'-1;
      }
      else if (c == 'r')
      {
        cmd[3] = '0'+1;
      }
      else if (c == 't')
      {
        cmd[3] = '0'-1;
      }
      else if (c == 'f')
      {
        cmd[4] = '0'+1;
      }
      else if (c == 'g')
      {
        cmd[4] = '0'-1;
      }
      else if (c == 'c')
      {
        cmd[5] = '0'+1;
      }
      else if (c == 'v')
      {
        cmd[5] = '0'-1;
      }
      else if (c == 'b')
      {
        cmd[6] = '0'+1;
      }
      else if (c == 'n')
      {
        cmd[6] = '0'-1;
      }
      auto message = std_msgs::msg::String();
      message.data = cmd;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<MinimalPublisher>());
  
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MinimalPublisher>();
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });//*/
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
