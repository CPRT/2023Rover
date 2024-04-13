#include <memory>
#include <unistd.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"

class TestNode : public rclcpp::Node
{
  public:
    TestNode(const rclcpp::NodeOptions &options)
    : Node("hello_moveit", options) 
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&TestNode::topic_callback, this, std::placeholders::_1));
			
    }
    
    void setup(moveit::planning_interface::MoveGroupInterface* new_interface)
    {
      move_group_interface = new_interface;
      RCLCPP_INFO(this->get_logger(), "Setup?");
      
      geometry_msgs::msg::Pose current_pose = move_group_interface->getCurrentPose().pose;

			// Print the current pose of the end effector
			RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
				current_pose.position.x,
				current_pose.position.y,
				current_pose.position.z,
				current_pose.orientation.x,
				current_pose.orientation.y,
				current_pose.orientation.z,
				current_pose.orientation.w);
    }

  private:
    moveit::planning_interface::MoveGroupInterface* move_group_interface;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void topic_callback(const std_msgs::msg::String & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      std::string cmd = msg.data.c_str();
      
      geometry_msgs::msg::Pose current_pose = move_group_interface->getCurrentPose().pose;
      
      // Print the current pose of the end effector
			RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
				current_pose.position.x,
				current_pose.position.y,
				current_pose.position.z,
				current_pose.orientation.x,
				current_pose.orientation.y,
				current_pose.orientation.z,
				current_pose.orientation.w);
    }
};

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<TestNode>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });
  

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "rover_arm");
  
  node->setup(&move_group_interface);
  
  //rclcpp::spin(node);
  
  spinner.join();
  
  /*geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;

	// Print the current pose of the end effector
	RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f %f",
		current_pose.position.x,
		current_pose.position.y,
		current_pose.position.z,
		current_pose.orientation.x,
		current_pose.orientation.y,
		current_pose.orientation.z,
		current_pose.orientation.w);*/


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
