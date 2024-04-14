#include <memory>
#include <unistd.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <string>
#include<unistd.h>

class TestNode : public rclcpp::Node
{
  public:
    TestNode(const rclcpp::NodeOptions &options)
    : Node("hello_moveit", options), node_ptr(std::make_shared<rclcpp::Node>("example_moveit")), executor_ptr(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {
      node_name = ((std::string) this->get_namespace()).erase(0, 1);//"hello_moveit";
      //node_name.pop_back();
      
      RCLCPP_INFO(this->get_logger(), node_name.c_str());
      
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&TestNode::topic_callback, this, std::placeholders::_1));
      
      //auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(node_name + "_ur_manipulator", node_name, "rover_arm");
      
      move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "rover_arm");
      executor_ptr->add_node(node_ptr);
      executor_thread = std::thread([this]() {this->executor_ptr->spin(); });
			
    }
    
    /*void setup(moveit::planning_interface::MoveGroupInterface* new_interface)
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
    }*/

  private:
    //moveit::planning_interface::MoveGroupInterface* move_group_interface;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    std::string node_name;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;
    rclcpp::Node::SharedPtr node_ptr;
    rclcpp::Executor::SharedPtr executor_ptr;
    std::thread executor_thread;
    
    void topic_callback(const std_msgs::msg::String & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      std::string cmd = msg.data.c_str();
      
      auto myid = std::this_thread::get_id();
			std::stringstream ss;
			ss << myid;
			std::string mystring = ss.str();
      
      RCLCPP_INFO(this->get_logger(), mystring.c_str());
      //RCLCPP_INFO(this->get_logger(), *(this->get_clock()->now()).to_msg().data.c_str());
      RCLCPP_INFO(this->get_logger(), std::to_string(this->get_clock()->now().seconds()).c_str());
      //usleep(10000);
      
      //move_group_interface->setStartStateToCurrentState();
      //geometry_msgs::msg::Pose current_pose = move_group_interface->getCurrentPose().pose;
      
      move_group_ptr->setStartStateToCurrentState();
      geometry_msgs::msg::Pose current_pose = move_group_ptr->getCurrentPose().pose;
      
      // Print the current pose of the end effector
			RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
				current_pose.position.x,
				current_pose.position.y,
				current_pose.position.z,
				current_pose.orientation.x,
				current_pose.orientation.y,
				current_pose.orientation.z,
				current_pose.orientation.w);//*/
    }
};

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  rclcpp::spin(node);

  // Create a ROS logger
  /*//auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });//*/

  // Create the MoveIt MoveGroup Interface
  /*auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "rover_arm");
  
  // Slightly hacky cyclic query
  bool needsUpdate = false;
  std::string cmd = "";
  
  node->setup(&move_group_interface);
  
  //rclcpp::spin(node);
  
  auto myid = std::this_thread::get_id();
	std::stringstream ss;
	ss << myid;
	std::string mystring = ss.str();
  
  //RCLCPP_INFO(logger, mystring.c_str());
  
  //rclcpp::spin(node);
  
  spinner.join();
  
  //RCLCPP_INFO(logger, "Blocking?");
  
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
