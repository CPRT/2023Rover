#include <memory>
#include <unistd.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using moveit::planning_interface::MoveGroupInterface;

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
      
      geometry_msgs::msg::Pose target_pose = []{
				geometry_msgs::msg::Pose msg;
				msg.orientation.w = 1.0;
				msg.position.x = 0.636922;
				msg.position.y = 0.064768;
				msg.position.z = 0.678810;
				return msg;
			}();
			move_group_ptr->setPoseTarget(target_pose);

			// Create a plan to that target pose
			auto const [success, plan] = [&]{
				moveit::planning_interface::MoveGroupInterface::Plan msg;
				auto const ok = static_cast<bool>(move_group_ptr->plan(msg));
				return std::make_pair(ok, msg);
			}();

			// Execute the plan
			if(success) {
				move_group_ptr->execute(plan);
			} else {
				RCLCPP_ERROR(this->get_logger(), "Planing failed!");
			}
			
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
      
      std::vector<geometry_msgs::msg::Pose> points;
      
      move_group_ptr->setStartStateToCurrentState();
      geometry_msgs::msg::Pose current_pose = move_group_ptr->getCurrentPose().pose;
      
      points.push_back(current_pose);
      
      // Print the current pose of the end effector
			RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
				current_pose.position.x,
				current_pose.position.y,
				current_pose.position.z,
				current_pose.orientation.x,
				current_pose.orientation.y,
				current_pose.orientation.z,
				current_pose.orientation.w);//*/
				
				auto const new_pose = [&]{
				geometry_msgs::msg::Pose msg = current_pose;
				//msg.position.y += 0.2;
				msg.position.x += (cmd[0] - '0')/10.0;
				msg.position.y += (cmd[1] - '0')/10.0;
				msg.position.z += (cmd[2] - '0')/10.0;
				
				msg.orientation.x += (cmd[3] - '0')/10.0;
				msg.orientation.y += (cmd[4] - '0')/10.0;
				msg.orientation.z += (cmd[5] - '0')/10.0;
				msg.orientation.w += (cmd[6] - '0')/10.0;
				
				return msg;
			}();
			
			points.push_back(new_pose);
			
			RCLCPP_INFO(this->get_logger(), "New pose: %f %f %f %f %f %f %f",
				new_pose.position.x,
				new_pose.position.y,
				new_pose.position.z,
				new_pose.orientation.x,
				new_pose.orientation.y,
				new_pose.orientation.z,
				new_pose.orientation.w);
		  
		  moveit_msgs::msg::RobotTrajectory trajectory;
			const double jump_threshold = 0;
			const double eef_step = 0.01;
			//double fraction = move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
			move_group_ptr->computeCartesianPath(points, eef_step, jump_threshold, trajectory);
			//RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
			move_group_ptr->execute(trajectory);
			
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
}

/*class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(const rclcpp::NodeOptions &options)
    : Node("minimal_subscriber", options), node_(std::make_shared<rclcpp::Node>("example_group_node")),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) 
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      
			executor_->add_node(node_);
      executor_thread_ = std::thread([this]() { this->executor_->spin(); });
      
      target_pose = []{
				geometry_msgs::msg::Pose msg;
				msg.orientation.w = 1.0;
				msg.position.x = 0.636922;
				msg.position.y = 0.064768;
				msg.position.z = 0.678810;
				return msg;
			}();
			move_group_interface.setPoseTarget(target_pose);

			// Create a plan to that target pose
			auto const [success, plan] = [&]{
				moveit::planning_interface::MoveGroupInterface::Plan msg;
				auto const ok = static_cast<bool>(move_group_interface.plan(msg));
				return std::make_pair(ok, msg);
			}();

			// Execute the plan
			if(success) {
				move_group_interface.execute(plan);
			} else {
				RCLCPP_ERROR(this->get_logger(), "Planing failed!");
			}
      
      //print random crap
			geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;

			// Print the current pose of the end effector
			RCLCPP_INFO(this->get_logger(), "Initial pose: %f %f %f %f %f %f %f",
				current_pose.position.x,
				current_pose.position.y,
				current_pose.position.z,
				current_pose.orientation.x,
				current_pose.orientation.y,
				current_pose.orientation.z,
				current_pose.orientation.w);
				
		  
			
			current_pose = target_pose;
			
			RCLCPP_INFO(this->get_logger(), "Supposed-to-be pose: %f %f %f %f %f %f %f",
				current_pose.position.x,
				current_pose.position.y,
				current_pose.position.z,
				current_pose.orientation.x,
				current_pose.orientation.y,
				current_pose.orientation.z,
				current_pose.orientation.w);
			
    }

  private:
    moveit::planning_interface::MoveGroupInterface move_group_interface = MoveGroupInterface(std::make_shared<rclcpp::Node>(this->get_name()), "rover_arm");
    geometry_msgs::msg::Pose target_pose;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    //random stuff
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
    
    void topic_callback(const std_msgs::msg::String & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      std::string cmd = msg.data.c_str();
      
      std::vector<geometry_msgs::msg::Pose> points;
			// print current pose
			//geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
			
			geometry_msgs::msg::Pose current_pose = target_pose;
			points.push_back(current_pose);

			// Print the current pose of the end effector
			RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
				current_pose.position.x,
				current_pose.position.y,
				current_pose.position.z,
				current_pose.orientation.x,
				current_pose.orientation.y,
				current_pose.orientation.z,
				current_pose.orientation.w);
				
			auto const new_pose = [&]{
				geometry_msgs::msg::Pose msg = current_pose;
				//msg.position.y += 0.2;
				msg.position.x += (cmd[0] - '0')/10.0;
				msg.position.y += (cmd[1] - '0')/10.0;
				msg.position.z += (cmd[2] - '0')/10.0;
				
				msg.orientation.x += (cmd[3] - '0')/10.0;
				msg.orientation.y += (cmd[4] - '0')/10.0;
				msg.orientation.z += (cmd[5] - '0')/10.0;
				msg.orientation.w += (cmd[6] - '0')/10.0;
				
				return msg;
			}();
			
			RCLCPP_INFO(this->get_logger(), "New pose: %f %f %f %f %f %f %f",
				new_pose.position.x,
				new_pose.position.y,
				new_pose.position.z,
				new_pose.orientation.x,
				new_pose.orientation.y,
				new_pose.orientation.z,
				new_pose.orientation.w);
			
			move_group_interface.setPoseTarget(new_pose);

			// Create a plan to that target pose
			auto const [success, plan] = [&]{
				moveit::planning_interface::MoveGroupInterface::Plan msg;
				auto const ok = static_cast<bool>(move_group_interface.plan(msg));
				return std::make_pair(ok, msg);
			}();

			// Execute the plan
			if(success) {
				move_group_interface.execute(plan);
			} else {
				RCLCPP_ERROR(this->get_logger(), "Planing failed!");
			}//*/
			
			//points.push_back(new_pose);
			
			/*moveit_msgs::msg::RobotTrajectory trajectory;
			const double jump_threshold = 0;
			const double eef_step = 0.01;
			//double fraction = move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
			move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
			//RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
			move_group_interface.execute(trajectory);
			target_pose = points[1];//
			//sleep(1000);
			target_pose = points[1];
			
			
			//print random crap
			current_pose = move_group_interface.getCurrentPose().pose;

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
};*/


// Create a ROS logger
//rclcpp::Logger logger;

/*void callback(const std_msgs::msg::String & msg) const
{
  RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str());
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hhhhh",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  
  //logger
  auto const logger = rclcpp::get_logger("hhhhh");
  
  //subscription things
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(callback, this, _1));

	rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = MoveGroupInterface(node, "rover_arm");

  // Set a target Pose
  auto const target_pose = []{
		geometry_msgs::msg::Pose msg;
		msg.orientation.w = 1.0;
		msg.position.x = 0.636922;
		msg.position.y = 0.064768;
		msg.position.z = 0.678810;
		return msg;
	}();
	move_group_interface.setPoseTarget(target_pose);

	// Create a plan to that target pose
	auto const [success, plan] = [&move_group_interface]{
		moveit::planning_interface::MoveGroupInterface::Plan msg;
		auto const ok = static_cast<bool>(move_group_interface.plan(msg));
		return std::make_pair(ok, msg);
	}();

	// Execute the plan
	if(success) {
		move_group_interface.execute(plan);
	} else {
		//RCLCPP_ERROR(logger, "Planing failed!");
	}
	
	//cartesian things
  /*for (int i = 0; i < 3; i++)
  {
    std::vector<geometry_msgs::msg::Pose> points;
		// print current pose
		geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
		
		points.push_back(current_pose);

		// Print the current pose of the end effector
		RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
		  current_pose.position.x,
		  current_pose.position.y,
		  current_pose.position.z,
		  current_pose.orientation.x,
		  current_pose.orientation.y,
		  current_pose.orientation.z,
		  current_pose.orientation.w);
		  
		auto const new_pose = [&]{
			geometry_msgs::msg::Pose msg = current_pose;
			msg.position.y += 0.2;
			return msg;
		}();
		
		points.push_back(new_pose);
		
		moveit_msgs::msg::RobotTrajectory trajectory;
		const double jump_threshold = 0;
		const double eef_step = 0.01;
		//double fraction = move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
		move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
		//RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
		move_group_interface.execute(trajectory);
		//sleep(1000);
		
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}//*/
