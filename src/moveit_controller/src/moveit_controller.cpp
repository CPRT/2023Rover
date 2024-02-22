#include <memory>
#include <unistd.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hhhhh",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hhhhh");

	rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
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
		RCLCPP_ERROR(logger, "Planing failed!");
	}//*/
	
	//cartesian things
  for (int i = 0; i < 3; i++)
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
		const double jump_threshold = 0.00;
		const double eef_step = 0.01;
		double fraction = move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
		//RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
		move_group_interface.execute(trajectory);
		//sleep(1000);
		
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
