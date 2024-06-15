#include "moveit_controller.h"

using std::placeholders::_1;
using moveit::planning_interface::MoveGroupInterface;

void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj, moveit::planning_interface::MoveGroupInterfacePtr mgi)
{
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Starting thread");
  mgi->asyncExecute(traj);
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Ending thread");
}

void executePlan(moveit::planning_interface::MoveGroupInterface::Plan &rotationPlan, moveit::planning_interface::MoveGroupInterfacePtr mgi)
{
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Starting thread");
  mgi->asyncExecute(rotationPlan);
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Ending thread");
}

bool isEmpty(const geometry_msgs::msg::Pose &p)
{
  return p.position.x == 0 && p.position.y == 0 && p.position.z == 0 && p.orientation.x == 0 && p.orientation.y == 0 && p.orientation.z == 0 && p.orientation.w == 0;
}


TestNode::TestNode(const rclcpp::NodeOptions &options)
: Node("hello_moveit", options), node_ptr(std::make_shared<rclcpp::Node>("example_moveit")), executor_ptr(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
  node_name = ((std::string) this->get_namespace()).erase(0, 1);//"hello_moveit";
  //node_name.pop_back();
  
  RCLCPP_INFO(this->get_logger(), node_name.c_str());
  
  subscription_ = this->create_subscription<interfaces::msg::ArmCmd>(
  "arm_base_commands", 10, std::bind(&TestNode::topic_callback, this, std::placeholders::_1));
  
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


void TestNode::topic_callback(const interfaces::msg::ArmCmd & armMsg)
{
  geometry_msgs::msg::Pose poseMsg = armMsg.pose;
  double stepSize = armMsg.speed;
  //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  RCLCPP_INFO(this->get_logger(), "I heard: %f %f %f %f %f %f %f",
		poseMsg.position.x,
		poseMsg.position.y,
		poseMsg.position.z,
		poseMsg.orientation.x,
		poseMsg.orientation.y,
		poseMsg.orientation.z,
		poseMsg.orientation.w);//*/
  
  
  move_group_ptr->stop();
  if (th.joinable())
	{
		th.join();
	}
	if ((isEmpty(poseMsg) && !armMsg.reset) || armMsg.estop)
	{
	  RCLCPP_INFO(this->get_logger(), "Stopping (for some reason)");
	  return;
	}
  
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
		msg.position.x += poseMsg.position.x*stepSize;
		msg.position.y += poseMsg.position.y*stepSize;
		msg.position.z += poseMsg.position.z*stepSize;
		
		
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
	if (armMsg.reset == true) //reset something
	{
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
	else if (poseMsg.orientation.x != 0 || poseMsg.orientation.y != 0 || poseMsg.orientation.z != 0 || poseMsg.orientation.w != 0) //rotation required
	{
	  tf2::Quaternion q1;
	  tf2::convert(current_pose.orientation, q1);
	  tf2::Quaternion q2;
	  q2.setRPY(poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z);
	  tf2::Quaternion q3 = q1 * q2;
	  geometry_msgs::msg::Quaternion q4 = tf2::toMsg(q3);
	  
	  geometry_msgs::msg::Pose rotation_pose = current_pose;
	  rotation_pose.orientation = q4;
	  
	  points.push_back(rotation_pose);
	  
	  const double jump_threshold = 0;
		const double eef_step = 0.01;
		//double fraction = move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
		move_group_ptr->computeCartesianPath(points, eef_step, jump_threshold, trajectory);
		//RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
		//move_group_ptr->execute(trajectory);
		
		//launch thread
		RCLCPP_INFO(this->get_logger(), "What");
		if (th.joinable())
		{
			th.join();
		}
		RCLCPP_INFO(this->get_logger(), "What??");
		th = std::thread(executeTrajectory, std::ref(trajectory), move_group_ptr);
		RCLCPP_INFO(this->get_logger(), "Why");
	  
	  //geometry_msgs::msg::Pose rotation_pose = current_pose;
	  //rotation_pose.orientation = q4;
	  
	  /*move_group_ptr->setOrientationTarget(q4.x, q4.y, q4.z, q4.w);
	  auto const ok = static_cast<bool>(move_group_ptr->plan(rotationPlan));
	  
	  if (ok)
	  {
	    RCLCPP_INFO(this->get_logger(), "Rotation");
			if (th.joinable())
			{
				th.join();
			}
			RCLCPP_INFO(this->get_logger(), "Rotation??");
			th = std::thread(executePlan, std::ref(rotationPlan), move_group_ptr);
			RCLCPP_INFO(this->get_logger(), "Rotation done");
	  }
	  else
	  {
	    RCLCPP_ERROR(this->get_logger(), "Planing failed!");
	  }*/
	  
	}
	else //rotation not required
	{
	  points.push_back(new_pose);
	  const double jump_threshold = 0;
		const double eef_step = 0.01;
		//double fraction = move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
		move_group_ptr->computeCartesianPath(points, eef_step, jump_threshold, trajectory);
		//RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
		//move_group_ptr->execute(trajectory);
		
		//launch thread
		RCLCPP_INFO(this->get_logger(), "What");
		RCLCPP_INFO(this->get_logger(), "What??");
		th = std::thread(executeTrajectory, std::ref(trajectory), move_group_ptr);
		RCLCPP_INFO(this->get_logger(), "Why");
	}
  
  
	
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  rclcpp::spin(node);
}

