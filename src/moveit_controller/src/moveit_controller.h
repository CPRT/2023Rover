#ifndef MOVEIT_CONTROLLER_H
#define MOVEIT_CONTROLLER_H

#include <memory>
#include <unistd.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/msg/arm_cmd.hpp"

void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj, moveit::planning_interface::MoveGroupInterfacePtr mgi);

void executePlan(moveit::planning_interface::MoveGroupInterface::Plan &rotationPlan, moveit::planning_interface::MoveGroupInterfacePtr mgi);

bool isEmpty(const geometry_msgs::msg::Pose &p);

class TestNode : public rclcpp::Node
{
  public:
    TestNode(const rclcpp::NodeOptions &options);

  private:
    rclcpp::Subscription<interfaces::msg::ArmCmd>::SharedPtr subscription_;
    
    std::string node_name;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;
    rclcpp::Node::SharedPtr node_ptr;
    rclcpp::Executor::SharedPtr executor_ptr;
    std::thread executor_thread;
    std::thread th;
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    moveit::planning_interface::MoveGroupInterface::Plan rotationPlan;
    
    void topic_callback(const interfaces::msg::ArmCmd & armMsg);
    
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr publisher_;
};

#endif
