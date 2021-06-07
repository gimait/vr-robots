
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "icub_ros/MoveService.h"

static const std::string HEAD_GROUP = "Head";
static const std::string LEFT_ARM_GROUP = "LeftArm";
static const std::string LEFT_EYE_GROUP = "LeftEye";
static const std::string LEFT_LEG_GROUP = "LeftLeg";
static const std::string RIGHT_ARM_GROUP = "RightArm";
static const std::string RIGHT_EYE_GROUP = "RightEye";
static const std::string RIGHT_LEG_GROUP = "RightLeg";

class TrajectoryPlanner
{
public:
  TrajectoryPlanner();
  bool planTrajectoryService(icub_ros::MoveService::Request &req, icub_ros::MoveService::Response &res);

private:
  ros::ServiceServer plan_trajectory_service;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface;
};

TrajectoryPlanner::TrajectoryPlanner()
{
  ros::NodeHandle nh("~");
  plan_trajectory_service = nh.advertiseService("plan_trajectory", &TrajectoryPlanner::planTrajectoryService, this);
  move_group_interface = moveit::planning_interface::MoveGroupInterfacePtr(
      new moveit::planning_interface::MoveGroupInterface(RIGHT_ARM_GROUP));
}

bool TrajectoryPlanner::planTrajectoryService(icub_ros::MoveService::Request &req, icub_ros::MoveService::Response &res)
{
  sensor_msgs::JointState js;
  js.name = req.joint_names;
  js.position = req.joint_positions;
  moveit_msgs::RobotState rs;
  rs.joint_state = js;

  geometry_msgs::Pose target;
  target.position.x = req.target_object.x;
  target.position.y = req.target_object.y;
  target.position.z = req.target_object.z;

  move_group_interface->setStartState(rs);
  move_group_interface->setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan new_plan;
  bool success = (move_group_interface->plan(new_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  res.trajectories.push_back(new_plan.trajectory_);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "icub_trajectory_planner");

  TrajectoryPlanner planner;

  ROS_INFO("Trajectory planner is ready.");
  ros::spin();

  return 0;
}