#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "geometry.hpp"
#include "icub_ros/MoveService.h"

class TrajectoryPlanner
{
public:
  TrajectoryPlanner(std::string move_group);

  bool planTrajectoryService(icub_ros::MoveService::Request &req, icub_ros::MoveService::Response &res);
  void startService();
  ros::V_string getJointsForLinks(ros::V_string links);
  geometry_msgs::Pose calculateTargetPosition(geometry_msgs::Pose original);

protected:
  std::string m_move_group;
  ros::ServiceServer m_plan_trajectory_service;
  moveit::planning_interface::MoveGroupInterfacePtr m_move_group_interface;
};

TrajectoryPlanner::TrajectoryPlanner(std::string move_group)
{
  m_move_group = move_group;
  m_move_group_interface =
      moveit::planning_interface::MoveGroupInterfacePtr(new moveit::planning_interface::MoveGroupInterface(move_group));
}

void TrajectoryPlanner::startService()
{
  ros::NodeHandle nh("~");
  m_plan_trajectory_service = nh.advertiseService("plan_trajectory", &TrajectoryPlanner::planTrajectoryService, this);
}

ros::V_string TrajectoryPlanner::getJointsForLinks(ros::V_string links)
{
  ros::V_string joint_list = m_move_group_interface->getRobotModel()->getJointModelNames();
  ros::V_string link_list = m_move_group_interface->getRobotModel()->getLinkModelNames();
  ros::V_string output = ros::V_string();

  for (std::string ln : links)
  {
    size_t link_pos = distance(link_list.begin(), find(link_list.begin(), link_list.end(), ln));
    if (link_pos < joint_list.size())
    {
      output.push_back(joint_list[link_pos]);
    }
  }
  return output;
}

geometry_msgs::Pose TrajectoryPlanner::calculateTargetPosition(geometry_msgs::Pose origin)
{
  const robot_state::JointModelGroup *joint_model_group =
      this->m_move_group_interface->getRobotModel()->getJointModelGroup(m_move_group);

  // Return if the target is already reachable.
  if (this->m_move_group_interface->getCurrentState()->setFromIK(joint_model_group, origin))
  {
    return origin;
  }

  // Transform the target position to a closer point within the range of the robot.
  // Note: I am calculating the closest point between the end effector and the target pose.
  // We should reconsider wether this point or the base of the move_group should be used to find the target
  // position.
  geometry_msgs::Point closest_to_end =
      closest_point(origin.position, origin.orientation, this->m_move_group_interface->getCurrentPose().pose.position);

  geometry_msgs::Pose target;
  target.orientation = origin.orientation;
  target.position = closest_to_end;

  if (!this->m_move_group_interface->getCurrentState()->setFromIK(joint_model_group, target))
  {
    throw std::logic_error("Unable to find a valid target pose.");
  }
  else
  {
    // Iterate until we find the closest reachable point in the line.
  }
  return target;
}

bool TrajectoryPlanner::planTrajectoryService(icub_ros::MoveService::Request &req, icub_ros::MoveService::Response &res)
{
  sensor_msgs::JointState js;
  js.name = this->getJointsForLinks(req.link_names);
  js.position = req.joint_positions;
  moveit_msgs::RobotState rs;
  rs.joint_state = js;
  m_move_group_interface->setStartState(rs);

  geometry_msgs::Pose current_pose = this->m_move_group_interface->getCurrentPose().pose;

  geometry_msgs::Pose target = this->calculateTargetPosition(req.target_pose);

  std::vector<double> ik_seed_state = m_move_group_interface->getCurrentJointValues();
  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  kinematics::KinematicsQueryOptions options = kinematics::KinematicsQueryOptions();

  // Print names of used joints.
  std::copy(js.name.begin(), js.name.end(), std::ostream_iterator<std::string>(std::cout, ", "));

  m_move_group_interface->setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan new_plan;
  bool success = (m_move_group_interface->plan(new_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  res.trajectories.push_back(new_plan.trajectory_);

  return true;
}
