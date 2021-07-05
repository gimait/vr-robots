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
  ros::V_string getLinksForJoints(ros::V_string joints);
  geometry_msgs::Pose calculateTargetPosition(geometry_msgs::Pose target, geometry_msgs::Pose origin);

protected:
  std::string m_move_group;
  ros::ServiceServer m_plan_trajectory_service;
  moveit::planning_interface::MoveGroupInterfacePtr m_move_group_interface;

  ros::V_string convertBetweenLists(ros::V_string s_list, ros::V_string from_list, ros::V_string to_list);
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
  return this->convertBetweenLists(links, m_move_group_interface->getRobotModel()->getLinkModelNames(),
                                   m_move_group_interface->getRobotModel()->getJointModelNames());
}

ros::V_string TrajectoryPlanner::getLinksForJoints(ros::V_string joints)
{
  return this->convertBetweenLists(joints, m_move_group_interface->getRobotModel()->getJointModelNames(),
                                   m_move_group_interface->getRobotModel()->getLinkModelNames());
}

ros::V_string TrajectoryPlanner::convertBetweenLists(ros::V_string s_list, ros::V_string from_list,
                                                     ros::V_string to_list)
{
  ros::V_string output = ros::V_string();

  for (std::string s : s_list)
  {
    size_t joint_pos = distance(from_list.begin(), find(from_list.begin(), from_list.end(), s));
    if (joint_pos < to_list.size())
    {
      output.push_back(to_list[joint_pos]);
    }
  }
  return output;
}

geometry_msgs::Pose TrajectoryPlanner::calculateTargetPosition(geometry_msgs::Pose target, geometry_msgs::Pose origin)
{
  const robot_state::JointModelGroup *joint_model_group =
      this->m_move_group_interface->getRobotModel()->getJointModelGroup(m_move_group);

  // Return if the target is already reachable.
  if (this->m_move_group_interface->getCurrentState()->setFromIK(joint_model_group, target))
  {
    return target;
  }

  // Generate a path by creating a set of angles that move incrementally towards the goal (in a line) from the current
  // pose.

  std::vector<geometry_msgs::Pose> possible = calculateLinearPath(target, origin, 100);

  // Check backwards all calculated poses until we find a reachable one.
  for (auto pose = possible.rbegin(); pose != possible.rend(); ++pose)
  {
    if (this->m_move_group_interface->getCurrentState()->setFromIK(joint_model_group, *pose))
    {
      return *pose;
    }
  }

  // At least the current should be reachable.
  return origin;
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

  geometry_msgs::Pose target = this->calculateTargetPosition(req.target_pose, current_pose);

  std::vector<double> ik_seed_state = m_move_group_interface->getCurrentJointValues();
  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  kinematics::KinematicsQueryOptions options = kinematics::KinematicsQueryOptions();

  // Print names of used joints.
  std::copy(js.name.begin(), js.name.end(), std::ostream_iterator<std::string>(std::cout, ", "));

  m_move_group_interface->setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan new_plan;
  bool success = (m_move_group_interface->plan(new_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  new_plan.trajectory_.joint_trajectory.joint_names =
      this->getLinksForJoints(new_plan.trajectory_.joint_trajectory.joint_names);

  res.trajectories.push_back(new_plan.trajectory_);

  return true;
}
