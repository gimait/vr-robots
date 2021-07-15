#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>

#include "geometry.hpp"
#include "icub_ros/MoveService.h"

enum TargetType
{
  Pose,
  Position,
  Orientation
};

class TrajectoryPlanner
{
public:
  TrajectoryPlanner(std::string move_group, TargetType target = TargetType::Pose);

  bool planTrajectoryService(icub_ros::MoveService::Request &req, icub_ros::MoveService::Response &res);
  void startService();
  ros::V_string getJointsForLinks(ros::V_string links);
  ros::V_string getLinksForJoints(ros::V_string joints);
  geometry_msgs::Pose calculateTargetPosition(geometry_msgs::Pose target, Eigen::Isometry3d origin);
  geometry_msgs::Pose calculateTargetPosition(geometry_msgs::Pose target, geometry_msgs::Pose origin);

protected:
  TargetType m_target;
  std::string m_move_group;
  ros::ServiceServer m_plan_trajectory_service;
  moveit::planning_interface::MoveGroupInterfacePtr m_move_group_interface;
  moveit::core::RobotModelConstPtr m_kinematic_model;
  moveit::core::RobotStatePtr m_kinematic_state;
  ros::V_string convertBetweenLists(ros::V_string s_list, ros::V_string from_list, ros::V_string to_list);
};

TrajectoryPlanner::TrajectoryPlanner(std::string move_group, TargetType target)
{
  m_target = target;
  m_move_group = move_group;
  m_move_group_interface =
      moveit::planning_interface::MoveGroupInterfacePtr(new moveit::planning_interface::MoveGroupInterface(move_group));
  m_kinematic_model = m_move_group_interface->getRobotModel();
  m_kinematic_state = moveit::core::RobotStatePtr(new moveit::core::RobotState(m_kinematic_model));
}

void TrajectoryPlanner::startService()
{
  ros::NodeHandle nh("~/" + m_move_group);
  m_plan_trajectory_service = nh.advertiseService("plan_trajectory", &TrajectoryPlanner::planTrajectoryService, this);
}

ros::V_string TrajectoryPlanner::getJointsForLinks(ros::V_string links)
{
  return convertBetweenLists(links, m_move_group_interface->getRobotModel()->getLinkModelNames(),
                             m_move_group_interface->getRobotModel()->getJointModelNames());
}

ros::V_string TrajectoryPlanner::getLinksForJoints(ros::V_string joints)
{
  return convertBetweenLists(joints, m_move_group_interface->getRobotModel()->getJointModelNames(),
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

geometry_msgs::Pose TrajectoryPlanner::calculateTargetPosition(geometry_msgs::Pose target, Eigen::Isometry3d origin)
{
  return calculateTargetPosition(target, tf2::toMsg(origin));
}

// TODO: Atm, the target position calculated is quite limited, but it would be possible to upgrade it by combining
// the use of a Position and Orientation targets individually if the Pose is not reachable:
// If the pose is reachable, return it.
// If it's not, calculate the point closest to target that the hand can reach, plan to that position, then calculate
// closest orientation to target, add the new plan for it.
geometry_msgs::Pose TrajectoryPlanner::calculateTargetPosition(geometry_msgs::Pose target, geometry_msgs::Pose origin)
{
  const robot_state::JointModelGroup *joint_model_group =
      m_move_group_interface->getRobotModel()->getJointModelGroup(m_move_group);

  // Return if the target is already reachable.
  if (m_move_group_interface->getCurrentState()->setFromIK(joint_model_group, target))
  {
    return target;
  }

  // Generate a path by creating a set of angles that move incrementally towards the goal (in a line) from the current
  // pose.
  std::vector<geometry_msgs::Pose> possible = calculateLinearPath(target, origin, 100);

  // Check backwards all calculated poses until we find a reachable one.
  for (auto pose = possible.rbegin(); pose != possible.rend(); ++pose)
  {
    if (m_move_group_interface->getCurrentState()->setFromIK(joint_model_group, *pose))
    {
      return *pose;
    }
  }

  // At least the current should be reachable.
  return origin;
}

bool TrajectoryPlanner::planTrajectoryService(icub_ros::MoveService::Request &req, icub_ros::MoveService::Response &res)
{
  moveit_msgs::RobotState rs;
  rs.joint_state.name = getJointsForLinks(req.link_names);
  rs.joint_state.position = req.joint_positions;

  const moveit::core::JointModelGroup *joint_model_group = m_kinematic_model->getJointModelGroup(m_move_group);
  m_kinematic_state->setJointGroupPositions(joint_model_group, req.joint_positions);
  m_move_group_interface->setStartState(rs);

  geometry_msgs::Pose target = calculateTargetPosition(
      req.target_pose, m_kinematic_state->getGlobalLinkTransform(m_move_group_interface->getEndEffectorLink()));

  moveit_msgs::MoveItErrorCodes error_code;
  kinematics::KinematicsQueryOptions options = kinematics::KinematicsQueryOptions();

  m_move_group_interface->setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan new_plan;

  if (m_move_group_interface->plan(new_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    new_plan.trajectory_.joint_trajectory.joint_names =
        getLinksForJoints(new_plan.trajectory_.joint_trajectory.joint_names);
    res.trajectories.push_back(new_plan.trajectory_);

    return true;
  }
  else
  {
    return false;
  }
}
