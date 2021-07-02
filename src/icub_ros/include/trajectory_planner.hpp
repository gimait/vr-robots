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
  ros::V_string TrajectoryPlanner::getLinksForJoints(ros::V_string joints);
  geometry_msgs::Pose calculateTargetPosition(geometry_msgs::Pose target, geometry_msgs::Pose origin);
  std::vector<geometry_msgs::Pose> calculateLinearPath(geometry_msgs::Pose target, geometry_msgs::Pose origin,
                                                       size_t steps);

protected:
  std::string m_move_group;
  ros::ServiceServer m_plan_trajectory_service;
  moveit::planning_interface::MoveGroupInterfacePtr m_move_group_interface;

  ros::V_string TrajectoryPlanner::convertBetweenLists(ros::V_string s_list,
                                                      ros::V_string from_list,
                                                      ros::V_string to_list);
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
  return this->convertBetweenLists(links,
                                   m_move_group_interface->getRobotModel()->getLinkModelNames(),
                                   m_move_group_interface->getRobotModel()->getJointModelNames());
}

ros::V_string TrajectoryPlanner::getLinksForJoints(ros::V_string joints)
{
  return this->convertBetweenLists(joints,
                                  m_move_group_interface->getRobotModel()->getJointModelNames(),
                                  m_move_group_interface->getRobotModel()->getLinkModelNames());
}

ros::V_string TrajectoryPlanner::convertBetweenLists(ros::V_string s_list,
                                                     ros::V_string from_list,
                                                     ros::V_string to_list)
{
  ros::V_string output = ros::V_string();

  for (std::string s : joints)
  {
    size_t joint_pos = distance(from_list.begin(), find(from_list.begin(), from_list.end(), s));
    if (joint_pos < to_list.size())
    {
      output.push_back(to_list[joint_pos]);
    }
  }
  return output;
}

std::vector<geometry_msgs::Pose> TrajectoryPlanner::calculateLinearPath(geometry_msgs::Pose target,
                                                                        geometry_msgs::Pose origin, size_t steps)
{
  std::vector<geometry_msgs::Pose> linespace;
  geometry_msgs::Pose step_size;
  step_size.position.x = (target.position.x - origin.position.x) / steps;
  step_size.position.y = (target.position.y - origin.position.y) / steps;
  step_size.position.z = (target.position.z - origin.position.z) / steps;
  step_size.orientation.w = (target.orientation.w - origin.orientation.w) / steps;
  step_size.orientation.x = (target.orientation.x - origin.orientation.x) / steps;
  step_size.orientation.y = (target.orientation.y - origin.orientation.y) / steps;
  step_size.orientation.z = (target.orientation.z - origin.orientation.z) / steps;

  // Linespace without including target and current positions.
  for (int i = 1; i < steps; i++)
  {
    geometry_msgs::Pose step;
    step.position.x = step_size.position.x * i + origin.position.x;
    step.position.y = step_size.position.y * i + origin.position.y;
    step.position.z = step_size.position.z * i + origin.position.z;
    step.orientation.w = step_size.orientation.w * i + origin.orientation.w;
    step.orientation.x = step_size.orientation.x * i + origin.orientation.x;
    step.orientation.y = step_size.orientation.y * i + origin.orientation.y;
    step.orientation.z = step_size.orientation.z * i + origin.orientation.z;

    linespace.push_back(step);
  }

  return linespace;
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

  std::vector<geometry_msgs::Pose> possible = this->calculateLinearPath(target, origin, 100);

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

  res.trajectories.push_back(new_plan.trajectory_);

  return true;
}
