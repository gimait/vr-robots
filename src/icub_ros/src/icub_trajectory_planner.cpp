
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

#include "icub_ros/MoveService.h"

static const std::string HEAD_GROUP = "Head";
static const std::string LEFT_ARM_GROUP = "LeftArm";
static const std::string LEFT_EYE_GROUP = "LeftEye";
static const std::string LEFT_LEG_GROUP = "LeftLeg";
static const std::string RIGHT_ARM_GROUP = "RightArm";
static const std::string RIGHT_EYE_GROUP = "RightEye";
static const std::string RIGHT_LEG_GROUP = "RightLeg";

geometry_msgs::Point closest_point(geometry_msgs::Point tp, geometry_msgs::Quaternion tr, geometry_msgs::Point bp)
{
  Eigen::Quaternionf qr = Eigen::Quaternionf(tr.w, tr.x, tr.y, tr.z);
  auto ea = qr.toRotationMatrix().eulerAngles(0, 1, 2);
  // Direction of z axis after rotation.
  auto vr = Eigen::Vector3f(sin(ea.y()) * sin(ea.z()), cos(ea.y()) * sin(ea.z()), cos(ea.z()));
  double a = vr.x() * (bp.x - tp.x) + vr.y() * (bp.y - tp.y) + vr.z() * (bp.z - tp.z);
  a /= vr.x() * vr.x() + vr.y() * vr.y() + vr.z() * vr.z();

  geometry_msgs::Point point;
  point.x = tp.x + a * vr.x();
  point.y = tp.y + a * vr.y();
  point.z = tp.z + a * vr.z();

  return point;
}

class TrajectoryPlanner
{
public:
  TrajectoryPlanner();
  bool planTrajectoryService(icub_ros::MoveService::Request &req, icub_ros::MoveService::Response &res);
  ros::V_string getJointsForLinks(ros::V_string links);
  geometry_msgs::Pose calculateTargetPosition(geometry_msgs::Pose original);

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

ros::V_string TrajectoryPlanner::getJointsForLinks(ros::V_string links)
{
  ros::V_string joint_list = move_group_interface->getRobotModel()->getJointModelNames();
  ros::V_string link_list = move_group_interface->getRobotModel()->getLinkModelNames();
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
      this->move_group_interface->getRobotModel()->getJointModelGroup(RIGHT_ARM_GROUP);

  // Return if the target is already reachable.
  if (this->move_group_interface->getCurrentState()->setFromIK(joint_model_group, origin))
  {
    return origin;
  }

  // Transform the target position to a closer point within the range of the robot.
  // Note: I am calculating the closest point between the end effector and the target pose.
  // We should reconsider wether this point or the base of the move_group should be used to find the target
  // position.
  geometry_msgs::Point closest_to_end =
      closest_point(origin.position, origin.orientation, this->move_group_interface->getCurrentPose().pose.position);

  geometry_msgs::Pose target;
  target.orientation = origin.orientation;
  target.position = closest_to_end;

  if (!this->move_group_interface->getCurrentState()->setFromIK(joint_model_group, target))
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

  geometry_msgs::Pose target;
  target.position.x = req.target_object.x;
  target.position.y = req.target_object.y;
  target.position.z = req.target_object.z;

  target = this->calculateTargetPosition(target);

  std::vector<double> ik_seed_state = move_group_interface->getCurrentJointValues();
  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  kinematics::KinematicsQueryOptions options = kinematics::KinematicsQueryOptions();

  // Print names of used joints.
  std::copy(js.name.begin(), js.name.end(), std::ostream_iterator<std::string>(std::cout, ", "));

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
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}