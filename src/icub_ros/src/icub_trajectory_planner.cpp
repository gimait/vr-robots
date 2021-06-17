#include <ros/ros.h>

#include "trajectory_planner.hpp"

static const std::string HEAD_GROUP = "Head";
static const std::string LEFT_ARM_GROUP = "LeftArm";
static const std::string LEFT_EYE_GROUP = "LeftEye";
static const std::string LEFT_LEG_GROUP = "LeftLeg";
static const std::string RIGHT_ARM_GROUP = "RightArm";
static const std::string RIGHT_EYE_GROUP = "RightEye";
static const std::string RIGHT_LEG_GROUP = "RightLeg";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "icub_trajectory_planner");

  TrajectoryPlanner planner(RIGHT_ARM_GROUP);

  ROS_INFO("Trajectory planner is ready.");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}