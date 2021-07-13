#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>

geometry_msgs::Point closest_point(geometry_msgs::Point tp, geometry_msgs::Quaternion tr, geometry_msgs::Point bp)
{
  Eigen::Quaterniond qr = Eigen::Quaterniond(tr.w, tr.x, tr.y, tr.z);
  auto vr = qr * Eigen::Vector3d(1, 0, 0);
  double a = vr.x() * (bp.x - tp.x) + vr.y() * (bp.y - tp.y) + vr.z() * (bp.z - tp.z);
  a /= vr.x() * vr.x() + vr.y() * vr.y() + vr.z() * vr.z();

  geometry_msgs::Point point;
  point.x = tp.x + a * vr.x();
  point.y = tp.y + a * vr.y();
  point.z = tp.z + a * vr.z();

  return point;
}

std::vector<geometry_msgs::Pose> calculateLinearPath(geometry_msgs::Pose target, geometry_msgs::Pose origin,
                                                     size_t steps)
{
  std::vector<geometry_msgs::Pose> linespace;
  geometry_msgs::Pose step_size;
  step_size.position.x = (target.position.x - origin.position.x) / steps;
  step_size.position.y = (target.position.y - origin.position.y) / steps;
  step_size.position.z = (target.position.z - origin.position.z) / steps;

  // Linespace without including target and current positions.
  for (int i = 1; i < steps; i++)
  {
    geometry_msgs::Pose step;
    step.position.x = step_size.position.x * i + origin.position.x;
    step.position.y = step_size.position.y * i + origin.position.y;
    step.position.z = step_size.position.z * i + origin.position.z;
    step.orientation = target.orientation;
    linespace.push_back(step);
  }

  return linespace;
}
