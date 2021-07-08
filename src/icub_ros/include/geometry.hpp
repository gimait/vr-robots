#include <geometry_msgs/Pose.h>

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

// Linear interpolation between two orientations from
// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
geometry_msgs::Quaternion quaternion_interpolation(geometry_msgs::Quaternion start, geometry_msgs::Quaternion end,
                                                   float at)
{
  geometry_msgs::Quaternion result;

  // Calculate angle between them.
  double cosHalfTheta = start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z;
  // if start=end or start=-end then theta = 0 and we can return start
  if (abs(cosHalfTheta) >= 1.0)
  {
    result.w = start.w;
    result.x = start.x;
    result.y = start.y;
    result.z = start.z;
    return result;
  }
  // Calculate temporary values.
  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);
  // if theta = 180 degrees then result is not fully defined
  // we could rotate around any axis normal to start or end
  if (fabs(sinHalfTheta) < 0.001)
  {  // fabs is floating point absolute
    result.w = (start.w * 0.5 + end.w * 0.5);
    result.x = (start.x * 0.5 + end.x * 0.5);
    result.y = (start.y * 0.5 + end.y * 0.5);
    result.z = (start.z * 0.5 + end.z * 0.5);
    return result;
  }
  double ratioA = sin((1 - at) * halfTheta) / sinHalfTheta;
  double ratioB = sin(at * halfTheta) / sinHalfTheta;
  // calculate Quaternion.
  result.w = (start.w * ratioA + end.w * ratioB);
  result.x = (start.x * ratioA + end.x * ratioB);
  result.y = (start.y * ratioA + end.y * ratioB);
  result.z = (start.z * ratioA + end.z * ratioB);
  return result;
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
