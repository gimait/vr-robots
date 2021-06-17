#include <geometry_msgs/Pose.h>

#include <Eigen/Geometry>

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
