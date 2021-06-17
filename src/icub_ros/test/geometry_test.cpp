
#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "geometry.hpp"

geometry_msgs::Point _point(double x, double y, double z)
{
  geometry_msgs::Point base;
  base.x = x;
  base.y = y;
  base.z = z;
  return base;
}

geometry_msgs::Point base_point()
{
  return _point(0, 0, 0);
}

TEST(TestClosestPoint, PointAligned0)
{
  geometry_msgs::Point base = base_point();
  geometry_msgs::Point expected = base_point();

  geometry_msgs::Point target = _point(1, 0, 0);
  geometry_msgs::Quaternion rotation;
  rotation.w = 1;

  EXPECT_EQ(expected, closest_point(target, rotation, base));

  base.x = 10;
  expected.x = base.x;

  EXPECT_EQ(expected, closest_point(target, rotation, base));

  target.x = 10;
  target.y = 10;
  rotation.w = cos(M_PI / 4);
  rotation.z = sin(M_PI / 4);

  EXPECT_NEAR(expected.x, closest_point(target, rotation, base).x, 0.0000001);
  EXPECT_NEAR(expected.y, closest_point(target, rotation, base).y, 0.0000001);
  EXPECT_NEAR(expected.z, closest_point(target, rotation, base).z, 0.0000001);

  target.x = 15;
  target.y = 5;
  rotation.w = cos(M_PI / 8);
  rotation.z = sin(M_PI / 8);

  EXPECT_NEAR(expected.x, closest_point(target, rotation, base).x, 0.0000001);
  EXPECT_NEAR(expected.y, closest_point(target, rotation, base).y, 0.0000001);
  EXPECT_NEAR(expected.z, closest_point(target, rotation, base).z, 0.0000001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}