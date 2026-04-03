#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>

#include "motion_core/orientation_filter.hpp"

namespace motion_core
{
namespace
{
TEST(OrientationFilterTest, NormalizesNonUnitQuaternion)
{
  OrientationFilter filter;
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 2.0;

  std::string reason;
  ASSERT_TRUE(filter.normalize_and_validate(pose, reason));
  EXPECT_TRUE(reason.empty());

  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w);
  EXPECT_NEAR(norm, 1.0, 1e-12);
  EXPECT_NEAR(pose.orientation.w, 1.0, 1e-12);
}

TEST(OrientationFilterTest, RejectsZeroNormQuaternion)
{
  OrientationFilter filter;
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;

  std::string reason;
  EXPECT_FALSE(filter.normalize_and_validate(pose, reason));
  EXPECT_EQ(reason, "quaternion norm is zero");
}
}  // namespace
}  // namespace motion_core
