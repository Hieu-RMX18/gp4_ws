#include <string>

#include <gtest/gtest.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "motion_core/wrist_flip_guard.hpp"

namespace motion_core
{
namespace
{
trajectory_msgs::msg::JointTrajectory make_two_point_trajectory(double delta_joint_1)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = { "joint_1_s", "joint_2_l" };

  trajectory_msgs::msg::JointTrajectoryPoint point0;
  point0.positions = { 0.0, 0.0 };

  trajectory_msgs::msg::JointTrajectoryPoint point1;
  point1.positions = { delta_joint_1, 0.1 };

  traj.points.push_back(point0);
  traj.points.push_back(point1);
  return traj;
}

TEST(WristFlipGuardTest, PassesWhenConsecutiveDeltasAreWithinLimit)
{
  const WristFlipGuard guard;
  const auto traj = make_two_point_trajectory(0.4);

  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(WristFlipGuardTest, RejectsWhenAnyJointDeltaExceedsThirtyDegrees)
{
  const WristFlipGuard guard;
  const auto traj = make_two_point_trajectory(0.7);

  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_FALSE(reason.empty());
}
}  // namespace
}  // namespace motion_core
