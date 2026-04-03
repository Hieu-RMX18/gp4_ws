#include <cstddef>
#include <string>

#include <gtest/gtest.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "motion_core/quality_gate.hpp"

namespace motion_core
{
namespace
{
trajectory_msgs::msg::JointTrajectory make_valid_trajectory(std::size_t point_count)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = { "joint_1_s" };

  traj.points.reserve(point_count);
  for (std::size_t i = 0; i < point_count; ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = { static_cast<double>(i) * 0.001 };
    traj.points.push_back(point);
  }

  return traj;
}

TEST(QualityGateTest, RejectsPlanWithMoreThanTwoHundredPoints)
{
  const QualityGate gate;
  const auto traj = make_valid_trajectory(201);

  std::string reason;
  EXPECT_FALSE(gate.validate_plan(traj, QualityGate::kFractionNotApplicable, reason));
  EXPECT_EQ(reason, "trajectory exceeds point limit");
}

TEST(QualityGateTest, RejectsCartesianPlanWhenFractionTooLow)
{
  const QualityGate gate;
  const auto traj = make_valid_trajectory(2);

  std::string reason;
  EXPECT_FALSE(gate.validate_plan(traj, 0.80, reason));
  EXPECT_EQ(reason, "cartesian fraction below minimum threshold");
}
}  // namespace
}  // namespace motion_core
