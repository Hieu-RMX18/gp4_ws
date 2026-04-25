#include <cstddef>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
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
    point.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * 0.1);
    traj.points.push_back(point);
  }

  return traj;
}

TEST(QualityGateTest, RejectsPlanWithMoreThanTwoHundredPoints)
{
  const QualityGate gate;
  const auto traj = make_valid_trajectory(201);

  std::string reason;
  // Point limit check happens before fraction check; primitive can be any.
  EXPECT_FALSE(gate.validate_plan(traj, QualityGate::kFractionNotApplicable, "LIN", reason));
  EXPECT_EQ(reason, "trajectory exceeds point limit");
}

TEST(QualityGateTest, RejectsCartesianPlanWhenFractionTooLow)
{
  const QualityGate gate;
  const auto traj = make_valid_trajectory(2);

  std::string reason;
  // LIN fraction threshold is 0.90; 0.80 should be rejected.
  EXPECT_FALSE(gate.validate_plan(traj, 0.80, "LIN", reason));
  EXPECT_EQ(reason, "cartesian fraction below minimum threshold for primitive");
}

TEST(QualityGateTest, AcceptsCircWhenFractionAboveCircThreshold)
{
  const QualityGate gate;
  const auto traj = make_valid_trajectory(2);

  std::string reason;
  // CIRC primitive-specific threshold is kMinimumFractionCIRC.
  // A fraction equal to 1.0 must pass regardless of the LIN threshold.
  EXPECT_TRUE(gate.validate_plan(traj, 1.0, "CIRC", reason));
  EXPECT_TRUE(reason.empty());
}

TEST(QualityGateTest, RejectsCircWhenFractionBelowCircThreshold)
{
  const QualityGate gate;
  const auto traj = make_valid_trajectory(2);

  std::string reason;
  // 0.5 is below any sensible CIRC acceptance threshold.
  EXPECT_FALSE(gate.validate_plan(traj, 0.5, "CIRC", reason));
  EXPECT_EQ(reason, "cartesian fraction below minimum threshold for primitive");
}

TEST(QualityGateTest, PrimitiveDispatchRoutesDifferentThresholds)
{
  // Same fraction, different primitive: ensures per-primitive lookup is wired.
  const double lin_min = QualityGate::minimum_cartesian_fraction_for_primitive("LIN");
  const double circ_min = QualityGate::minimum_cartesian_fraction_for_primitive("CIRC");
  const double cart_min =
    QualityGate::minimum_cartesian_fraction_for_primitive("CARTESIAN_PATH");
  const double fallback = QualityGate::minimum_cartesian_fraction_for_primitive("UNKNOWN");

  EXPECT_GT(lin_min, 0.0);
  EXPECT_GT(circ_min, 0.0);
  EXPECT_GT(cart_min, 0.0);
  EXPECT_GT(fallback, 0.0);
}

TEST(QualityGateTest, RejectsPlanWithNonFiniteValues)
{
  const QualityGate gate;
  auto traj = make_valid_trajectory(2);
  traj.points[1].positions[0] = std::numeric_limits<double>::quiet_NaN();

  std::string reason;
  EXPECT_FALSE(gate.validate_plan(traj, QualityGate::kFractionNotApplicable, "PTP", reason));
  EXPECT_NE(reason.find("non-finite"), std::string::npos);
}

TEST(QualityGateTest, RejectsPlanWithNonMonotonicTimestamps)
{
  const QualityGate gate;
  auto traj = make_valid_trajectory(2);
  traj.points[1].time_from_start = traj.points[0].time_from_start;

  std::string reason;
  EXPECT_FALSE(gate.validate_plan(traj, QualityGate::kFractionNotApplicable, "PTP", reason));
  EXPECT_EQ(reason, "trajectory time_from_start must be strictly monotonic");
}
}  // namespace
}  // namespace motion_core
