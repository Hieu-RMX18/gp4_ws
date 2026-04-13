#pragma once

#include <cstddef>
#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "motion_core/wrist_flip_guard.hpp"

namespace motion_core
{
class QualityGate
{
public:
  // Hard safety ceiling for MotoROS2 dispatch compatibility.
  static constexpr std::size_t kMaxTrajectoryPoints = 200;
  static constexpr double kMinimumCartesianFraction = 0.95;
  static constexpr double kFractionNotApplicable = -1.0;

  explicit QualityGate(
    std::size_t max_trajectory_points = kMaxTrajectoryPoints,
    double minimum_cartesian_fraction = kMinimumCartesianFraction);

  bool validate_plan(
    const trajectory_msgs::msg::JointTrajectory & traj,
    double fraction,
    std::string & reason) const;

private:
  std::size_t max_trajectory_points_;
  double minimum_cartesian_fraction_;
  WristFlipGuard wrist_flip_guard_;
};
}  // namespace motion_core
