#pragma once

#include <cstddef>
#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "motion_core/joint_position_guard.hpp"
#include "motion_core/manipulability_guard.hpp"
#include "motion_core/wrist_flip_guard.hpp"

namespace motion_core {
class QualityGate {
public:
  // Hard safety ceiling for MotoROS2 dispatch compatibility.
  static constexpr std::size_t kMaxTrajectoryPoints = 200;
  // Generic Cartesian fraction threshold (used when primitive-specific value
  // not set).
  static constexpr double kMinimumCartesianFraction = 0.90;
  // Primitive-specific Cartesian fraction thresholds.
  // LIN/CIRC: 0.90 — slightly relaxed to reduce false rejects on ordinary arcs.
  // CARTESIAN_PATH: 0.95 — keep stricter because multi-waypoint fidelity
  // matters.
  static constexpr double kMinimumFractionCartesianPath = 0.95;
  static constexpr double kMinimumFractionLin = 0.90;
  static constexpr double kMinimumFractionCIRC = 0.90;
  static constexpr double kFractionNotApplicable = -1.0;

  /// Returns the minimum acceptable Cartesian fraction for the given primitive.
  /// Falls back to kMinimumCartesianFraction for non-Cartesian primitives.
  static double
  minimum_cartesian_fraction_for_primitive(const std::string &primitive);

  explicit QualityGate(
      std::size_t max_trajectory_points = kMaxTrajectoryPoints,
      double minimum_cartesian_fraction = kMinimumCartesianFraction,
      JointPositionGuard joint_position_guard = JointPositionGuard{},
      ManipulabilityGuard manipulability_guard = ManipulabilityGuard{});

  bool validate_plan(
      const trajectory_msgs::msg::JointTrajectory &traj, double fraction,
      const std::string &primitive, std::string &reason,
      JointPositionGuard::Mode mode = JointPositionGuard::Mode::Default) const;

private:
  std::size_t max_trajectory_points_;
  double minimum_cartesian_fraction_;
  WristFlipGuard wrist_flip_guard_;
  JointPositionGuard joint_position_guard_;
  ManipulabilityGuard manipulability_guard_;
};
} // namespace motion_core
