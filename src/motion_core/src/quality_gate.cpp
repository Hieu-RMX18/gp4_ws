#include "motion_core/quality_gate.hpp"
#include "motion_core/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {

double QualityGate::minimum_cartesian_fraction_for_primitive(
    const std::string &primitive) {
  if (primitive == "CARTESIAN_PATH") {
    return kMinimumFractionCartesianPath;
  }
  if (primitive == "LIN") {
    return kMinimumFractionLin;
  }
  if (primitive == "CIRC") {
    return kMinimumFractionCIRC;
  }
  return kMinimumCartesianFraction;
}

QualityGate::QualityGate(std::size_t max_trajectory_points,
                         double minimum_cartesian_fraction,
                         JointPositionGuard joint_position_guard,
                         ManipulabilityGuard manipulability_guard)
    : max_trajectory_points_(max_trajectory_points),
      minimum_cartesian_fraction_(minimum_cartesian_fraction),
      joint_position_guard_(std::move(joint_position_guard)),
      manipulability_guard_(std::move(manipulability_guard)) {}

bool QualityGate::validate_plan(
    const trajectory_msgs::msg::JointTrajectory &traj, double fraction,
    const std::string &primitive, std::string &reason,
    JointPositionGuard::Mode mode) const {
  reason.clear();

  if (!validate_trajectory_structure(traj, reason)) {
    return false;
  }

  if (!wrist_flip_guard_.check_trajectory(traj, reason)) {
    return false;
  }

  if (!joint_position_guard_.check_trajectory(traj, reason, mode)) {
    return false;
  }

  if (!manipulability_guard_.check_trajectory(traj, reason)) {
    return false;
  }

  if (traj.points.size() > max_trajectory_points_) {
    reason = "trajectory exceeds point limit";
    return false;
  }

  // Safe convention: fraction < 0.0 means non-Cartesian, so fraction check is
  // skipped.
  if (fraction >= 0.0) {
    if (!std::isfinite(fraction)) {
      reason = "cartesian fraction is non-finite";
      return false;
    }

    if (fraction > 1.0) {
      reason = "cartesian fraction is above 1.0";
      return false;
    }

    const double min_fraction =
        minimum_cartesian_fraction_for_primitive(primitive);
    if (fraction < min_fraction) {
      reason = "cartesian fraction below minimum threshold for primitive";
      return false;
    }
  }

  reason.clear();
  return true;
}
} // namespace motion_core
