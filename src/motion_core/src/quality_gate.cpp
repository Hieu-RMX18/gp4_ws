#include "motion_core/quality_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {
namespace {
bool is_finite_vector(const std::vector<double> &values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}
} // namespace

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
    const std::string &primitive, std::string &reason) const {
  reason.clear();

  if (traj.points.empty()) {
    reason = "trajectory has no points";
    return false;
  }
  if (traj.joint_names.empty()) {
    reason = "trajectory has no joint names";
    return false;
  }

  if (!wrist_flip_guard_.check_trajectory(traj, reason)) {
    return false;
  }

  if (!joint_position_guard_.check_trajectory(traj, reason)) {
    return false;
  }

  if (!manipulability_guard_.check_trajectory(traj, reason)) {
    return false;
  }

  const std::size_t dof = traj.joint_names.size();
  int64_t previous_time_ns = -1;
  for (std::size_t index = 0; index < traj.points.size(); ++index) {
    const auto &point = traj.points[index];
    const std::string point_label =
        "trajectory point[" + std::to_string(index) + "]";
    if (point.positions.size() != dof) {
      reason = point_label + " positions size does not match joint count";
      return false;
    }
    if (!point.velocities.empty() && point.velocities.size() != dof) {
      reason = point_label + " velocities size does not match joint count";
      return false;
    }
    if (!point.accelerations.empty() && point.accelerations.size() != dof) {
      reason = point_label + " accelerations size does not match joint count";
      return false;
    }
    if (!point.effort.empty() && point.effort.size() != dof) {
      reason = point_label + " effort size does not match joint count";
      return false;
    }
    if (!is_finite_vector(point.positions) ||
        (!point.velocities.empty() && !is_finite_vector(point.velocities)) ||
        (!point.accelerations.empty() &&
         !is_finite_vector(point.accelerations)) ||
        (!point.effort.empty() && !is_finite_vector(point.effort))) {
      reason = point_label + " contains NaN or Inf";
      return false;
    }

    const int64_t current_time_ns =
        static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
        static_cast<int64_t>(point.time_from_start.nanosec);
    if (current_time_ns < 0) {
      reason = point_label + " has negative time_from_start";
      return false;
    }
    if (previous_time_ns >= 0 && current_time_ns <= previous_time_ns) {
      reason = "trajectory time_from_start must be strictly monotonic";
      return false;
    }
    previous_time_ns = current_time_ns;
  }

  if (previous_time_ns <= 0) {
    reason = "trajectory total duration must be greater than zero";
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
