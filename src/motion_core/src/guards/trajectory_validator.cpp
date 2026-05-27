#include "motion_core/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace motion_core {

bool is_finite_vector(const std::vector<double> &values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool validate_trajectory_structure(
    const trajectory_msgs::msg::JointTrajectory &traj, std::string &reason) {
  if (traj.points.empty()) {
    reason = "trajectory has no points";
    return false;
  }
  if (traj.points.size() < 2U) {
    reason = "trajectory must contain at least two points";
    return false;
  }
  if (traj.joint_names.empty()) {
    reason = "trajectory has no joint names";
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
      reason = point_label + " contains non-finite value (NaN or Inf)";
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

  return true;
}

bool is_single_point_noop_trajectory(
    const trajectory_msgs::msg::JointTrajectory &traj,
    const std::vector<double> &current_joint_positions, double tolerance_rad,
    std::string &reason) {
  reason.clear();
  if (traj.points.size() != 1U) {
    reason = "trajectory is not single-point";
    return false;
  }
  if (traj.joint_names.empty()) {
    reason = "single-point trajectory has no joint names";
    return false;
  }

  const auto &positions = traj.points.front().positions;
  if (positions.size() != current_joint_positions.size()) {
    reason = "single-point trajectory joint count differs from current joint state";
    return false;
  }
  if (positions.size() != traj.joint_names.size()) {
    reason = "single-point trajectory positions size does not match joint count";
    return false;
  }
  if (!is_finite_vector(positions) || !is_finite_vector(current_joint_positions)) {
    reason = "single-point trajectory contains non-finite value";
    return false;
  }

  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (std::abs(positions[index] - current_joint_positions[index]) >
        tolerance_rad) {
      reason = "single-point trajectory does not match current joint state";
      return false;
    }
  }

  reason.clear();
  return true;
}

} // namespace motion_core
