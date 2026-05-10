#include "motion_core/trajectory_validator.hpp"

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

} // namespace motion_core
