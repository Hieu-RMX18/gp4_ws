#include "motion_core/joint_position_guard.hpp"

#include <iomanip>
#include <sstream>

namespace motion_core {

JointPositionGuard::JointPositionGuard(
    std::unordered_map<std::string, JointLimit> limits) {
  for (auto &[name, lim] : limits) {
    limits_[name] = TieredLimit{lim, std::nullopt};
  }
}

JointPositionGuard::JointPositionGuard(
    std::unordered_map<std::string, TieredLimit> tiered_limits)
    : limits_(std::move(tiered_limits)) {}

bool JointPositionGuard::check_trajectory(
    const trajectory_msgs::msg::JointTrajectory &traj, std::string &reason,
    Mode mode) const {
  reason.clear();

  if (limits_.empty()) {
    return true;
  }

  const char *mode_label = (mode == Mode::Extended) ? "extended" : "default";

  for (std::size_t i = 0; i < traj.points.size(); ++i) {
    const auto &point = traj.points[i];
    for (std::size_t j = 0; j < traj.joint_names.size(); ++j) {
      if (j >= point.positions.size()) {
        break;
      }
      const auto it = limits_.find(traj.joint_names[j]);
      if (it == limits_.end()) {
        continue;
      }
      const double value = point.positions[j];
      const auto &tiered = it->second;
      const JointLimit *active_limit = &tiered.default_limit;
      if (mode == Mode::Extended && tiered.extended_limit.has_value()) {
        active_limit = &tiered.extended_limit.value();
      }
      if (value < active_limit->min || value > active_limit->max) {
        std::ostringstream stream;
        stream << "joint_position_guard reject at point[" << i
               << "]: " << traj.joint_names[j] << " = " << std::fixed
               << std::setprecision(4) << value << " rad outside ["
               << std::fixed << std::setprecision(4) << active_limit->min
               << ", " << std::fixed << std::setprecision(4)
               << active_limit->max << "] (mode=" << mode_label << ")";
        reason = stream.str();
        return false;
      }
    }
  }

  return true;
}

bool JointPositionGuard::has_limit(const std::string &joint_name) const {
  return limits_.count(joint_name) > 0;
}

JointLimit JointPositionGuard::get_limit(const std::string &joint_name) const {
  return limits_.at(joint_name).default_limit;
}

} // namespace motion_core
