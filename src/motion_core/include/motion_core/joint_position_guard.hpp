#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {

struct JointLimit {
  double min;
  double max;
};

struct TieredLimit {
  JointLimit default_limit;
  std::optional<JointLimit> extended_limit;
};

class JointPositionGuard {
public:
  enum class Mode { Default, Extended };

  static constexpr double kDefaultToleranceRad = 0.005;

  JointPositionGuard() = default;
  explicit JointPositionGuard(
      std::unordered_map<std::string, JointLimit> limits,
      double tolerance_rad = kDefaultToleranceRad);
  explicit JointPositionGuard(
      std::unordered_map<std::string, TieredLimit> tiered_limits,
      double tolerance_rad = kDefaultToleranceRad);

  bool check_trajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                        std::string &reason, Mode mode = Mode::Default) const;

  bool has_limit(const std::string &joint_name) const;
  JointLimit get_limit(const std::string &joint_name) const;
  double tolerance_rad() const { return tolerance_rad_; }

private:
  std::unordered_map<std::string, TieredLimit> limits_;
  double tolerance_rad_{kDefaultToleranceRad};
};

} // namespace motion_core
