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

  JointPositionGuard() = default;
  explicit JointPositionGuard(
      std::unordered_map<std::string, JointLimit> limits);
  explicit JointPositionGuard(
      std::unordered_map<std::string, TieredLimit> tiered_limits);

  bool check_trajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                        std::string &reason, Mode mode = Mode::Default) const;

  bool has_limit(const std::string &joint_name) const;
  JointLimit get_limit(const std::string &joint_name) const;

private:
  std::unordered_map<std::string, TieredLimit> limits_;
};

} // namespace motion_core
