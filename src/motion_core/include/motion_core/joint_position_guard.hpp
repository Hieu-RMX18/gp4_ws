#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {

struct JointLimit {
  double min;
  double max;
};

class JointPositionGuard {
public:
  JointPositionGuard() = default;
  explicit JointPositionGuard(
      std::unordered_map<std::string, JointLimit> limits);

  bool check_trajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                        std::string &reason) const;

  bool has_limit(const std::string &joint_name) const;
  JointLimit get_limit(const std::string &joint_name) const;

private:
  std::unordered_map<std::string, JointLimit> limits_;
};

} // namespace motion_core
