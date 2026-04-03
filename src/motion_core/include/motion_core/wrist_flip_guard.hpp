#pragma once

#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core
{
class WristFlipGuard
{
public:
  static constexpr double kMaxJointDeltaRad = 0.5235987755982988;

  bool check_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::string & reason) const;
};
}  // namespace motion_core
