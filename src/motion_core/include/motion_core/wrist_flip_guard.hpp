#pragma once

#include <array>
#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core
{
class WristFlipGuard
{
public:
  // V4 F1: per-joint delta thresholds instead of one fixed value for all joints.
  // J1/J2/J3 (large joints): 25° = 0.436 rad
  // J4/J5 (wrist joints): 45° = 0.785 rad
  // J6 (wrist rotate): 30° = 0.524 rad (with unwrap for continuous rotation)
  static constexpr double kDeltaRadJ123 = 0.4363323129985824;  // 25°
  static constexpr double kDeltaRadJ45  = 0.7853981633974483;  // 45°
  static constexpr double kDeltaRadJ6   = 0.5235987755982988;  // 30°

  // Legacy API compatibility
  static constexpr double kMaxJointDeltaRad = kDeltaRadJ123;

  bool check_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::string & reason) const;

  /// V4 F3: Returns per-joint threshold for the given joint index (0-based, 6 joints).
  static double max_delta_for_joint(std::size_t joint_idx);

private:
  /// V4 F.2: detect repeated sign-flips on wrist joints (indices 3,4,5).
  bool check_wrist_sign_flips(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::string & reason) const;
};
}  // namespace motion_core
