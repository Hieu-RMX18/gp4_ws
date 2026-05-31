#pragma once

#include <cstddef>
#include <string>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {
class TrajectoryPostProcessor {
public:
  // MotoROS2 hard safety ceiling.
  static constexpr std::size_t kMaxTrajectoryPoints = 200;
  static constexpr double kDefaultVelocityScaling = 0.06;
  static constexpr double kDefaultAccelerationScaling = 0.06;

  bool apply_totg(robot_trajectory::RobotTrajectory &traj, double vel_scale,
                  double acc_scale, std::string &reason) const;

  bool downsample_to_max_points(trajectory_msgs::msg::JointTrajectory &traj,
                                std::size_t max_points,
                                std::string &reason) const;

  bool apply_ruckig_smoothing(robot_trajectory::RobotTrajectory &traj,
                              double vel_scale, double acc_scale,
                              std::string &reason) const;
};
} // namespace motion_core
