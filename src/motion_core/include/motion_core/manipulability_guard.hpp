#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {

class ManipulabilityGuard {
public:
  ManipulabilityGuard() = default;

  ManipulabilityGuard(moveit::core::RobotModelConstPtr model,
                      std::string group_name, double floor,
                      std::size_t sample_every_n_points);

  bool check_trajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                        std::string &reason) const;

  double
  compute_yoshikawa_index(const std::vector<double> &joint_positions) const;

  bool enabled() const { return model_ != nullptr; }

private:
  moveit::core::RobotModelConstPtr model_;
  std::string group_name_;
  double floor_ = 0.0;
  std::size_t sample_every_n_points_ = 1;
};

} // namespace motion_core
