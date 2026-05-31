#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {

struct ManipulabilitySample {
  std::size_t index = 0;
  double value = 0.0;
};

double normalize_yoshikawa_index(double raw_index, double reference_length_m);

bool check_manipulability_samples(
    double floor, const std::vector<ManipulabilitySample> &samples,
    std::string &reason);

class ManipulabilityGuard {
public:
  static ManipulabilityGuard disabled();

  ManipulabilityGuard(moveit::core::RobotModelConstPtr model,
                      std::string group_name, double floor,
                      std::size_t sample_every_n_points,
                      double reference_length_m = 1.0);

  bool check_trajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                        std::string &reason) const;

  double
  compute_yoshikawa_index(const std::vector<double> &joint_positions) const;

  bool enabled() const { return enabled_; }

private:
  explicit ManipulabilityGuard(bool enabled);

  bool enabled_ = false;
  moveit::core::RobotModelConstPtr model_;
  std::string group_name_;
  double floor_ = 0.0;
  std::size_t sample_every_n_points_ = 1;
  double reference_length_m_ = 1.0;
};

} // namespace motion_core
