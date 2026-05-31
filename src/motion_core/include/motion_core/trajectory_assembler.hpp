// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {

class TrajectoryAssembler {
public:
  /// Default budget for real hardware; hard MotoROS2 upper limit is 200.
  static constexpr std::size_t kMaxTrajectoryPoints = 180;

  struct MergeResult {
    bool success = false;
    std::string error_message;
    trajectory_msgs::msg::JointTrajectory trajectory;
    std::size_t total_points = 0;
  };

  /// Merge a vector of segment trajectories into one continuous trajectory.
  /// Checks: joint_names consistency, duplicate point deduplication,
  /// monotonic timestamps, point count budget.
  static MergeResult merge(
      const std::vector<trajectory_msgs::msg::JointTrajectory> &segments);
};

} // namespace motion_core
