// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "motion_core/trajectory_assembler.hpp"

#include <cstdint>
#include <limits>

namespace motion_core {

TrajectoryAssembler::MergeResult TrajectoryAssembler::merge(
    const std::vector<trajectory_msgs::msg::JointTrajectory> &segments) {
  MergeResult result;

  if (segments.empty()) {
    result.error_message = "TrajectoryAssembler: no segments to merge";
    return result;
  }

  // ── Consistent joint_names check ──────────────────────────────────────
  const auto &reference_joint_names = segments.front().joint_names;
  for (std::size_t i = 0; i < segments.size(); ++i) {
    if (segments[i].joint_names != reference_joint_names) {
      result.error_message =
          "TrajectoryAssembler: joint_names mismatch at segment[" +
          std::to_string(i) + "]";
      return result;
    }
  }

  result.trajectory.joint_names = reference_joint_names;
  int64_t accumulated_ns = 0;

  for (std::size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
    const auto &segment = segments[seg_idx];

    if (segment.points.empty()) {
      result.error_message = "TrajectoryAssembler: segment[" +
                             std::to_string(seg_idx) + "] is empty";
      result.trajectory.points.clear();
      return result;
    }

    for (std::size_t pt_idx = 0; pt_idx < segment.points.size(); ++pt_idx) {
      const auto &point = segment.points[pt_idx];

      // ── Skip duplicate first point of segment N+1 if identical to last ──
      if (seg_idx > 0 && pt_idx == 0 && !result.trajectory.points.empty()) {
        const auto &last = result.trajectory.points.back();
        bool identical = (point.positions.size() == last.positions.size());
        if (identical) {
          for (std::size_t j = 0; j < point.positions.size(); ++j) {
            if (point.positions[j] != last.positions[j]) {
              identical = false;
              break;
            }
          }
        }
        if (identical) {
          continue;
        }
      }

      auto new_point = point;
      int64_t point_ns = static_cast<int64_t>(point.time_from_start.sec) *
                             1'000'000'000LL +
                         static_cast<int64_t>(point.time_from_start.nanosec);
      int64_t total_ns = accumulated_ns + point_ns;

      // Ensure monotonic: bump by 1 ns if this point lands on or before the
      // previous point (can happen for the first point of a new segment).
      if (!result.trajectory.points.empty()) {
        const auto &prev = result.trajectory.points.back();
        int64_t prev_ns =
            static_cast<int64_t>(prev.time_from_start.sec) *
                1'000'000'000LL +
            static_cast<int64_t>(prev.time_from_start.nanosec);
        if (total_ns <= prev_ns) {
          total_ns = prev_ns + 1;
        }
      }

      new_point.time_from_start.sec =
          static_cast<int32_t>(total_ns / 1'000'000'000LL);
      new_point.time_from_start.nanosec =
          static_cast<uint32_t>(total_ns % 1'000'000'000LL);

      result.trajectory.points.push_back(std::move(new_point));
    }

    // Advance accumulated time by the last point of this segment
    const auto &last_pt = segment.points.back();
    accumulated_ns +=
        static_cast<int64_t>(last_pt.time_from_start.sec) * 1'000'000'000LL +
        static_cast<int64_t>(last_pt.time_from_start.nanosec);
  }

  result.total_points = result.trajectory.points.size();

  if (result.total_points == 0) {
    result.error_message = "TrajectoryAssembler: merged trajectory is empty";
    return result;
  }

  if (result.total_points > kMaxTrajectoryPoints) {
    result.error_message =
        "TrajectoryAssembler: total points (" +
        std::to_string(result.total_points) + ") exceeds budget (" +
        std::to_string(kMaxTrajectoryPoints) +
        ")";
    result.trajectory.points.clear();
    return result;
  }

  // ── Monotonic timestamp check ────────────────────────────────────────
  for (std::size_t i = 1; i < result.trajectory.points.size(); ++i) {
    int64_t prev_ns =
        static_cast<int64_t>(result.trajectory.points[i - 1].time_from_start.sec) *
            1'000'000'000LL +
        static_cast<int64_t>(result.trajectory.points[i - 1].time_from_start.nanosec);
    int64_t curr_ns =
        static_cast<int64_t>(result.trajectory.points[i].time_from_start.sec) *
            1'000'000'000LL +
        static_cast<int64_t>(result.trajectory.points[i].time_from_start.nanosec);
    if (curr_ns <= prev_ns) {
      result.error_message =
          "TrajectoryAssembler: non-monotonic timestamp at point " +
          std::to_string(i);
      result.trajectory.points.clear();
      return result;
    }
  }

  result.success = true;
  return result;
}

} // namespace motion_core
