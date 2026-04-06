#include "motion_core/wrist_flip_guard.hpp"

#include <cmath>
#include <sstream>

namespace motion_core {

double WristFlipGuard::max_delta_for_joint(std::size_t joint_idx) {
  // V4 F1: Per-joint thresholds
  switch (joint_idx) {
  case 0: // joint_1_s
  case 1: // joint_2_l
  case 2: // joint_3_u
    return kDeltaRadJ123;
  case 3: // joint_4_r
  case 4: // joint_5_b
    return kDeltaRadJ45;
  case 5: // joint_6_t — shortest angular distance unwrap
    return kDeltaRadJ6;
  default:
    return kDeltaRadJ123; // conservative fallback
  }
}

bool WristFlipGuard::check_trajectory(
    const trajectory_msgs::msg::JointTrajectory &traj,
    std::string &reason) const {
  reason.clear();

  if (traj.points.empty()) {
    reason = "trajectory has no points";
    return false;
  }

  if (traj.joint_names.empty()) {
    reason = "trajectory has no joint names";
    return false;
  }

  const std::size_t expected_joint_count = traj.joint_names.size();

  for (std::size_t i = 0; i < traj.points.size(); ++i) {
    const auto &point = traj.points[i];
    if (point.positions.size() != expected_joint_count) {
      std::ostringstream stream;
      stream << "malformed trajectory point " << i << ": expected "
             << expected_joint_count << " positions, got "
             << point.positions.size();
      reason = stream.str();
      return false;
    }

    for (std::size_t joint_idx = 0; joint_idx < point.positions.size();
         ++joint_idx) {
      if (!std::isfinite(point.positions[joint_idx])) {
        std::ostringstream stream;
        stream << "malformed trajectory point " << i
               << ": non-finite position at joint index " << joint_idx;
        reason = stream.str();
        return false;
      }
    }
  }

  if (traj.points.size() < 2) {
    return true;
  }

  // V4 F1: Per-joint continuity check with per-joint thresholds
  for (std::size_t i = 1; i < traj.points.size(); ++i) {
    const auto &previous = traj.points[i - 1];
    const auto &current = traj.points[i];

    for (std::size_t joint_idx = 0; joint_idx < current.positions.size();
         ++joint_idx) {
      double raw_delta =
          current.positions[joint_idx] - previous.positions[joint_idx];

      // V4 F1: For joint_6_t (index 5), use shortest angular distance (unwrap)
      if (joint_idx == 5) {
        // Normalize to [-pi, pi]
        while (raw_delta > M_PI) {
          raw_delta -= 2.0 * M_PI;
        }
        while (raw_delta < -M_PI) {
          raw_delta += 2.0 * M_PI;
        }
      }

      const double delta = std::abs(raw_delta);
      const double threshold = max_delta_for_joint(joint_idx);

      if (delta > threshold) {
        std::ostringstream stream;
        stream << "wrist flip guard reject at segment " << (i - 1) << "->" << i
               << ", joint '" << traj.joint_names[joint_idx]
               << "': delta=" << delta << " rad > " << threshold << " rad";
        reason = stream.str();
        return false;
      }
    }
  }

  // V4 F2: Check for repeated sign-flips on wrist joints
  if (!check_wrist_sign_flips(traj, reason)) {
    return false;
  }

  return true;
}

bool WristFlipGuard::check_wrist_sign_flips(
    const trajectory_msgs::msg::JointTrajectory &traj,
    std::string &reason) const {
  if (traj.points.size() < 3) {
    return true;
  }

  // V4 F3: Check wrist joints (indices 3, 4, 5) for repeated sign inversions.
  // A sign-flip is when the direction of motion reverses consecutively.
  // More than 2 consecutive sign-flips indicates unstable wrist branch
  // selection.
  static constexpr std::size_t kWristJointIndices[] = {3, 4, 5};
  static constexpr std::size_t kMaxConsecutiveSignFlips = 5;

  for (std::size_t joint_idx : kWristJointIndices) {
    if (joint_idx >= traj.joint_names.size()) {
      continue;
    }

    std::size_t consecutive_flips = 0;
    double prev_delta = 0.0;

    for (std::size_t i = 1; i < traj.points.size(); ++i) {
      double delta = traj.points[i].positions[joint_idx] -
                     traj.points[i - 1].positions[joint_idx];

      // Apply J6 unwrap
      if (joint_idx == 5) {
        while (delta > M_PI) {
          delta -= 2.0 * M_PI;
        }
        while (delta < -M_PI) {
          delta += 2.0 * M_PI;
        }
      }

      // Skip near-zero deltas (no motion)
      if (std::abs(delta) < 1e-6) {
        continue;
      }

      if (i > 1 && prev_delta != 0.0) {
        // Sign flip: previous delta was positive, current is negative, or vice
        // versa
        if ((prev_delta > 0.0 && delta < 0.0) ||
            (prev_delta < 0.0 && delta > 0.0)) {
          ++consecutive_flips;
          if (consecutive_flips > kMaxConsecutiveSignFlips) {
            std::ostringstream stream;
            stream << "wrist sign-flip reject: joint '"
                   << traj.joint_names[joint_idx] << "' has "
                   << consecutive_flips
                   << " consecutive sign reversals (threshold="
                   << kMaxConsecutiveSignFlips
                   << ") — indicates unstable wrist branch";
            reason = stream.str();
            return false;
          }
        } else {
          consecutive_flips = 0;
        }
      }

      prev_delta = delta;
    }
  }

  return true;
}

} // namespace motion_core
