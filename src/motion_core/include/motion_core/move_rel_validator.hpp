#pragma once
/// @file move_rel_validator.hpp
/// Pure-function validation for MOVE_REL translation commands.
/// Single source of truth for workspace bounds and delta limits
/// used by both motion_core_node and unit tests.

#include <cmath>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose.hpp>

namespace motion_core
{

struct MoveRelLimits
{
  // Max single-command translation norm (meters).
  static constexpr double kMaxDeltaNorm = 0.20;

  // Workspace bounds for the computed absolute target.
  // These MUST match safety_rules.yaml; if safety_rules.yaml changes,
  // update here and add a cross-check test.  Centralising in one header
  // removes the duplication that previously existed between
  // motion_core_node.cpp and safety_rules.yaml.
  static constexpr double kXMin = -0.8;
  static constexpr double kXMax =  0.8;
  static constexpr double kYMin = -0.8;
  static constexpr double kYMax =  0.8;
  static constexpr double kZMin =  0.02;
  static constexpr double kZMax =  1.2;
};

/// Validate that reference_frame is either empty or "base_link".
/// Returns false with a reason string when the frame is unsupported.
inline bool validate_move_rel_frame(
  const std::string & reference_frame,
  std::string & reason)
{
  if (!reference_frame.empty() && reference_frame != "base_link")
  {
    reason = "MOVE_REL: unsupported reference_frame '" + reference_frame +
             "'; only 'base_link' is supported in this step";
    return false;
  }
  return true;
}

/// Validate that delta components are not all zero and that the
/// Euclidean norm does not exceed the safety limit.
inline bool validate_move_rel_deltas(
  double dx, double dy, double dz,
  std::string & reason)
{
  if (dx == 0.0 && dy == 0.0 && dz == 0.0)
  {
    reason = "MOVE_REL: all delta components are zero; at least one must be non-zero";
    return false;
  }

  const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (norm > MoveRelLimits::kMaxDeltaNorm)
  {
    std::ostringstream oss;
    oss << "MOVE_REL: delta norm " << norm
        << " m exceeds safety limit " << MoveRelLimits::kMaxDeltaNorm << " m";
    reason = oss.str();
    return false;
  }
  return true;
}

/// Compute absolute target = current_pose + (dx, dy, dz).
/// Orientation is copied from current_pose (preserved).
inline geometry_msgs::msg::Pose compute_move_rel_target(
  const geometry_msgs::msg::Pose & current,
  double dx, double dy, double dz)
{
  geometry_msgs::msg::Pose target;
  target.position.x = current.position.x + dx;
  target.position.y = current.position.y + dy;
  target.position.z = current.position.z + dz;
  target.orientation = current.orientation;
  return target;
}

/// Validate that the computed target is inside workspace bounds.
inline bool validate_move_rel_target_bounds(
  const geometry_msgs::msg::Pose & target,
  std::string & reason)
{
  if (target.position.x < MoveRelLimits::kXMin || target.position.x > MoveRelLimits::kXMax ||
      target.position.y < MoveRelLimits::kYMin || target.position.y > MoveRelLimits::kYMax ||
      target.position.z < MoveRelLimits::kZMin || target.position.z > MoveRelLimits::kZMax)
  {
    std::ostringstream oss;
    oss << "MOVE_REL: computed target ("
        << target.position.x << ", "
        << target.position.y << ", "
        << target.position.z
        << ") is outside workspace bounds";
    reason = oss.str();
    return false;
  }
  return true;
}

}  // namespace motion_core
