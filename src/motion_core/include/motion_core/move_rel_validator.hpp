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
  // MUST match safety_rules.yaml motion_limits.max_move_rel_translation.
  // Update both locations together. This pass raises from 0.03 to 0.08
  // to enable practical short task nudges while preserving hardware safety.
  static constexpr double kMaxDeltaNorm = 0.08;

  // Workspace bounds for the computed absolute target.
  // These MUST match safety_rules.yaml; if safety_rules.yaml changes,
  // update here and add a cross-check test.  Centralising in one header
  // removes the duplication that previously existed between
  // motion_core_node.cpp and safety_rules.yaml.
  static constexpr double kXMin = -0.25;
  static constexpr double kXMax =  0.38;
  static constexpr double kYMin = -0.25;
  static constexpr double kYMax =  0.38;
  static constexpr double kZMin =  0.20;
  static constexpr double kZMax =  0.56;

  // Commissioning keepout zones — center/size AABB form to mirror safety_rules.yaml.
  static constexpr double kTableClearanceX = 0.10;
  static constexpr double kTableClearanceY = 0.05;
  static constexpr double kTableClearanceZ = 0.09;
  static constexpr double kTableClearanceSizeX = 1.10;
  static constexpr double kTableClearanceSizeY = 0.95;
  static constexpr double kTableClearanceSizeZ = 0.18;

  static constexpr double kAvoidLeftX = -0.22;
  static constexpr double kAvoidLeftY = 0.21;
  static constexpr double kAvoidLeftZ = 0.35;
  static constexpr double kAvoidLeftSizeX = 0.22;
  static constexpr double kAvoidLeftSizeY = 0.24;
  static constexpr double kAvoidLeftSizeZ = 0.70;

  static constexpr double kWallX = 0.34;
  static constexpr double kWallY = 0.32;
  static constexpr double kWallZ = 0.35;
  static constexpr double kWallSizeX = 0.18;
  static constexpr double kWallSizeY = 0.32;
  static constexpr double kWallSizeZ = 0.70;

  static constexpr double kCornerGuardX = 0.24;
  static constexpr double kCornerGuardY = 0.24;
  static constexpr double kCornerGuardZ = 0.35;
  static constexpr double kCornerGuardSizeX = 0.22;
  static constexpr double kCornerGuardSizeY = 0.22;
  static constexpr double kCornerGuardSizeZ = 0.70;
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

  const auto inside_aabb = [](
    const geometry_msgs::msg::Pose & pose,
    double cx, double cy, double cz,
    double sx, double sy, double sz) -> bool
  {
    const double min_x = cx - sx / 2.0;
    const double max_x = cx + sx / 2.0;
    const double min_y = cy - sy / 2.0;
    const double max_y = cy + sy / 2.0;
    const double min_z = cz - sz / 2.0;
    const double max_z = cz + sz / 2.0;
    return pose.position.x >= min_x && pose.position.x <= max_x &&
           pose.position.y >= min_y && pose.position.y <= max_y &&
           pose.position.z >= min_z && pose.position.z <= max_z;
  };

  if (inside_aabb(
        target,
        MoveRelLimits::kTableClearanceX,
        MoveRelLimits::kTableClearanceY,
        MoveRelLimits::kTableClearanceZ,
        MoveRelLimits::kTableClearanceSizeX,
        MoveRelLimits::kTableClearanceSizeY,
        MoveRelLimits::kTableClearanceSizeZ))
  {
    reason = "MOVE_REL: computed target intersects forbidden zone 'table_clearance_guard'";
    return false;
  }

  if (inside_aabb(
        target,
        MoveRelLimits::kAvoidLeftX, MoveRelLimits::kAvoidLeftY, MoveRelLimits::kAvoidLeftZ,
        MoveRelLimits::kAvoidLeftSizeX, MoveRelLimits::kAvoidLeftSizeY, MoveRelLimits::kAvoidLeftSizeZ))
  {
    reason = "MOVE_REL: computed target intersects forbidden zone 'avoid_left_region'";
    return false;
  }

  if (inside_aabb(
        target,
        MoveRelLimits::kWallX, MoveRelLimits::kWallY, MoveRelLimits::kWallZ,
        MoveRelLimits::kWallSizeX, MoveRelLimits::kWallSizeY, MoveRelLimits::kWallSizeZ))
  {
    reason = "MOVE_REL: computed target intersects forbidden zone 'wall_region'";
    return false;
  }

  if (inside_aabb(
        target,
        MoveRelLimits::kCornerGuardX, MoveRelLimits::kCornerGuardY, MoveRelLimits::kCornerGuardZ,
        MoveRelLimits::kCornerGuardSizeX, MoveRelLimits::kCornerGuardSizeY, MoveRelLimits::kCornerGuardSizeZ))
  {
    reason = "MOVE_REL: computed target intersects forbidden zone 'corner_clearance_guard'";
    return false;
  }

  return true;
}

}  // namespace motion_core
