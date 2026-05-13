#pragma once
/// @file move_rel_validator.hpp
/// Pure-function validation for MOVE_REL translation commands.
/// Single source of truth for workspace bounds and delta limits
/// used by both motion_core_node and unit tests.

#include <cmath>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose.hpp>

namespace motion_core {

struct MoveRelLimits {
  // Max single-command translation norm (meters).
  // MUST match safety_rules.yaml motion_limits.max_move_rel_translation.
  // Update both locations together. Hardware pass keeps MOVE_REL short and
  // conservative for real-cell nudges only.
  static constexpr double kMaxDeltaNorm = 0.05;

  // Workspace bounds — MUST match safety_rules.yaml workspace_bounds.
  // Derived from current gp4_station xacro station mesh in base_link:
  // X[-0.482,+0.761] Y[-0.197,+0.806] Z[-0.757,+1.093].
  // Hardware policy is conservative: do not expand positive X beyond +0.45.
  static constexpr double kXMin =
      -0.45; // near side wall -0.482 + margin (rounded conservative)
  static constexpr double kXMax = 0.45; // conservative positive cap
  static constexpr double kYMin =
      -0.16; // front wall -0.197 + 37mm; above front_wall_guard max
  static constexpr double kYMax = 0.52; // reach-limited
  static constexpr double kZMin = 0.23; // table top ~0.20 + 30mm
  static constexpr double kZMax =
      0.65; // raised per operator approval for taller workpieces

  // Station wall keepout zones — MUST mirror safety_rules.yaml forbidden_zones.
  // front_wall_guard: station front face Y = -0.197m, 30mm inflated zone.
  static constexpr double kFrontWallX = 0.00;
  static constexpr double kFrontWallY = -0.197;
  static constexpr double kFrontWallZ = 0.00;
  static constexpr double kFrontWallSizeX = 1.30;
  static constexpr double kFrontWallSizeY = 0.06;
  static constexpr double kFrontWallSizeZ = 1.50;

  // right_wall_guard: station side wall on negative X, center X = -0.482m.
  static constexpr double kRightWallX = -0.482;
  static constexpr double kRightWallY = 0.305;
  static constexpr double kRightWallZ = 0.00;
  static constexpr double kRightWallSizeX = 0.06;
  static constexpr double kRightWallSizeY = 1.10;
  static constexpr double kRightWallSizeZ = 1.50;

  // floor_clearance_guard: low-Z table/floor zone, Z in [0.00, 0.20].
  static constexpr double kFloorClearanceX = 0.00;
  static constexpr double kFloorClearanceY = 0.30;
  static constexpr double kFloorClearanceZ = 0.10;
  static constexpr double kFloorClearanceSizeX = 1.50;
  static constexpr double kFloorClearanceSizeY = 1.10;
  static constexpr double kFloorClearanceSizeZ = 0.20;
};

/// Validate that reference_frame is either empty or "base_link".
/// Returns false with a reason string when the frame is unsupported.
inline bool validate_move_rel_frame(const std::string &reference_frame,
                                    std::string &reason) {
  if (!reference_frame.empty() && reference_frame != "base_link") {
    reason = "MOVE_REL: unsupported reference_frame '" + reference_frame +
             "'; only 'base_link' is supported in this step";
    return false;
  }
  return true;
}

/// Validate that delta components are not all zero and that the
/// Euclidean norm does not exceed the safety limit.
inline bool validate_move_rel_deltas(double dx, double dy, double dz,
                                     std::string &reason) {
  if (dx == 0.0 && dy == 0.0 && dz == 0.0) {
    reason = "MOVE_REL: all delta components are zero; at least one must be "
             "non-zero";
    return false;
  }

  const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (norm > MoveRelLimits::kMaxDeltaNorm) {
    std::ostringstream oss;
    oss << "MOVE_REL: delta norm " << norm << " m exceeds safety limit "
        << MoveRelLimits::kMaxDeltaNorm << " m";
    reason = oss.str();
    return false;
  }
  return true;
}

/// Compute absolute target = current_pose + (dx, dy, dz).
/// Orientation is copied from current_pose (preserved).
inline geometry_msgs::msg::Pose
compute_move_rel_target(const geometry_msgs::msg::Pose &current, double dx,
                        double dy, double dz) {
  geometry_msgs::msg::Pose target;
  target.position.x = current.position.x + dx;
  target.position.y = current.position.y + dy;
  target.position.z = current.position.z + dz;
  target.orientation = current.orientation;
  return target;
}

/// Validate that the computed target is inside workspace bounds.
inline bool
validate_move_rel_target_bounds(const geometry_msgs::msg::Pose &target,
                                std::string &reason) {
  if (target.position.x < MoveRelLimits::kXMin ||
      target.position.x > MoveRelLimits::kXMax ||
      target.position.y < MoveRelLimits::kYMin ||
      target.position.y > MoveRelLimits::kYMax ||
      target.position.z < MoveRelLimits::kZMin ||
      target.position.z > MoveRelLimits::kZMax) {
    std::ostringstream oss;
    oss << "MOVE_REL: computed target (" << target.position.x << ", "
        << target.position.y << ", " << target.position.z
        << ") is outside workspace bounds";
    reason = oss.str();
    return false;
  }

  // Defense-in-depth:
  // Under the current conservative workspace policy, points that would enter
  // these keepout zones are already rejected by the workspace bounds above.
  // Keep these explicit AABB checks so future workspace expansions cannot
  // silently bypass wall/floor guard intent.
  const auto inside_aabb = [](const geometry_msgs::msg::Pose &pose, double cx,
                              double cy, double cz, double sx, double sy,
                              double sz) -> bool {
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
          target, MoveRelLimits::kFrontWallX, MoveRelLimits::kFrontWallY,
          MoveRelLimits::kFrontWallZ, MoveRelLimits::kFrontWallSizeX,
          MoveRelLimits::kFrontWallSizeY, MoveRelLimits::kFrontWallSizeZ)) {
    reason = "MOVE_REL: computed target intersects forbidden zone "
             "'front_wall_guard'";
    return false;
  }

  if (inside_aabb(
          target, MoveRelLimits::kRightWallX, MoveRelLimits::kRightWallY,
          MoveRelLimits::kRightWallZ, MoveRelLimits::kRightWallSizeX,
          MoveRelLimits::kRightWallSizeY, MoveRelLimits::kRightWallSizeZ)) {
    reason = "MOVE_REL: computed target intersects forbidden zone "
             "'right_wall_guard'";
    return false;
  }

  if (inside_aabb(target, MoveRelLimits::kFloorClearanceX,
                  MoveRelLimits::kFloorClearanceY,
                  MoveRelLimits::kFloorClearanceZ,
                  MoveRelLimits::kFloorClearanceSizeX,
                  MoveRelLimits::kFloorClearanceSizeY,
                  MoveRelLimits::kFloorClearanceSizeZ)) {
    reason = "MOVE_REL: computed target intersects forbidden zone "
             "'floor_clearance_guard'";
    return false;
  }

  return true;
}

} // namespace motion_core
