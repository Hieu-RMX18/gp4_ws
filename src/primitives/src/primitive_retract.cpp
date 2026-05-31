// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#include "primitives/primitive_retract.hpp"

#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace primitives {
namespace {
geometry_msgs::msg::Pose
offset_along_eef_local_z_world(const geometry_msgs::msg::Pose &reference_pose,
                               double signed_distance) {
  geometry_msgs::msg::Pose offset_pose = reference_pose;

  const tf2::Quaternion orientation(
      reference_pose.orientation.x, reference_pose.orientation.y,
      reference_pose.orientation.z, reference_pose.orientation.w);

  // FRAME CONVENTION: apply ±Z offset in the EEF local frame, then express the
  // translated pose in WORLD frame.
  const tf2::Vector3 eef_z_world =
      tf2::quatRotate(orientation, tf2::Vector3(0.0, 0.0, 1.0));
  offset_pose.position.x += signed_distance * eef_z_world.x();
  offset_pose.position.y += signed_distance * eef_z_world.y();
  offset_pose.position.z += signed_distance * eef_z_world.z();

  return offset_pose;
}
} // namespace

PrimitiveResult PrimitiveRetract::execute(const RetractGoal &goal,
                                          MoveGroupInterface &mgi) {
  auto backend = make_move_group_linear_backend(mgi);
  return execute(goal, *backend);
}

PrimitiveResult PrimitiveRetract::execute(const ExecuteMotionGoal &goal,
                                          MoveGroupInterface &mgi) {
  if (goal.joint_target.empty()) {
    return make_failure(PrimitiveFailReason::UNKNOWN,
                        "ExecuteMotion goal for RETRACT does not provide "
                        "retract_distance; use typed RetractGoal API");
  }

  RetractGoal retract_goal;
  retract_goal.retract_distance = goal.joint_target.front();
  retract_goal.velocity_scale = goal.velocity_scale;
  retract_goal.acceleration_scale = goal.acceleration_scale;
  return execute(retract_goal, mgi);
}

PrimitiveResult PrimitiveRetract::execute(const RetractGoal &goal,
                                          LinearExecutionBackend &backend) {
  if (goal.retract_distance <= 0.0) {
    return make_failure(PrimitiveFailReason::INVALID_DISTANCE_PARAM,
                        "retract_distance must be greater than 0");
  }

  std::string reason;
  geometry_msgs::msg::Pose current_pose;
  if (!backend.get_current_pose_world(current_pose, reason)) {
    return make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, reason);
  }

  if (!backend.normalize_pose(current_pose, reason)) {
    return make_failure(PrimitiveFailReason::INVALID_ORIENTATION, reason);
  }

  const geometry_msgs::msg::Pose retract_pose =
      offset_along_eef_local_z_world(current_pose, goal.retract_distance);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(2);
  waypoints.push_back(current_pose);
  waypoints.push_back(retract_pose);

  PrimitiveLin lin_primitive;
  return lin_primitive.execute_waypoints(waypoints, goal.velocity_scale,
                                         goal.acceleration_scale, backend);
}
} // namespace primitives
