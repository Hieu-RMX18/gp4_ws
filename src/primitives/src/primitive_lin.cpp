// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#include "primitives/primitive_lin.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>

#include "motion_core/orientation_filter.hpp"
#include "motion_core/quality_gate.hpp"
#include "motion_core/trajectory_post_processor.hpp"

namespace primitives
{
namespace
{
constexpr double kWaitForServersTimeoutSec = 10.0;
constexpr double kCartesianEefStep = 0.005;
constexpr double kCartesianJumpThreshold = 1.0;
constexpr double kDefaultVelocityCap = 0.3;
constexpr double kDefaultAccelerationCap = 0.2;
constexpr double kDefaultVelocityScale = 0.3;
constexpr double kDefaultAccelerationScale = 0.2;

double sanitize_positive_scale(double value, double fallback)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    return fallback;
  }

  return std::min(value, 1.0);
}

double read_scale_parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  double fallback)
{
  if (!node)
  {
    return fallback;
  }

  if (!node->has_parameter(name))
  {
    node->declare_parameter<double>(name, fallback);
  }

  double value = fallback;
  node->get_parameter(name, value);
  return sanitize_positive_scale(value, fallback);
}

double resolve_scale(double requested, double fallback, double cap)
{
  const double resolved = sanitize_positive_scale(requested, fallback);
  return std::min(resolved, cap);
}

PrimitiveFailReason map_quality_failure(const std::string & reason)
{
  if (reason.find("wrist flip guard reject") != std::string::npos)
  {
    return PrimitiveFailReason::WRIST_FLIP_DETECTED;
  }
  if (reason.find("trajectory exceeds point limit") != std::string::npos)
  {
    return PrimitiveFailReason::TRAJECTORY_TOO_LONG;
  }
  if (reason.find("cartesian fraction below minimum threshold") != std::string::npos)
  {
    return PrimitiveFailReason::CARTESIAN_FRACTION_LOW;
  }

  return PrimitiveFailReason::QUALITY_GATE_REJECTED;
}

PrimitiveResult make_result_failure(PrimitiveFailReason reason, const std::string & message)
{
  PrimitiveResult failure;
  failure.success = false;
  failure.reason = reason;
  failure.message = message;
  return failure;
}

class MoveGroupLinearExecutionBackend final : public LinearExecutionBackend
{
public:
  explicit MoveGroupLinearExecutionBackend(MoveGroupInterface & mgi)
  : mgi_(mgi),
    node_(std::make_shared<rclcpp::Node>(
      "primitive_lin_backend",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
    quality_gate_(
      motion_core::TrajectoryPostProcessor::kMaxTrajectoryPoints,
      motion_core::QualityGate::kMinimumCartesianFraction)
  {
    // FRAME CONVENTION: all Cartesian waypoints are expressed in MoveIt's planning frame (world frame).
    mgi_.setPoseReferenceFrame(mgi_.getPlanningFrame());
  }

  bool wait_for_servers(std::string & reason) override
  {
    if (mgi_.getMoveGroupClient().wait_for_action_server(
          std::chrono::duration<double>(kWaitForServersTimeoutSec)))
    {
      reason.clear();
      return true;
    }

    reason = "MoveGroup action server unavailable after 10.0 seconds";
    return false;
  }

  bool normalize_pose(geometry_msgs::msg::Pose & pose, std::string & reason) override
  {
    if (!orientation_filter_.normalize_and_validate(pose, reason))
    {
      return false;
    }

    reason.clear();
    return true;
  }

  bool get_current_pose_world(geometry_msgs::msg::Pose & pose, std::string & reason) override
  {
    const auto current_pose = mgi_.getCurrentPose(mgi_.getEndEffectorLink());
    pose = current_pose.pose;

    const auto all_finite = [](const geometry_msgs::msg::Pose & p) {
        return std::isfinite(p.position.x) &&
               std::isfinite(p.position.y) &&
               std::isfinite(p.position.z) &&
               std::isfinite(p.orientation.x) &&
               std::isfinite(p.orientation.y) &&
               std::isfinite(p.orientation.z) &&
               std::isfinite(p.orientation.w);
      };

    if (!all_finite(pose))
    {
      reason = "current pose contains non-finite values";
      return false;
    }

    reason.clear();
    return true;
  }

  bool compute_cartesian_path(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double & fraction,
    std::string & reason) override
  {
    planned_trajectory_ = moveit_msgs::msg::RobotTrajectory{};
    fraction = mgi_.computeCartesianPath(
      waypoints,
      kCartesianEefStep,
      kCartesianJumpThreshold,
      planned_trajectory_,
      true);

    if (fraction < 0.0)
    {
      reason = "computeCartesianPath returned an error";
      return false;
    }

    if (planned_trajectory_.joint_trajectory.points.empty())
    {
      reason = "computeCartesianPath returned empty joint trajectory";
      return false;
    }

    reason.clear();
    return true;
  }

  LinearScalingConfig scaling_config() const override
  {
    LinearScalingConfig config;
    config.velocity_cap = read_scale_parameter(
      node_,
      "primitives.max_velocity_scaling_factor",
      kDefaultVelocityCap);
    config.acceleration_cap = read_scale_parameter(
      node_,
      "primitives.max_acceleration_scaling_factor",
      kDefaultAccelerationCap);
    config.default_velocity = read_scale_parameter(
      node_,
      "primitives.default_velocity_scaling_factor",
      kDefaultVelocityScale);
    config.default_acceleration = read_scale_parameter(
      node_,
      "primitives.default_acceleration_scaling_factor",
      kDefaultAccelerationScale);

    config.default_velocity = std::min(config.default_velocity, config.velocity_cap);
    config.default_acceleration = std::min(config.default_acceleration, config.acceleration_cap);
    return config;
  }

  PrimitiveResult postprocess_and_validate(
    double velocity_scale,
    double acceleration_scale,
    double cartesian_fraction) override
  {
    auto current_state = mgi_.getCurrentState(1.0);
    if (!current_state)
    {
      return make_result_failure(
        PrimitiveFailReason::PLANNING_TIMEOUT,
        "LIN post-processing failed: current robot state unavailable");
    }

    robot_trajectory::RobotTrajectory robot_traj(mgi_.getRobotModel(), mgi_.getName());
    robot_traj.setRobotTrajectoryMsg(*current_state, planned_trajectory_);

    std::string post_reason;
    if (!post_processor_.apply_totg(robot_traj, velocity_scale, acceleration_scale, post_reason))
    {
      return make_result_failure(PrimitiveFailReason::TOTG_FAILED, "LIN TOTG failed: " + post_reason);
    }

    moveit_msgs::msg::RobotTrajectory postprocessed_msg;
    robot_traj.getRobotTrajectoryMsg(postprocessed_msg);
    trajectory_msgs::msg::JointTrajectory output_traj = postprocessed_msg.joint_trajectory;

    if (!post_processor_.downsample_to_max_points(
          output_traj,
          motion_core::TrajectoryPostProcessor::kMaxTrajectoryPoints,
          post_reason))
    {
      return make_result_failure(
        PrimitiveFailReason::TRAJECTORY_TOO_LONG,
        "LIN downsample failed: " + post_reason);
    }

    std::string quality_reason;
    if (!quality_gate_.validate_plan(
          output_traj,
          cartesian_fraction,
          "LIN",
          quality_reason))
    {
      return make_result_failure(
        map_quality_failure(quality_reason),
        "LIN quality gate rejected: " + quality_reason);
    }

    PrimitiveResult success;
    success.success = true;
    success.reason = PrimitiveFailReason::UNKNOWN;
    success.message = "LIN plan ready";
    success.trajectory_points = output_traj.points.size();
    return success;
  }

private:
  MoveGroupInterface & mgi_;
  rclcpp::Node::SharedPtr node_;
  moveit_msgs::msg::RobotTrajectory planned_trajectory_;

  motion_core::OrientationFilter orientation_filter_;
  motion_core::TrajectoryPostProcessor post_processor_;
  motion_core::QualityGate quality_gate_;
};
}  // namespace

std::unique_ptr<LinearExecutionBackend> make_move_group_linear_backend(MoveGroupInterface & mgi)
{
  return std::make_unique<MoveGroupLinearExecutionBackend>(mgi);
}

PrimitiveResult PrimitiveLin::execute(const LINGoal & goal, MoveGroupInterface & mgi)
{
  auto backend = make_move_group_linear_backend(mgi);
  return execute(goal, *backend);
}

PrimitiveResult PrimitiveLin::execute(const ExecuteMotionGoal & goal, MoveGroupInterface & mgi)
{
  LINGoal lin_goal;
  lin_goal.target_pose = goal.target_pose;
  lin_goal.velocity_scale = goal.velocity_scale;
  lin_goal.acceleration_scale = goal.acceleration_scale;
  return execute(lin_goal, mgi);
}

PrimitiveResult PrimitiveLin::execute(const LINGoal & goal, LinearExecutionBackend & backend)
{
  std::string reason;
  geometry_msgs::msg::Pose target_pose = goal.target_pose;
  if (!backend.normalize_pose(target_pose, reason))
  {
    return make_failure(PrimitiveFailReason::INVALID_ORIENTATION, reason);
  }

  geometry_msgs::msg::Pose current_pose;
  if (!backend.get_current_pose_world(current_pose, reason))
  {
    return make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, reason);
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(2);
  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);

  return execute_waypoints(waypoints, goal.velocity_scale, goal.acceleration_scale, backend);
}

PrimitiveResult PrimitiveLin::execute_waypoints(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  double velocity_scale,
  double acceleration_scale,
  LinearExecutionBackend & backend)
{
  std::string reason;
  if (!backend.wait_for_servers(reason))
  {
    return make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, reason);
  }

  double fraction = 0.0;
  if (!backend.compute_cartesian_path(waypoints, fraction, reason))
  {
    return make_failure(PrimitiveFailReason::UNKNOWN, reason);
  }

  if (fraction < motion_core::QualityGate::kMinimumCartesianFraction)
  {
    std::ostringstream stream;
    stream << "cartesian fraction below minimum threshold: " << fraction;
    return make_failure(PrimitiveFailReason::CARTESIAN_FRACTION_LOW, stream.str());
  }

  const LinearScalingConfig scales = backend.scaling_config();
  const double resolved_velocity = resolve_scale(velocity_scale, scales.default_velocity, scales.velocity_cap);
  const double resolved_acceleration = resolve_scale(
    acceleration_scale,
    scales.default_acceleration,
    scales.acceleration_cap);

  return backend.postprocess_and_validate(resolved_velocity, resolved_acceleration, fraction);
}
}  // namespace primitives
