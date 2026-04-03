// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#include "primitives/primitive_circ.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iterator>
#include <memory>
#include <string>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "motion_core/orientation_filter.hpp"
#include "motion_core/quality_gate.hpp"
#include "motion_core/trajectory_post_processor.hpp"

namespace primitives
{
namespace
{
constexpr double kWaitForServersTimeoutSec = 10.0;
constexpr double kDefaultVelocityCap = 0.3;
constexpr double kDefaultAccelerationCap = 0.2;
constexpr double kDefaultVelocityScale = 0.3;
constexpr double kDefaultAccelerationScale = 0.2;
constexpr double kPositionEpsilonM = 1e-4;
constexpr double kInterimConstraintRadiusM = 1e-4;

class PathConstraintGuard
{
public:
  explicit PathConstraintGuard(CircExecutionBackend & backend)
  : backend_(backend)
  {
  }

  ~PathConstraintGuard()
  {
    if (enabled_)
    {
      backend_.clear_path_constraints();
    }
  }

  void enable()
  {
    enabled_ = true;
  }

private:
  CircExecutionBackend & backend_;
  bool enabled_{ false };
};

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

double position_distance(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

bool contains_degenerate_arc_hint(const std::string & message)
{
  std::string lower;
  lower.reserve(message.size());
  std::transform(
    message.begin(),
    message.end(),
    std::back_inserter(lower),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  return lower.find("degenerate") != std::string::npos ||
         lower.find("collinear") != std::string::npos ||
         lower.find("zero radius") != std::string::npos ||
         lower.find("arc undefined") != std::string::npos;
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

class MoveGroupCircExecutionBackend final : public CircExecutionBackend
{
public:
  explicit MoveGroupCircExecutionBackend(MoveGroupInterface & mgi)
  : mgi_(mgi),
    node_(std::make_shared<rclcpp::Node>(
      "primitive_circ_backend",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
    quality_gate_(
      motion_core::TrajectoryPostProcessor::kMaxTrajectoryPoints,
      motion_core::QualityGate::kMinimumCartesianFraction)
  {
    mgi_.setPoseReferenceFrame("world");
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

  bool configure_circ_planner(std::string & reason) override
  {
    mgi_.setPlannerId("CIRC");
    mgi_.setStartStateToCurrentState();
    mgi_.clearPoseTargets();
    reason.clear();
    return true;
  }

  bool set_interim_path_constraint(
    const geometry_msgs::msg::Pose & auxiliary_point,
    std::string & reason) override
  {
    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::PositionConstraint pc;

    pc.header.frame_id = "world";
    pc.link_name = mgi_.getEndEffectorLink();
    if (pc.link_name.empty())
    {
      reason = "end effector link is empty; cannot set CIRC interim constraint";
      return false;
    }

    pc.weight = 1.0;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = { kInterimConstraintRadiusM };

    geometry_msgs::msg::Pose primitive_pose;
    primitive_pose.orientation.w = 1.0;
    primitive_pose.position = auxiliary_point.position;

    pc.constraint_region.primitives.push_back(primitive);
    pc.constraint_region.primitive_poses.push_back(primitive_pose);

    // Non-negotiable PILZ CIRC convention:
    // Use path_constraints name "interim" (on-arc waypoint), NOT "center".
    // PILZ CIRC with "center" always selects the shorter arc between start and goal,
    // which is not controllable and can produce unexpected obstacle-near motion.
    // Using "interim" forces passage through an explicit on-arc point and gives
    // deterministic control over which arc segment is executed.
    path_constraints.name = "interim";
    path_constraints.position_constraints.push_back(pc);

    mgi_.setPathConstraints(path_constraints);
    reason.clear();
    return true;
  }

  bool set_goal_pose(const geometry_msgs::msg::Pose & goal_pose, std::string & reason) override
  {
    if (!mgi_.setPoseTarget(goal_pose, mgi_.getEndEffectorLink()))
    {
      reason = "MoveGroupInterface rejected CIRC goal pose target";
      return false;
    }

    reason.clear();
    return true;
  }

  void clear_path_constraints() override
  {
    mgi_.clearPathConstraints();
  }

  CircScalingConfig scaling_config() const override
  {
    CircScalingConfig config;
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

  PrimitiveResult plan_with_pipeline(double velocity_scale, double acceleration_scale) override
  {
    MoveGroupInterface::Plan plan;
    const moveit::core::MoveItErrorCode plan_code = mgi_.plan(plan);
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      return make_result_failure(
        PrimitiveFailReason::PLANNING_TIMEOUT,
        "CIRC planning failed: " + moveit::core::error_code_to_string(plan_code));
    }

    auto current_state = mgi_.getCurrentState(1.0);
    if (!current_state)
    {
      return make_result_failure(
        PrimitiveFailReason::PLANNING_TIMEOUT,
        "CIRC planning failed: current robot state unavailable for TOTG");
    }

    robot_trajectory::RobotTrajectory robot_traj(mgi_.getRobotModel(), mgi_.getName());
    robot_traj.setRobotTrajectoryMsg(*current_state, plan.trajectory_);

    std::string post_reason;
    if (!post_processor_.apply_totg(robot_traj, velocity_scale, acceleration_scale, post_reason))
    {
      return make_result_failure(PrimitiveFailReason::TOTG_FAILED, "CIRC TOTG failed: " + post_reason);
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
        "CIRC downsample failed: " + post_reason);
    }

    std::string quality_reason;
    if (!quality_gate_.validate_plan(
          output_traj,
          motion_core::QualityGate::kFractionNotApplicable,
          quality_reason))
    {
      return make_result_failure(
        map_quality_failure(quality_reason),
        "CIRC quality gate rejected: " + quality_reason);
    }

    PrimitiveResult success;
    success.success = true;
    success.reason = PrimitiveFailReason::UNKNOWN;
    success.message = "CIRC plan ready";
    success.planning_time_sec = plan.planning_time_;
    success.trajectory_points = output_traj.points.size();
    return success;
  }

private:
  MoveGroupInterface & mgi_;
  rclcpp::Node::SharedPtr node_;

  motion_core::OrientationFilter orientation_filter_;
  motion_core::TrajectoryPostProcessor post_processor_;
  motion_core::QualityGate quality_gate_;
};
}  // namespace

PrimitiveResult PrimitiveCirc::execute(const CIRCGoal & goal, MoveGroupInterface & mgi)
{
  MoveGroupCircExecutionBackend backend(mgi);
  return execute(goal, backend);
}

PrimitiveResult PrimitiveCirc::execute(const ExecuteMotionGoal & goal, MoveGroupInterface & mgi)
{
  (void)goal;
  (void)mgi;

  return make_failure(
    PrimitiveFailReason::UNKNOWN,
    "ExecuteMotion goal for CIRC lacks auxiliary_point; use typed CIRCGoal API");
}

PrimitiveResult PrimitiveCirc::execute(const CIRCGoal & goal, CircExecutionBackend & backend)
{
  std::string reason;
  if (!backend.wait_for_servers(reason))
  {
    return make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, reason);
  }

  geometry_msgs::msg::Pose auxiliary_point = goal.auxiliary_point;
  if (!backend.normalize_pose(auxiliary_point, reason))
  {
    return make_failure(PrimitiveFailReason::INVALID_ORIENTATION, reason);
  }

  geometry_msgs::msg::Pose goal_pose = goal.goal_pose;
  if (!backend.normalize_pose(goal_pose, reason))
  {
    return make_failure(PrimitiveFailReason::INVALID_ORIENTATION, reason);
  }

  geometry_msgs::msg::Pose start_pose;
  if (!backend.get_current_pose_world(start_pose, reason))
  {
    return make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, reason);
  }

  if (position_distance(auxiliary_point, start_pose) <= kPositionEpsilonM)
  {
    return make_failure(
      PrimitiveFailReason::DEGENERATE_GEOMETRY,
      "CIRC degenerate geometry: auxiliary point coincides with start pose");
  }

  if (position_distance(auxiliary_point, goal_pose) <= kPositionEpsilonM)
  {
    return make_failure(
      PrimitiveFailReason::DEGENERATE_GEOMETRY,
      "CIRC degenerate geometry: auxiliary point coincides with goal pose");
  }

  if (!backend.configure_circ_planner(reason))
  {
    return make_failure(PrimitiveFailReason::UNKNOWN, reason);
  }

  if (!backend.set_interim_path_constraint(auxiliary_point, reason))
  {
    return make_failure(PrimitiveFailReason::UNKNOWN, reason);
  }

  PathConstraintGuard path_constraint_guard(backend);
  path_constraint_guard.enable();

  if (!backend.set_goal_pose(goal_pose, reason))
  {
    return make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, reason);
  }

  const CircScalingConfig scales = backend.scaling_config();
  const double velocity_scale = resolve_scale(goal.velocity_scale, scales.default_velocity, scales.velocity_cap);
  const double acceleration_scale = resolve_scale(
    goal.acceleration_scale,
    scales.default_acceleration,
    scales.acceleration_cap);

  PrimitiveResult result = backend.plan_with_pipeline(velocity_scale, acceleration_scale);
  if (!result.success &&
      (result.reason == PrimitiveFailReason::PLANNING_TIMEOUT || result.reason == PrimitiveFailReason::UNKNOWN) &&
      contains_degenerate_arc_hint(result.message))
  {
    result.reason = PrimitiveFailReason::DEGENERATE_GEOMETRY;
  }

  return result;
}
}  // namespace primitives
