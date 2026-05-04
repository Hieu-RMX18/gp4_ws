// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#include "primitives/primitive_home.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>

#include "motion_core/planner_router.hpp"
#include "motion_core/quality_gate.hpp"
#include "motion_core/trajectory_post_processor.hpp"

namespace primitives {
namespace {
constexpr double kWaitForServersTimeoutSec = 10.0;
constexpr double kDefaultVelocityCap = 0.3;
constexpr double kDefaultAccelerationCap = 0.2;
constexpr double kDefaultVelocityScale = 0.3;
constexpr double kDefaultAccelerationScale = 0.2;

double sanitize_positive_scale(double value, double fallback) {
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }

  return std::min(value, 1.0);
}

double read_scale_parameter(const rclcpp::Node::SharedPtr &node,
                            const std::string &name, double fallback) {
  if (!node) {
    return fallback;
  }

  if (!node->has_parameter(name)) {
    node->declare_parameter<double>(name, fallback);
  }

  double value = fallback;
  node->get_parameter(name, value);
  return sanitize_positive_scale(value, fallback);
}

PrimitiveFailReason
map_plan_failure(const moveit::core::MoveItErrorCode &code) {
  if (code == moveit::core::MoveItErrorCode::TIMED_OUT ||
      code == moveit::core::MoveItErrorCode::COMMUNICATION_FAILURE ||
      code == moveit::core::MoveItErrorCode::ROBOT_STATE_STALE) {
    return PrimitiveFailReason::PLANNING_TIMEOUT;
  }

  return PrimitiveFailReason::UNKNOWN;
}

PrimitiveFailReason map_quality_failure(const std::string &reason) {
  if (reason.find("wrist flip guard reject") != std::string::npos) {
    return PrimitiveFailReason::WRIST_FLIP_DETECTED;
  }
  if (reason.find("trajectory exceeds point limit") != std::string::npos) {
    return PrimitiveFailReason::TRAJECTORY_TOO_LONG;
  }

  return PrimitiveFailReason::QUALITY_GATE_REJECTED;
}

PrimitiveResult make_result_failure(PrimitiveFailReason reason,
                                    const std::string &message) {
  PrimitiveResult failure;
  failure.success = false;
  failure.reason = reason;
  failure.message = message;
  return failure;
}

class MoveGroupHomeExecutionBackend final : public HomeExecutionBackend {
public:
  explicit MoveGroupHomeExecutionBackend(MoveGroupInterface &mgi)
      : mgi_(mgi),
        node_(std::make_shared<rclcpp::Node>(
            "primitive_home_backend",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(true))),
        quality_gate_(
            motion_core::TrajectoryPostProcessor::kMaxTrajectoryPoints,
            motion_core::QualityGate::kMinimumCartesianFraction) {}

  bool wait_for_servers(std::string &reason) override {
    if (mgi_.getMoveGroupClient().wait_for_action_server(
            std::chrono::duration<double>(kWaitForServersTimeoutSec))) {
      reason.clear();
      return true;
    }

    reason = "MoveGroup action server unavailable after 10.0 seconds";
    return false;
  }

  bool set_named_target_home(std::string &reason) override {
    const std::string planner_id = planner_router_.route_planner("PTP", false);
    if (!planner_id.empty()) {
      mgi_.setPlannerId(planner_id);
    }

    mgi_.setStartStateToCurrentState();
    mgi_.clearPoseTargets();

    if (mgi_.setNamedTarget("home")) {
      reason.clear();
      return true;
    }

    reason = "named target 'home' is unavailable";
    return false;
  }

  HomeScalingConfig scaling_config() const override {
    HomeScalingConfig config;
    config.velocity_cap = read_scale_parameter(
        node_, "primitives.max_velocity_scaling_factor", kDefaultVelocityCap);
    config.acceleration_cap = read_scale_parameter(
        node_, "primitives.max_acceleration_scaling_factor",
        kDefaultAccelerationCap);
    config.default_velocity = read_scale_parameter(
        node_, "primitives.default_velocity_scaling_factor",
        kDefaultVelocityScale);
    config.default_acceleration = read_scale_parameter(
        node_, "primitives.default_acceleration_scaling_factor",
        kDefaultAccelerationScale);

    config.default_velocity =
        std::min(config.default_velocity, config.velocity_cap);
    config.default_acceleration =
        std::min(config.default_acceleration, config.acceleration_cap);
    return config;
  }

  PrimitiveResult plan_with_pipeline(double velocity_scale,
                                     double acceleration_scale) override {
    MoveGroupInterface::Plan plan;
    const moveit::core::MoveItErrorCode plan_code = mgi_.plan(plan);
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
      PrimitiveResult failure;
      failure.success = false;
      failure.reason = map_plan_failure(plan_code);
      failure.message = "HOME planning failed: " +
                        moveit::core::error_code_to_string(plan_code);
      return failure;
    }

    auto current_state = mgi_.getCurrentState(1.0);
    if (!current_state) {
      return make_result_failure(
          PrimitiveFailReason::PLANNING_TIMEOUT,
          "HOME planning failed: current robot state unavailable for TOTG");
    }

    robot_trajectory::RobotTrajectory robot_traj(mgi_.getRobotModel(),
                                                 mgi_.getName());
    robot_traj.setRobotTrajectoryMsg(*current_state, plan.trajectory_);

    std::string post_reason;
    if (!post_processor_.apply_totg(robot_traj, velocity_scale,
                                    acceleration_scale, post_reason)) {
      return make_result_failure(PrimitiveFailReason::TOTG_FAILED,
                                 "HOME TOTG failed: " + post_reason);
    }

    moveit_msgs::msg::RobotTrajectory postprocessed_msg;
    robot_traj.getRobotTrajectoryMsg(postprocessed_msg);
    trajectory_msgs::msg::JointTrajectory output_traj =
        postprocessed_msg.joint_trajectory;

    if (!post_processor_.downsample_to_max_points(
            output_traj,
            motion_core::TrajectoryPostProcessor::kMaxTrajectoryPoints,
            post_reason)) {
      return make_result_failure(PrimitiveFailReason::TRAJECTORY_TOO_LONG,
                                 "HOME downsample failed: " + post_reason);
    }

    std::string quality_reason;
    if (!quality_gate_.validate_plan(
            output_traj, motion_core::QualityGate::kFractionNotApplicable,
            "HOME", quality_reason)) {
      return make_result_failure(map_quality_failure(quality_reason),
                                 "HOME quality gate rejected: " +
                                     quality_reason);
    }

    PrimitiveResult success;
    success.success = true;
    success.reason = PrimitiveFailReason::UNKNOWN;
    success.message = "HOME plan ready";
    success.planning_time_sec = plan.planning_time_;
    success.trajectory_points = output_traj.points.size();
    return success;
  }

private:
  MoveGroupInterface &mgi_;
  rclcpp::Node::SharedPtr node_;
  motion_core::PlannerRouter planner_router_;
  motion_core::TrajectoryPostProcessor post_processor_;
  motion_core::QualityGate quality_gate_;
};

double resolve_home_scale(double requested, double fallback, double cap) {
  const double resolved = sanitize_positive_scale(requested, fallback);
  return std::min(resolved, cap);
}
} // namespace

PrimitiveResult PrimitiveHome::execute(MoveGroupInterface &mgi) {
  MoveGroupHomeExecutionBackend backend(mgi);
  return execute(backend);
}

PrimitiveResult PrimitiveHome::execute(const ExecuteMotionGoal &goal,
                                       MoveGroupInterface &mgi) {
  (void)goal;
  return execute(mgi);
}

PrimitiveResult PrimitiveHome::execute(HomeExecutionBackend &backend) {
  std::string reason;
  if (!backend.wait_for_servers(reason)) {
    return make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, reason);
  }

  if (!backend.set_named_target_home(reason)) {
    return make_failure(PrimitiveFailReason::NAMED_TARGET_NOT_FOUND, reason);
  }

  const HomeScalingConfig scales = backend.scaling_config();
  const double velocity_scale =
      resolve_home_scale(0.0, scales.default_velocity, scales.velocity_cap);
  const double acceleration_scale = resolve_home_scale(
      0.0, scales.default_acceleration, scales.acceleration_cap);

  return backend.plan_with_pipeline(velocity_scale, acceleration_scale);
}
} // namespace primitives
