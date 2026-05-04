// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#include "primitives/primitive_ptp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motion_core/ik_selector.hpp"
#include "motion_core/orientation_filter.hpp"
#include "motion_core/planner_router.hpp"
#include "motion_core/quality_gate.hpp"
#include "motion_core/seed_manager.hpp"
#include "motion_core/trajectory_post_processor.hpp"

namespace primitives {
namespace {
constexpr double kWaitForServersTimeoutSec = 10.0;
constexpr std::size_t kExpectedJointCount = 6U;
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
  if (reason.find("cartesian fraction below minimum threshold") !=
      std::string::npos) {
    return PrimitiveFailReason::CARTESIAN_FRACTION_LOW;
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

bool has_pose_target(const geometry_msgs::msg::Pose &pose) {
  return pose.position.x != 0.0 || pose.position.y != 0.0 ||
         pose.position.z != 0.0 || pose.orientation.x != 0.0 ||
         pose.orientation.y != 0.0 || pose.orientation.z != 0.0 ||
         pose.orientation.w != 0.0;
}

class MoveGroupPtpExecutionBackend final : public PtpExecutionBackend {
public:
  explicit MoveGroupPtpExecutionBackend(MoveGroupInterface &mgi)
      : mgi_(mgi),
        node_(std::make_shared<rclcpp::Node>(
            "primitive_ptp_backend",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(true))),
        quality_gate_(
            motion_core::TrajectoryPostProcessor::kMaxTrajectoryPoints,
            motion_core::QualityGate::kMinimumCartesianFraction) {
    seed_manager_ = std::make_unique<motion_core::SeedManager>(*node_);

    const auto non_owning_mgi = std::shared_ptr<MoveGroupInterface>(
        &mgi_, [](MoveGroupInterface * /*unused*/) {});
    ik_selector_.set_move_group(non_owning_mgi);
    ik_selector_.set_planning_group(mgi_.getName());
  }

  bool wait_for_servers(std::string &reason) override {
    if (mgi_.getMoveGroupClient().wait_for_action_server(
            std::chrono::duration<double>(kWaitForServersTimeoutSec))) {
      reason.clear();
      return true;
    }

    reason = "MoveGroup action server unavailable after 10.0 seconds";
    return false;
  }

  bool configure_ptp_planner(std::string &reason) override {
    const std::string planner_id = planner_router_.route_planner("PTP", false);
    if (planner_id.empty()) {
      reason = "planner router did not resolve PTP planner";
      return false;
    }

    mgi_.setPlannerId(planner_id);
    mgi_.setStartStateToCurrentState();
    mgi_.clearPoseTargets();
    reason.clear();
    return true;
  }

  bool set_joint_target(const std::vector<double> &target,
                        std::string &reason) override {
    if (!mgi_.setJointValueTarget(target)) {
      reason = "MoveGroupInterface rejected joint target";
      return false;
    }

    reason.clear();
    return true;
  }

  bool normalize_pose(geometry_msgs::msg::Pose &pose,
                      std::string &reason) override {
    if (!orientation_filter_.normalize_and_validate(pose, reason)) {
      return false;
    }

    reason.clear();
    return true;
  }

  bool solve_pose_to_joints(const geometry_msgs::msg::Pose &pose,
                            std::vector<double> &joint_solution,
                            std::string &reason) override {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    const auto spin_start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::duration<double>>(
               std::chrono::steady_clock::now() - spin_start)
               .count() < 0.2) {
      executor.spin_some();
    }
    executor.remove_node(node_);

    std::vector<double> seed_state;
    if (!seed_manager_->get_seed_state(seed_state)) {
      reason = "failed to obtain IK seed from /yaskawa/joint_states";
      return false;
    }

    if (!ik_selector_.solve_ik(pose, seed_state, joint_solution, reason)) {
      return false;
    }

    reason.clear();
    return true;
  }

  PtpScalingConfig scaling_config() const override {
    PtpScalingConfig config;
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
      failure.message = "PTP planning failed: " +
                        moveit::core::error_code_to_string(plan_code);
      return failure;
    }

    auto current_state = mgi_.getCurrentState(1.0);
    if (!current_state) {
      return make_result_failure(
          PrimitiveFailReason::PLANNING_TIMEOUT,
          "PTP planning failed: current robot state unavailable for TOTG");
    }

    robot_trajectory::RobotTrajectory robot_traj(mgi_.getRobotModel(),
                                                 mgi_.getName());
    robot_traj.setRobotTrajectoryMsg(*current_state, plan.trajectory_);

    std::string post_reason;
    if (!post_processor_.apply_totg(robot_traj, velocity_scale,
                                    acceleration_scale, post_reason)) {
      return make_result_failure(PrimitiveFailReason::TOTG_FAILED,
                                 "PTP TOTG failed: " + post_reason);
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
                                 "PTP downsample failed: " + post_reason);
    }

    std::string quality_reason;
    if (!quality_gate_.validate_plan(
            output_traj, motion_core::QualityGate::kFractionNotApplicable,
            "PTP", quality_reason)) {
      return make_result_failure(map_quality_failure(quality_reason),
                                 "PTP quality gate rejected: " +
                                     quality_reason);
    }

    PrimitiveResult success;
    success.success = true;
    success.reason = PrimitiveFailReason::UNKNOWN;
    success.message = "PTP plan ready";
    success.planning_time_sec = plan.planning_time_;
    success.trajectory_points = output_traj.points.size();
    return success;
  }

private:
  MoveGroupInterface &mgi_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<motion_core::SeedManager> seed_manager_;

  motion_core::PlannerRouter planner_router_;
  motion_core::IkSelector ik_selector_;
  motion_core::OrientationFilter orientation_filter_;
  motion_core::TrajectoryPostProcessor post_processor_;
  motion_core::QualityGate quality_gate_;
};

double resolve_scale(double requested, double fallback, double cap) {
  const double resolved = sanitize_positive_scale(requested, fallback);
  return std::min(resolved, cap);
}
} // namespace

PrimitiveResult PrimitivePtp::execute(const PTPGoal &goal,
                                      MoveGroupInterface &mgi) {
  MoveGroupPtpExecutionBackend backend(mgi);
  return execute(goal, backend);
}

PrimitiveResult PrimitivePtp::execute(const ExecuteMotionGoal &goal,
                                      MoveGroupInterface &mgi) {
  PTPGoal ptp_goal;
  ptp_goal.joint_target = goal.joint_target;
  ptp_goal.pose_target = goal.target_pose;
  ptp_goal.velocity_scale = goal.velocity_scale;
  ptp_goal.acceleration_scale = goal.acceleration_scale;
  return execute(ptp_goal, mgi);
}

PrimitiveResult PrimitivePtp::execute(const PTPGoal &goal,
                                      PtpExecutionBackend &backend) {
  std::string reason;
  if (!backend.wait_for_servers(reason)) {
    return make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, reason);
  }

  if (!backend.configure_ptp_planner(reason)) {
    return make_failure(PrimitiveFailReason::UNKNOWN, reason);
  }

  const bool has_joint_target = !goal.joint_target.empty();
  const bool has_pose = has_pose_target(goal.pose_target);

  if (has_joint_target) {
    if (goal.joint_target.size() != kExpectedJointCount) {
      return make_failure(PrimitiveFailReason::JOINT_COUNT_MISMATCH,
                          "PTP joint_target must have exactly 6 values");
    }

    if (!backend.set_joint_target(goal.joint_target, reason)) {
      return make_failure(PrimitiveFailReason::UNKNOWN, reason);
    }
  } else if (has_pose) {
    geometry_msgs::msg::Pose normalized_pose = goal.pose_target;
    if (!backend.normalize_pose(normalized_pose, reason)) {
      return make_failure(PrimitiveFailReason::INVALID_ORIENTATION, reason);
    }

    std::vector<double> ik_solution;
    if (!backend.solve_pose_to_joints(normalized_pose, ik_solution, reason)) {
      return make_failure(PrimitiveFailReason::IK_FAILED, reason);
    }

    if (ik_solution.size() != kExpectedJointCount) {
      return make_failure(PrimitiveFailReason::JOINT_COUNT_MISMATCH,
                          "IK solution must contain exactly 6 joints for GP4");
    }

    if (!backend.set_joint_target(ik_solution, reason)) {
      return make_failure(PrimitiveFailReason::IK_FAILED, reason);
    }
  } else {
    return make_failure(PrimitiveFailReason::UNKNOWN, "no target specified");
  }

  const PtpScalingConfig scales = backend.scaling_config();
  const double velocity_scale = resolve_scale(
      goal.velocity_scale, scales.default_velocity, scales.velocity_cap);
  const double acceleration_scale =
      resolve_scale(goal.acceleration_scale, scales.default_acceleration,
                    scales.acceleration_cap);

  return backend.plan_with_pipeline(velocity_scale, acceleration_scale);
}
} // namespace primitives
