#include "motion_core/motion_primitive_executor.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

#include "motion_core/trajectory_post_processor.hpp"

namespace motion_core {
MotionPrimitiveExecutor::MotionPrimitiveExecutor(Dependencies dependencies)
    : dependencies_(std::move(dependencies)) {}

MotionPrimitiveExecutor::Result MotionPrimitiveExecutor::execute(
    const std::shared_ptr<const ExecuteMotion::Goal> &goal,
    const std::string &goal_id, const std::string &primitive,
    const std::uint64_t goal_sequence) const {
  Result result;

  if (dependencies_.update_phase) {
    dependencies_.update_phase(ExecutionPhase::kPlanning, "ensure_move_group");
  }

  std::string move_group_reason;
  if (!dependencies_.ensure_move_group ||
      !dependencies_.ensure_move_group(move_group_reason)) {
    result.status = Status::kAborted;
    result.message =
        move_group_reason.empty() ? "MoveGroup unavailable" : move_group_reason;
    return result;
  }

  std::string scene_reason;
  if (!dependencies_.ensure_scene_ready ||
      !dependencies_.ensure_scene_ready(scene_reason)) {
    result.status = Status::kAborted;
    result.message =
        scene_reason.empty() ? "planning scene unavailable" : scene_reason;
    return result;
  }

  const std::string effective_primitive =
      (primitive == "MOVE_JOINTS") ? "PTP" : primitive;

  const double velocity_scale =
      (goal->velocity_scale > 0.0)
          ? goal->velocity_scale
          : TrajectoryPostProcessor::kDefaultVelocityScaling;
  const double acceleration_scale =
      (goal->acceleration_scale > 0.0)
          ? goal->acceleration_scale
          : TrajectoryPostProcessor::kDefaultAccelerationScaling;

  auto move_group = dependencies_.move_group_provider
                        ? dependencies_.move_group_provider()
                        : nullptr;
  if (!move_group) {
    result.status = Status::kAborted;
    result.message = "MoveGroup unavailable";
    return result;
  }

  moveit::core::RobotState current_robot_state(move_group->getRobotModel());
  builtin_interfaces::msg::Time source_joint_state_stamp;
  std::string current_state_reason;
  if (!dependencies_.build_current_robot_state ||
      !dependencies_.build_current_robot_state(current_robot_state,
                                               current_state_reason,
                                               &source_joint_state_stamp)) {
    result.status = Status::kAborted;
    result.message =
        "failed to read current joint state: " + current_state_reason;
    return result;
  }

  const auto *joint_model_group =
      current_robot_state.getJointModelGroup(dependencies_.planning_group);
  if (!joint_model_group) {
    result.status = Status::kAborted;
    result.message =
        "planning group '" + dependencies_.planning_group + "' not found";
    return result;
  }

  std::vector<double> current_joint_positions;
  current_robot_state.copyJointGroupPositions(joint_model_group,
                                              current_joint_positions);
  const auto &active_joint_models = joint_model_group->getActiveJointModels();
  const auto &active_joint_names =
      joint_model_group->getActiveJointModelNames();

  if (dependencies_.update_phase) {
    dependencies_.update_phase(ExecutionPhase::kPlanning,
                               "planning_request_prepared");
  }
  if (dependencies_.publish_feedback) {
    dependencies_.publish_feedback(0.2, "planning_request_prepared");
  }

  PrimitiveRouterDispatch::PlanningRequest planning_request(
      current_robot_state);
  planning_request.goal = goal;
  planning_request.primitive = primitive;
  planning_request.effective_primitive = effective_primitive;
  planning_request.goal_sequence = goal_sequence;
  planning_request.velocity_scale = velocity_scale;
  planning_request.acceleration_scale = acceleration_scale;
  planning_request.current_joint_positions = current_joint_positions;
  planning_request.active_joint_models = active_joint_models;
  planning_request.active_joint_names = active_joint_names;
  planning_request.plan_with_interruption =
      dependencies_.plan_with_interruption;
  planning_request.interrupt_reason = dependencies_.interrupt_reason;
  if (goal->extended_mode) {
    planning_request.joint_position_guard_mode =
        JointPositionGuard::Mode::Extended;
  }

  const auto planning_result =
      dependencies_.primitive_router_dispatch.plan_for_primitive(
          planning_request);
  if (planning_result.status ==
      PrimitiveRouterDispatch::PlanningStatus::kCanceled) {
    result.status = Status::kCanceled;
    result.message =
        planning_result.is_move_joint
            ? ("MOVE_JOINT planning canceled: " + planning_result.reason)
            : ("planning canceled: " + planning_result.reason);
    return result;
  }
  if (planning_result.status !=
      PrimitiveRouterDispatch::PlanningStatus::kSuccess) {
    result.status = Status::kAborted;
    result.message = planning_result.reason;
    return result;
  }

  if (dependencies_.publish_feedback) {
    dependencies_.publish_feedback(0.55, "post_processing");
  }

  DispatchTrajectoryExecutor::DispatchMetadata dispatch_metadata;
  dispatch_metadata.command_id = goal_id;
  dispatch_metadata.primitive = planning_result.dispatch_primitive;
  dispatch_metadata.planner_id = planning_result.planner_id;
  dispatch_metadata.source_joint_state_stamp = source_joint_state_stamp;
  dispatch_metadata.enforce_start_state_match = true;
  dispatch_metadata.extended_mode = goal->extended_mode;

  std::size_t dispatched_point_count = 0U;
  std::size_t dispatched_segment_count = 0U;
  const auto dispatch_result =
      dependencies_.dispatch_executor.apply_budget_quality_and_dispatch(
          planning_result.trajectory, planning_result.dispatch_primitive,
          dispatch_metadata, planning_result.cartesian_fraction,
          dependencies_.interrupt_reason, dependencies_.publish_feedback,
          dependencies_.update_phase, dispatched_point_count,
          dispatched_segment_count, planning_request.joint_position_guard_mode);

  if (dispatch_result.status == DispatchTrajectoryExecutor::Status::kCanceled) {
    result.status = Status::kCanceled;
    result.message =
        planning_result.is_move_joint
            ? ("MOVE_JOINT dispatch canceled: " + dispatch_result.reason)
            : ("trajectory dispatch canceled: " + dispatch_result.reason);
    return result;
  }
  if (dispatch_result.status != DispatchTrajectoryExecutor::Status::kSuccess) {
    result.status = Status::kAborted;
    result.message =
        planning_result.is_move_joint
            ? ("MOVE_JOINT dispatch failed: " + dispatch_result.reason)
            : ("trajectory dispatch failed: " + dispatch_result.reason);
    return result;
  }

  if (dependencies_.publish_feedback) {
    dependencies_.publish_feedback(0.95, "trajectory_execution_complete");
  }

  std::ostringstream message;
  if (planning_result.is_move_joint) {
    message << "MOVE_JOINT success: joint[" << planning_result.move_joint_index
            << "]=" << planning_result.move_joint_target_angle
            << " rad, points=" << dispatched_point_count
            << ", segments=" << dispatched_segment_count;
  } else {
    message << "execution success; primitive=" << primitive
            << ", planner_id=" << planning_result.planner_id
            << ", points=" << dispatched_point_count
            << ", segments=" << dispatched_segment_count;
    if (planning_result.cartesian_fraction >= 0.0) {
      message << ", cartesian_fraction=" << planning_result.cartesian_fraction;
    }
  }
  if (!planning_result.time_parameterization_note.empty()) {
    message << ", " << planning_result.time_parameterization_note;
  }
  if (!planning_result.ruckig_reason.empty()) {
    message << ", ruckig_status=" << planning_result.ruckig_reason;
  }
  if (!dispatch_result.note.empty()) {
    message << ", " << dispatch_result.note;
  }

  result.status = Status::kSucceeded;
  result.message = message.str();
  return result;
}
} // namespace motion_core
