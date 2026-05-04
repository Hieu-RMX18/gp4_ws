#include "motion_core/dispatch_trajectory_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>

namespace motion_core {
DispatchTrajectoryExecutor::DispatchTrajectoryExecutor(
    rclcpp::Logger logger,
    rclcpp_action::Client<DispatchTrajectory>::SharedPtr dispatch_client,
    std::string dispatch_action_name, const double dispatch_timeout_sec,
    const std::size_t safe_budget_points, QualityGate &quality_gate,
    TrajectoryPostProcessor &trajectory_post_processor)
    : logger_(logger), dispatch_client_(std::move(dispatch_client)),
      dispatch_action_name_(std::move(dispatch_action_name)),
      dispatch_timeout_sec_(dispatch_timeout_sec),
      safe_budget_points_(safe_budget_points), quality_gate_(quality_gate),
      trajectory_post_processor_(trajectory_post_processor) {}

bool DispatchTrajectoryExecutor::is_geometry_sensitive_primitive(
    const std::string &primitive) {
  return primitive == "LIN" || primitive == "CIRC" || primitive == "MOVE_REL" ||
         primitive == "CARTESIAN_PATH";
}

bool DispatchTrajectoryExecutor::is_finite_trajectory_vector(
    const std::vector<double> &values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool DispatchTrajectoryExecutor::validate_dispatch_segment_contract(
    const trajectory_msgs::msg::JointTrajectory &segment,
    std::string &reason) const {
  reason.clear();

  if (segment.points.empty()) {
    reason = "segment trajectory has no points";
    return false;
  }
  if (segment.joint_names.empty()) {
    reason = "segment trajectory has no joint names";
    return false;
  }

  const std::size_t dof = segment.joint_names.size();
  int64_t previous_time_ns = -1;
  for (std::size_t index = 0; index < segment.points.size(); ++index) {
    const auto &point = segment.points[index];
    const std::string point_label =
        "segment point[" + std::to_string(index) + "]";
    if (point.positions.size() != dof) {
      reason = point_label + " positions size does not match joint count";
      return false;
    }
    if (!point.velocities.empty() && point.velocities.size() != dof) {
      reason = point_label + " velocities size does not match joint count";
      return false;
    }
    if (!point.accelerations.empty() && point.accelerations.size() != dof) {
      reason = point_label + " accelerations size does not match joint count";
      return false;
    }
    if (!point.effort.empty() && point.effort.size() != dof) {
      reason = point_label + " effort size does not match joint count";
      return false;
    }
    if (!is_finite_trajectory_vector(point.positions) ||
        (!point.velocities.empty() &&
         !is_finite_trajectory_vector(point.velocities)) ||
        (!point.accelerations.empty() &&
         !is_finite_trajectory_vector(point.accelerations)) ||
        (!point.effort.empty() && !is_finite_trajectory_vector(point.effort))) {
      reason = point_label + " contains NaN or Inf";
      return false;
    }

    const int64_t current_time_ns =
        rclcpp::Duration(point.time_from_start).nanoseconds();
    if (current_time_ns < 0) {
      reason = point_label + " has negative time_from_start";
      return false;
    }
    if (previous_time_ns >= 0 && current_time_ns <= previous_time_ns) {
      reason = "segment time_from_start must be strictly monotonic";
      return false;
    }
    previous_time_ns = current_time_ns;
  }

  if (previous_time_ns <= 0) {
    reason = "segment total duration must be greater than zero";
    return false;
  }

  reason.clear();
  return true;
}

bool DispatchTrajectoryExecutor::validate_mitigated_segments_contract(
    const trajectory_msgs::msg::JointTrajectory &original,
    const std::vector<trajectory_msgs::msg::JointTrajectory> &segments,
    std::string &reason) const {
  reason.clear();
  if (original.points.empty() || segments.empty()) {
    reason = "mitigation produced no dispatchable trajectory segment";
    return false;
  }

  if (segments.front().points.front().positions !=
      original.points.front().positions) {
    reason = "post-processing changed trajectory start point";
    return false;
  }
  if (segments.back().points.back().positions !=
      original.points.back().positions) {
    reason = "post-processing changed trajectory end point";
    return false;
  }

  for (std::size_t index = 0; index < segments.size(); ++index) {
    std::string segment_reason;
    if (!validate_dispatch_segment_contract(segments[index], segment_reason)) {
      reason = "segment " + std::to_string(index + 1U) +
               " violates dispatch contract: " + segment_reason;
      return false;
    }

    if (index > 0U && segments[index].points.front().positions !=
                          segments[index - 1U].points.back().positions) {
      reason = "segment boundary continuity mismatch between segment " +
               std::to_string(index) + " and segment " +
               std::to_string(index + 1U);
      return false;
    }
  }

  reason.clear();
  return true;
}

bool DispatchTrajectoryExecutor::split_trajectory_for_dispatch(
    const trajectory_msgs::msg::JointTrajectory &input_trajectory,
    const std::size_t max_points_per_segment,
    std::vector<trajectory_msgs::msg::JointTrajectory> &output_segments,
    std::string &reason) const {
  reason.clear();
  output_segments.clear();

  if (input_trajectory.points.empty()) {
    reason = "cannot split an empty trajectory";
    return false;
  }

  if (max_points_per_segment < 2U) {
    reason = "split policy requires max_points_per_segment >= 2";
    return false;
  }

  if (input_trajectory.points.size() <= max_points_per_segment) {
    output_segments.push_back(input_trajectory);
    return true;
  }

  const std::size_t point_count = input_trajectory.points.size();
  std::size_t start_index = 0U;

  while (start_index < point_count) {
    const std::size_t end_index =
        std::min(start_index + max_points_per_segment - 1U, point_count - 1U);

    trajectory_msgs::msg::JointTrajectory segment;
    segment.header = input_trajectory.header;
    segment.joint_names = input_trajectory.joint_names;
    segment.points.reserve((end_index - start_index) + 1U);
    for (std::size_t point_index = start_index; point_index <= end_index;
         ++point_index) {
      segment.points.push_back(input_trajectory.points[point_index]);
    }

    if (segment.points.size() < 2U) {
      reason = "split policy produced a segment with fewer than 2 points";
      return false;
    }

    const rclcpp::Duration segment_start_time(
        segment.points.front().time_from_start);
    for (auto &point : segment.points) {
      const rclcpp::Duration point_time(point.time_from_start);
      if (point_time < segment_start_time) {
        reason = "split policy produced non-monotonic segment timestamps";
        return false;
      }
      point.time_from_start = point_time - segment_start_time;
    }

    output_segments.push_back(std::move(segment));

    if (end_index >= point_count - 1U) {
      break;
    }

    // Segment boundary overlap keeps endpoint continuity while respecting
    // "split and dispatch sequentially" with a full stop between segments.
    start_index = end_index;
  }

  return true;
}

DispatchTrajectoryExecutor::Result
DispatchTrajectoryExecutor::dispatch_to_hw_adapter(
    const trajectory_msgs::msg::JointTrajectory &trajectory,
    const DispatchMetadata &metadata, const InterruptReasonFn &interrupt_reason,
    const UpdatePhaseFn &update_phase) const {
  constexpr auto kPollPeriod = std::chrono::milliseconds(50);
  Result result;

  if (!dispatch_client_) {
    result.reason = "DispatchTrajectory client not initialized";
    return result;
  }

  if (!dispatch_client_->wait_for_action_server(std::chrono::seconds(5))) {
    result.reason = "DispatchTrajectory action server unavailable at " +
                    dispatch_action_name_;
    return result;
  }

  const auto query_interrupt =
      [&interrupt_reason](const std::string &stage) -> std::string {
    if (!interrupt_reason) {
      return {};
    }
    return interrupt_reason(stage);
  };

  const std::string pre_dispatch_interrupt = query_interrupt("dispatch_setup");
  if (!pre_dispatch_interrupt.empty()) {
    result.status = Status::kCanceled;
    result.reason = pre_dispatch_interrupt;
    return result;
  }

  DispatchTrajectory::Goal goal;
  goal.trajectory = trajectory;
  goal.timeout_sec = dispatch_timeout_sec_;
  goal.command_id = metadata.command_id;
  goal.primitive = metadata.primitive;
  goal.planner_id = metadata.planner_id;
  goal.source_joint_state_stamp = metadata.source_joint_state_stamp;
  goal.expected_start_positions = trajectory.points.empty()
                                      ? std::vector<double>{}
                                      : trajectory.points.front().positions;
  goal.enforce_start_state_match = metadata.enforce_start_state_match;

  RCLCPP_INFO(
      logger_,
      "execute_motion dispatch_start target=%s points=%zu timeout=%.2fs",
      dispatch_action_name_.c_str(), trajectory.points.size(),
      dispatch_timeout_sec_);

  auto send_goal_future = dispatch_client_->async_send_goal(goal);
  const auto send_goal_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (send_goal_future.wait_for(kPollPeriod) != std::future_status::ready) {
    const std::string interrupt = query_interrupt("dispatch_goal_send");
    if (!interrupt.empty()) {
      result.status = Status::kCanceled;
      result.reason = interrupt;
      return result;
    }

    if (std::chrono::steady_clock::now() >= send_goal_deadline) {
      result.reason =
          "timed out sending trajectory to " + dispatch_action_name_;
      return result;
    }
  }

  auto dispatch_goal_handle = send_goal_future.get();
  if (!dispatch_goal_handle) {
    result.reason = "hw_adapter rejected trajectory dispatch goal (dispatch "
                    "already in progress or unavailable)";
    return result;
  }

  if (update_phase) {
    update_phase(ExecutionPhase::kExecuting, "dispatch accepted by hw_adapter");
  }

  auto result_future = dispatch_client_->async_get_result(dispatch_goal_handle);
  const auto result_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(std::max(dispatch_timeout_sec_, 10.0)));
  while (result_future.wait_for(kPollPeriod) != std::future_status::ready) {
    const std::string interrupt = query_interrupt("dispatch_wait");
    if (!interrupt.empty()) {
      dispatch_client_->async_cancel_goal(dispatch_goal_handle);
      result.status = Status::kCanceled;
      result.reason = interrupt;
      return result;
    }

    if (std::chrono::steady_clock::now() >= result_deadline) {
      dispatch_client_->async_cancel_goal(dispatch_goal_handle);
      result.reason =
          "timed out waiting for dispatch result from " + dispatch_action_name_;
      return result;
    }
  }

  const auto wrapped_result = result_future.get();
  if (!wrapped_result.result) {
    result.reason = "hw_adapter returned no dispatch result";
    return result;
  }

  if (!wrapped_result.result->success) {
    std::ostringstream stream;
    stream << (wrapped_result.result->message.empty()
                   ? "hw_adapter execution failed"
                   : wrapped_result.result->message);
    if (!wrapped_result.result->failure_stage.empty() &&
        wrapped_result.result->failure_stage != "none") {
      stream << " [stage=" << wrapped_result.result->failure_stage << "]";
    }
    if (!wrapped_result.result->controller_error_name.empty() &&
        wrapped_result.result->controller_error_name != "SUCCESSFUL") {
      stream << " [controller_error="
             << wrapped_result.result->controller_error_name << "("
             << wrapped_result.result->controller_error_code << ")";
      if (!wrapped_result.result->controller_error_string.empty()) {
        stream << ": " << wrapped_result.result->controller_error_string;
      }
      stream << "]";
    }
    result.reason = stream.str();
    const std::string interrupt = query_interrupt("dispatch_result");
    if (!interrupt.empty()) {
      result.status = Status::kCanceled;
      result.reason = interrupt + " (" + result.reason + ")";
    }
    return result;
  }

  std::ostringstream note;
  note << "dispatched_via=" << dispatch_action_name_
       << ", hw_execution_time=" << wrapped_result.result->execution_time_sec
       << "s"
       << ", failure_stage=" << wrapped_result.result->failure_stage;
  result.status = Status::kSuccess;
  result.note = note.str();
  RCLCPP_INFO(logger_,
              "execute_motion dispatch_end target=%s success=true detail=%s",
              dispatch_action_name_.c_str(), result.note.c_str());
  return result;
}

DispatchTrajectoryExecutor::Result
DispatchTrajectoryExecutor::apply_budget_quality_and_dispatch(
    trajectory_msgs::msg::JointTrajectory trajectory,
    const std::string &primitive, const DispatchMetadata &dispatch_metadata,
    const double cartesian_fraction, const InterruptReasonFn &interrupt_reason,
    const PublishFeedbackFn &publish_feedback,
    const UpdatePhaseFn &update_phase, std::size_t &reported_point_count,
    std::size_t &reported_segment_count) {
  Result result;
  std::ostringstream note_stream;
  const trajectory_msgs::msg::JointTrajectory original_trajectory = trajectory;
  const std::size_t original_point_count = trajectory.points.size();
  reported_point_count = original_point_count;
  reported_segment_count = 0U;

  std::vector<trajectory_msgs::msg::JointTrajectory> segments;

  if (original_point_count > safe_budget_points_) {
    if (is_geometry_sensitive_primitive(primitive)) {
      std::string split_reason;
      if (!split_trajectory_for_dispatch(trajectory, safe_budget_points_,
                                         segments, split_reason)) {
        result.reason = "split-and-dispatch mitigation failed: " + split_reason;
        return result;
      }

      RCLCPP_WARN(logger_,
                  "primitive=%s trajectory points=%zu exceed safe budget=%zu; "
                  "using split-and-dispatch mitigation (segments=%zu, "
                  "full-stop between segments).",
                  primitive.c_str(), original_point_count, safe_budget_points_,
                  segments.size());

      note_stream << "budget_mitigation=split_sequential"
                  << ", original_points=" << original_point_count
                  << ", safe_budget=" << safe_budget_points_
                  << ", segments=" << segments.size();
    } else {
      std::string downsample_reason;
      if (!trajectory_post_processor_.downsample_to_max_points(
              trajectory, safe_budget_points_, downsample_reason)) {
        result.reason = "downsample mitigation failed: " + downsample_reason;
        return result;
      }

      RCLCPP_WARN(logger_,
                  "primitive=%s trajectory points=%zu exceed safe budget=%zu; "
                  "using downsample mitigation (mitigated_points=%zu).",
                  primitive.c_str(), original_point_count, safe_budget_points_,
                  trajectory.points.size());

      segments.push_back(trajectory);
      reported_point_count = trajectory.points.size();
      note_stream << "budget_mitigation=downsample"
                  << ", original_points=" << original_point_count
                  << ", mitigated_points=" << trajectory.points.size()
                  << ", safe_budget=" << safe_budget_points_;
    }
  } else {
    segments.push_back(trajectory);
    note_stream << "budget_mitigation=none"
                << ", points=" << original_point_count;
  }

  std::string contract_reason;
  if (!validate_mitigated_segments_contract(original_trajectory, segments,
                                            contract_reason)) {
    result.reason = "post-processing dispatch contract validation failed: " +
                    contract_reason;
    return result;
  }

  reported_segment_count = segments.size();

  for (std::size_t index = 0; index < segments.size(); ++index) {
    const double fraction_for_segment =
        (index == 0U) ? cartesian_fraction
                      : QualityGate::kFractionNotApplicable;
    std::string quality_reason;
    if (!quality_gate_.validate_plan(segments[index], fraction_for_segment,
                                     primitive, quality_reason)) {
      std::ostringstream quality_stream;
      quality_stream << "quality gate failed for dispatch segment "
                     << (index + 1U) << "/" << segments.size() << ": "
                     << quality_reason;
      result.reason = quality_stream.str();
      return result;
    }
  }

  if (update_phase) {
    update_phase(ExecutionPhase::kDispatchWait,
                 "trajectory dispatch requested");
  }

  for (std::size_t index = 0; index < segments.size(); ++index) {
    if (interrupt_reason) {
      const std::string pre_dispatch_interrupt = interrupt_reason(
          "pre_dispatch_segment_" + std::to_string(index + 1U));
      if (!pre_dispatch_interrupt.empty()) {
        result.status = Status::kCanceled;
        result.reason = pre_dispatch_interrupt;
        return result;
      }
    }

    if (update_phase) {
      std::ostringstream dispatch_detail;
      dispatch_detail << "dispatching segment " << (index + 1U) << "/"
                      << segments.size();
      update_phase(ExecutionPhase::kDispatchWait, dispatch_detail.str());
    }

    if (publish_feedback) {
      const double progress =
          std::min(0.75 + (0.18 * static_cast<double>(index + 1U) /
                           static_cast<double>(segments.size())),
                   0.94);
      publish_feedback(progress, "trajectory_dispatch_requested");
    }

    const auto dispatch_result = dispatch_to_hw_adapter(
        segments[index], dispatch_metadata, interrupt_reason, update_phase);
    if (dispatch_result.status == Status::kCanceled) {
      result.status = Status::kCanceled;
      std::ostringstream canceled_stream;
      canceled_stream << "dispatch segment " << (index + 1U) << "/"
                      << segments.size()
                      << " canceled: " << dispatch_result.reason;
      result.reason = canceled_stream.str();
      return result;
    }
    if (dispatch_result.status != Status::kSuccess) {
      std::ostringstream failure_stream;
      failure_stream << "dispatch segment " << (index + 1U) << "/"
                     << segments.size()
                     << " failed: " << dispatch_result.reason;
      result.reason = failure_stream.str();
      return result;
    }

    if (!dispatch_result.note.empty()) {
      note_stream << ", segment_" << (index + 1U) << "_detail={"
                  << dispatch_result.note << "}";
    }
  }

  result.status = Status::kSuccess;
  result.note = note_stream.str();
  return result;
}
} // namespace motion_core
