// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/trajectory_executor.hpp"
#include "motion_core/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace hw_adapter {
namespace {
std::string timeout_reason(const std::string &contract_type,
                           const std::string &contract_name,
                           const std::chrono::milliseconds timeout) {
  std::ostringstream stream;
  stream << "Timed out after " << timeout.count() << " ms waiting for "
         << contract_type << " '" << contract_name << "'";
  return stream.str();
}

bool is_zero_stamp(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec == 0 && stamp.nanosec == 0U;
}
} // namespace

TrajectoryExecutor::TrajectoryExecutor(
    rclcpp::Node &node, std::string action_name,
    RuntimeSnapshotProvider runtime_snapshot_provider,
    std::chrono::milliseconds default_timeout,
    std::vector<std::string> canonical_joint_names,
    std::chrono::milliseconds trajectory_header_max_age,
    const double start_state_max_abs_delta_rad,
    const double start_state_max_l2_delta_rad,
    motion_core::JointPositionGuard joint_position_guard)
    : logger_(node.get_logger()), action_name_(std::move(action_name)),
      runtime_snapshot_provider_(std::move(runtime_snapshot_provider)),
      default_timeout_(default_timeout),
      canonical_joint_names_(std::move(canonical_joint_names)),
      trajectory_header_max_age_(trajectory_header_max_age),
      start_state_max_abs_delta_rad_(start_state_max_abs_delta_rad),
      start_state_max_l2_delta_rad_(start_state_max_l2_delta_rad),
      joint_position_guard_(std::move(joint_position_guard)) {
  if (canonical_joint_names_.empty()) {
    canonical_joint_names_ = {"joint_1_s", "joint_2_l", "joint_3_u",
                              "joint_4_r", "joint_5_b", "joint_6_t"};
  }
  if (trajectory_header_max_age_.count() <= 0) {
    trajectory_header_max_age_ = std::chrono::milliseconds(200);
  }

  rclcpp::NodeOptions client_node_options;
  client_node_options.context(node.get_node_base_interface()->get_context());
  client_node_ = std::make_shared<rclcpp::Node>(
      std::string(node.get_name()) + "_trajectory_executor",
      node.get_namespace(), client_node_options);

  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      client_node_, action_name_);
}

const std::string &TrajectoryExecutor::action_name() const {
  return action_name_;
}

bool TrajectoryExecutor::wait_for_server(
    std::chrono::milliseconds timeout) const {
  return action_client_->wait_for_action_server(timeout);
}

bool TrajectoryExecutor::validate_trajectory_request(
    const trajectory_msgs::msg::JointTrajectory &traj,
    std::string &reason) const {
  return validate_trajectory(traj, reason);
}

bool TrajectoryExecutor::execute(
    const trajectory_msgs::msg::JointTrajectory &traj, std::string &reason) {
  return execute_with_timeout(
      traj, std::chrono::duration<double>(default_timeout_).count(), reason);
}

bool TrajectoryExecutor::execute_with_timeout(
    const trajectory_msgs::msg::JointTrajectory &traj, const double timeout_sec,
    std::string &reason) {
  if (timeout_sec <= 0.0) {
    reason = "timeout_sec must be greater than zero";
    return false;
  }

  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(timeout_sec));
  TrajectoryExecutionRequest request;
  request.trajectory = traj;
  request.result_timeout = timeout;
  const auto result = execute_blocking(request);
  reason = result.message;
  return result.success;
}

TrajectoryExecutionResult TrajectoryExecutor::execute_blocking(
    const TrajectoryExecutionRequest &request) {
  TrajectoryExecutionResult result;
  auto finalize = [this, &request, &result]() {
    log_dispatch_diagnostics(request, result);
    return result;
  };

  if (request.result_timeout.count() <= 0) {
    result.failure_stage = "invalid_request";
    result.message = "result_timeout must be greater than zero";
    return finalize();
  }

  if (!validate_trajectory(request.trajectory, result.message)) {
    result.failure_stage = "trajectory_validation";
    return finalize();
  }

  if (!runtime_snapshot_provider_) {
    result.failure_stage = "runtime_preflight";
    result.message = "runtime snapshot provider is not configured";
    return finalize();
  }

  const auto preflight_snapshot = runtime_snapshot_provider_();
  result.preflight_snapshot_available = true;
  result.preflight_joint_state_age = preflight_snapshot.joint_state_age;
  result.preflight_robot_status_age = preflight_snapshot.robot_status_age;
  result.preflight_session_ready = preflight_snapshot.session_ready;
  result.preflight_session_status = preflight_snapshot.session_status;
  if (!validate_runtime_snapshot(preflight_snapshot, result.message)) {
    result.failure_stage = "runtime_preflight";
    return finalize();
  }

  const std::vector<double> expected_start_positions =
      request.expected_start_positions.empty()
          ? request.trajectory.points.front().positions
          : request.expected_start_positions;

  if (request.enforce_start_state_match) {
    const auto gate = evaluate_start_state_gate(
        expected_start_positions, preflight_snapshot.current_joint_positions);
    result.max_start_state_abs_delta = gate.max_abs_delta;
    result.start_state_l2_delta = gate.l2_delta;
    if (!gate.valid) {
      result.failure_stage = "start_state_gate_preflight";
      result.message = gate.reason;
      return finalize();
    }
  }

  if (!wait_for_server(request.result_timeout)) {
    result.failure_stage = "action_server";
    result.message =
        timeout_reason("action server", action_name_, request.result_timeout);
    return finalize();
  }

  const auto commit_snapshot = runtime_snapshot_provider_();
  result.commit_snapshot_available = true;
  result.commit_joint_state_age = commit_snapshot.joint_state_age;
  result.commit_robot_status_age = commit_snapshot.robot_status_age;
  result.commit_session_ready = commit_snapshot.session_ready;
  result.commit_session_status = commit_snapshot.session_status;
  if (!validate_runtime_snapshot(commit_snapshot, result.message)) {
    result.failure_stage = "runtime_commit";
    return finalize();
  }

  if (request.enforce_start_state_match) {
    const auto gate = evaluate_start_state_gate(
        expected_start_positions, commit_snapshot.current_joint_positions);
    result.max_start_state_abs_delta = gate.max_abs_delta;
    result.start_state_l2_delta = gate.l2_delta;
    if (!gate.valid) {
      result.failure_stage = "start_state_gate_commit";
      result.message = gate.reason;
      return finalize();
    }

    const auto drift_gate =
        evaluate_start_state_gate(preflight_snapshot.current_joint_positions,
                                  commit_snapshot.current_joint_positions);
    result.commit_drift_max_abs_delta = drift_gate.max_abs_delta;
    result.commit_drift_l2_delta = drift_gate.l2_delta;
    if (!drift_gate.valid) {
      std::ostringstream stream;
      stream << "commit-time drift recheck rejected dispatch: max_abs_delta="
             << drift_gate.max_abs_delta << " rad (limit "
             << start_state_max_abs_delta_rad_
             << "), l2_delta=" << drift_gate.l2_delta << " rad (limit "
             << start_state_max_l2_delta_rad_ << ")";
      result.failure_stage = "start_state_drift_commit";
      result.message = stream.str();
      return finalize();
    }
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = request.trajectory;

  std::string guard_reason;
  const auto guard_mode = request.extended_mode
                              ? motion_core::JointPositionGuard::Mode::Extended
                              : motion_core::JointPositionGuard::Mode::Default;
  if (!joint_position_guard_.check_trajectory(goal.trajectory, guard_reason,
                                              guard_mode)) {
    result.failure_stage = "dispatch_joint_position_guard";
    result.message = "dispatch boundary " + guard_reason;
    return finalize();
  }

  auto send_goal_options =
      rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.feedback_callback =
      [](rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr,
         const std::shared_ptr<const FollowJointTrajectory::Feedback>) {
        // Feedback is informational only for this blocking executor.
      };

  auto goal_future = action_client_->async_send_goal(goal, send_goal_options);
  const auto goal_future_state = rclcpp::spin_until_future_complete(
      client_node_->get_node_base_interface(), goal_future,
      request.result_timeout);
  if (goal_future_state != rclcpp::FutureReturnCode::SUCCESS) {
    result.failure_stage = "goal_response";
    result.message =
        goal_future_state == rclcpp::FutureReturnCode::TIMEOUT
            ? timeout_reason("goal response from", action_name_,
                             request.result_timeout)
            : "Interrupted while waiting for goal response from '" +
                  action_name_ + "'";
    return finalize();
  }

  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    result.failure_stage = "goal_rejected";
    result.message =
        "FollowJointTrajectory goal was rejected by '" + action_name_ + "'";
    return finalize();
  }

  result.accepted = true;

  auto result_future = action_client_->async_get_result(goal_handle);
  const auto result_future_state = rclcpp::spin_until_future_complete(
      client_node_->get_node_base_interface(), result_future,
      request.result_timeout);
  if (result_future_state != rclcpp::FutureReturnCode::SUCCESS) {
    result.failure_stage = "result_wait";
    result.message = result_future_state == rclcpp::FutureReturnCode::TIMEOUT
                         ? timeout_reason("result from", action_name_,
                                          request.result_timeout)
                         : "Interrupted while waiting for result from '" +
                               action_name_ + "'";
    return finalize();
  }

  result.completed = true;

  const auto wrapped_result = result_future.get();
  if (!wrapped_result.result) {
    result.failure_stage = "controller_result";
    result.message = "FollowJointTrajectory returned a null result";
    return finalize();
  }
  result.action_result_code = static_cast<int32_t>(wrapped_result.code);
  result.action_result_name =
      decode_action_result_name(result.action_result_code);

  result.controller_error_code = wrapped_result.result->error_code;
  result.controller_error_name =
      decode_controller_error_name(result.controller_error_code);
  result.controller_error_string = wrapped_result.result->error_string;

  const auto build_controller_error_message = [&result]() {
    std::ostringstream stream;
    stream << "FollowJointTrajectory reported " << result.controller_error_name
           << " (" << result.controller_error_code << ")";
    if (!result.controller_error_string.empty()) {
      stream << ": " << result.controller_error_string;
    }
    return stream.str();
  };

  switch (wrapped_result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    if (result.controller_error_code ==
        FollowJointTrajectory::Result::SUCCESSFUL) {
      result.success = true;
      result.failure_stage = "none";
      result.message.clear();
    } else {
      result.failure_stage = "controller_result";
      result.message = build_controller_error_message();
    }
    return finalize();

  case rclcpp_action::ResultCode::ABORTED:
    result.failure_stage = "controller_aborted";
    result.message = build_controller_error_message();
    return finalize();

  case rclcpp_action::ResultCode::CANCELED:
    result.canceled = true;
    result.failure_stage = "controller_canceled";
    result.message = build_controller_error_message();
    return finalize();

  default:
    result.failure_stage = "controller_unknown";
    result.message = "FollowJointTrajectory returned an unknown result code";
    return finalize();
  }
}

bool TrajectoryExecutor::validate_trajectory(
    const trajectory_msgs::msg::JointTrajectory &traj,
    std::string &reason) const {
  if (!motion_core::validate_trajectory_structure(traj, reason)) {
    return false;
  }

  if (traj.joint_names != canonical_joint_names_) {
    reason =
        "trajectory joint order does not match canonical GP4 joint ordering";
    return false;
  }

  if (traj.points.size() > kMaxTrajectoryPoints) {
    reason = "trajectory has " + std::to_string(traj.points.size()) +
             " points; MotoROS2 YRC1000micro limit is 200 and downsampling "
             "must happen earlier";
    return false;
  }

  if (!is_zero_stamp(traj.header.stamp)) {
    const rclcpp::Time header_stamp(traj.header.stamp);
    const rclcpp::Time now = client_node_->now();
    if (header_stamp <= now) {
      const auto age_ns = (now - header_stamp).nanoseconds();
      const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::nanoseconds(age_ns < 0 ? 0 : age_ns));
      if (age_ms > trajectory_header_max_age_) {
        std::ostringstream stream;
        stream << "trajectory header stamp is stale: age=" << age_ms.count()
               << " ms exceeds max_age=" << trajectory_header_max_age_.count()
               << " ms";
        reason = stream.str();
        return false;
      }
    }
  }

  reason.clear();
  return true;
}

bool TrajectoryExecutor::validate_runtime_snapshot(
    const ExecutionRuntimeSnapshot &snapshot, std::string &reason) const {
  if (!snapshot.joint_state_valid) {
    reason = snapshot.failure_reason.empty()
                 ? "joint state monitor reports invalid execution state"
                 : snapshot.failure_reason;
    return false;
  }
  if (snapshot.current_joint_positions.empty()) {
    reason = "joint state snapshot has no ordered positions";
    return false;
  }
  if (!snapshot.robot_ready) {
    reason = snapshot.failure_reason.empty()
                 ? "robot status monitor reports not ready for motion"
                 : snapshot.failure_reason;
    return false;
  }
  if (!snapshot.session_ready) {
    reason =
        snapshot.session_status.empty()
            ? "MotoROS2 session manager reports trajectory session not ready"
            : snapshot.session_status;
    return false;
  }

  reason.clear();
  return true;
}

TrajectoryExecutor::StartStateGateResult
TrajectoryExecutor::evaluate_start_state_gate(
    const std::vector<double> &expected,
    const std::vector<double> &actual) const {
  StartStateGateResult gate;
  if (expected.empty()) {
    gate.reason = "expected_start_positions is empty while start-state "
                  "enforcement is enabled";
    return gate;
  }
  if (expected.size() != actual.size()) {
    std::ostringstream stream;
    stream << "start-state vector size mismatch: expected " << expected.size()
           << " joints but runtime snapshot has " << actual.size();
    gate.reason = stream.str();
    return gate;
  }

  double squared_sum = 0.0;
  for (std::size_t index = 0; index < expected.size(); ++index) {
    const double delta = expected[index] - actual[index];
    const double abs_delta = std::abs(delta);
    gate.max_abs_delta = std::max(gate.max_abs_delta, abs_delta);
    squared_sum += delta * delta;
  }
  gate.l2_delta = std::sqrt(squared_sum);
  gate.valid = gate.max_abs_delta <= start_state_max_abs_delta_rad_ &&
               gate.l2_delta <= start_state_max_l2_delta_rad_;

  if (!gate.valid) {
    std::ostringstream stream;
    stream << "start-state continuity gate rejected dispatch: max_abs_delta="
           << gate.max_abs_delta << " rad (limit "
           << start_state_max_abs_delta_rad_ << "), l2_delta=" << gate.l2_delta
           << " rad (limit " << start_state_max_l2_delta_rad_ << ")";
    gate.reason = stream.str();
  }

  return gate;
}

void TrajectoryExecutor::log_dispatch_diagnostics(
    const TrajectoryExecutionRequest &request,
    const TrajectoryExecutionResult &result) const {
  const std::size_t point_count = request.trajectory.points.size();
  const double duration_sec =
      point_count > 0U
          ? rclcpp::Duration(request.trajectory.points.back().time_from_start)
                .seconds()
          : 0.0;
  const auto preflight_joint_age_ms =
      result.preflight_snapshot_available
          ? result.preflight_joint_state_age.count()
          : -1;
  const auto commit_joint_age_ms = result.commit_snapshot_available
                                       ? result.commit_joint_state_age.count()
                                       : -1;
  const auto preflight_robot_age_ms =
      result.preflight_snapshot_available
          ? result.preflight_robot_status_age.count()
          : -1;
  const auto commit_robot_age_ms = result.commit_snapshot_available
                                       ? result.commit_robot_status_age.count()
                                       : -1;

  RCLCPP_INFO(
      logger_,
      "dispatch_diagnostics{command_id=%s, primitive=%s, planner_id=%s, "
      "points=%zu, "
      "duration_sec=%.6f, preflight_joint_state_age_ms=%ld, "
      "commit_joint_state_age_ms=%ld, "
      "preflight_robot_status_age_ms=%ld, commit_robot_status_age_ms=%ld, "
      "preflight_session_ready=%s, commit_session_ready=%s, "
      "preflight_session_status=\"%s\", "
      "commit_session_status=\"%s\", start_state_max_abs_delta=%.6f, "
      "start_state_l2_delta=%.6f, "
      "commit_drift_max_abs_delta=%.6f, commit_drift_l2_delta=%.6f, "
      "failure_stage=%s, "
      "action_result_code=%d, action_result_name=%s, controller_error_code=%d, "
      "controller_error_name=%s, controller_error_string=\"%s\", "
      "message=\"%s\"}",
      request.command_id.empty() ? "<unset>" : request.command_id.c_str(),
      request.primitive.empty() ? "<unset>" : request.primitive.c_str(),
      request.planner_id.empty() ? "<unset>" : request.planner_id.c_str(),
      point_count, duration_sec, static_cast<long>(preflight_joint_age_ms),
      static_cast<long>(commit_joint_age_ms),
      static_cast<long>(preflight_robot_age_ms),
      static_cast<long>(commit_robot_age_ms),
      result.preflight_session_ready ? "true" : "false",
      result.commit_session_ready ? "true" : "false",
      result.preflight_session_status.empty()
          ? "<unset>"
          : result.preflight_session_status.c_str(),
      result.commit_session_status.empty()
          ? "<unset>"
          : result.commit_session_status.c_str(),
      result.max_start_state_abs_delta, result.start_state_l2_delta,
      result.commit_drift_max_abs_delta, result.commit_drift_l2_delta,
      result.failure_stage.c_str(), result.action_result_code,
      result.action_result_name.c_str(), result.controller_error_code,
      result.controller_error_name.c_str(),
      result.controller_error_string.empty()
          ? "<none>"
          : result.controller_error_string.c_str(),
      result.message.empty() ? "<none>" : result.message.c_str());
}

std::string
TrajectoryExecutor::decode_action_result_name(const int32_t result_code) {
  switch (static_cast<rclcpp_action::ResultCode>(result_code)) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    return "SUCCEEDED";
  case rclcpp_action::ResultCode::ABORTED:
    return "ABORTED";
  case rclcpp_action::ResultCode::CANCELED:
    return "CANCELED";
  case rclcpp_action::ResultCode::UNKNOWN:
  default:
    return result_code < 0 ? "NOT_SENT" : "UNKNOWN";
  }
}

std::string
TrajectoryExecutor::decode_controller_error_name(const int32_t error_code) {
  switch (error_code) {
  case FollowJointTrajectory::Result::SUCCESSFUL:
    return "SUCCESSFUL";
  case FollowJointTrajectory::Result::INVALID_GOAL:
    return "INVALID_GOAL";
  case FollowJointTrajectory::Result::INVALID_JOINTS:
    return "INVALID_JOINTS";
  case FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
    return "OLD_HEADER_TIMESTAMP";
  case FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
    return "PATH_TOLERANCE_VIOLATED";
  case FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
    return "GOAL_TOLERANCE_VIOLATED";
  default:
    return "UNKNOWN_ERROR_CODE";
  }
}
} // namespace hw_adapter
