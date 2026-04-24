// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "supervisor/execution_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <action_msgs/msg/goal_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace
{
template<typename ParameterT>
ParameterT declare_or_get_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const ParameterT & default_value)
{
  if (node.has_parameter(name))
  {
    return node.get_parameter(name).get_value<ParameterT>();
  }

  return node.declare_parameter<ParameterT>(name, default_value);
}

diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

namespace supervisor
{
std::string ExecutionMonitor::state_to_string(const ExecutionMonitorState state)
{
  switch (state)
  {
    case ExecutionMonitorState::kIdle:
      return "IDLE";
    case ExecutionMonitorState::kExecuting:
      return "EXECUTING";
    case ExecutionMonitorState::kSuccess:
      return "SUCCESS";
    case ExecutionMonitorState::kFailed:
      return "FAILED";
    default:
      return "UNKNOWN";
  }
}

ExecutionMonitor::ExecutionMonitor(rclcpp::Node & node)
: logger_(node.get_logger()),
  clock_(node.get_clock()),
  nominal_duration_sec_(declare_or_get_parameter<double>(
      node,
      "execution_monitor_nominal_duration_sec",
      5.0)),
  timeout_multiplier_(declare_or_get_parameter<double>(
      node,
      "execution_monitor_timeout_multiplier",
      2.0)),
  min_velocity_scale_(declare_or_get_parameter<double>(
      node,
      "execution_monitor_min_velocity_scale",
      0.05))
{
  const auto execute_motion_action_name = declare_or_get_parameter<std::string>(
    node,
    "execute_motion_action_name",
    "/execute_motion");
  const auto check_period_ms = declare_or_get_parameter<int64_t>(
    node,
    "execution_monitor_check_period_ms",
    100);
  const auto alert_heartbeat_period_ms = declare_or_get_parameter<int64_t>(
    node,
    "supervisor_alert_heartbeat_period_ms",
    1000);
  const auto alert_topic = declare_or_get_parameter<std::string>(
    node,
    "supervisor_alert_topic",
    "/supervisor/alerts");
  const auto send_goal_request_topic = declare_or_get_parameter<std::string>(
    node,
    "execute_motion_send_goal_request_topic",
    execute_motion_action_name + "/_action/send_goal/_request");
  const auto send_goal_response_topic = declare_or_get_parameter<std::string>(
    node,
    "execute_motion_send_goal_response_topic",
    execute_motion_action_name + "/_action/send_goal/_response");

  alert_pub_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    alert_topic,
    rclcpp::QoS(10).reliable());

  send_goal_request_sub_ = node.create_subscription<ExecuteMotionSendGoalRequest>(
    send_goal_request_topic,
    rclcpp::ServicesQoS(),
    std::bind(&ExecutionMonitor::send_goal_request_callback, this, std::placeholders::_1));

  send_goal_response_sub_ = node.create_subscription<ExecuteMotionSendGoalResponse>(
    send_goal_response_topic,
    rclcpp::ServicesQoS(),
    std::bind(&ExecutionMonitor::send_goal_response_callback, this, std::placeholders::_1));

  feedback_sub_ = node.create_subscription<ExecuteMotionFeedbackMessage>(
    execute_motion_action_name + "/_action/feedback",
    rclcpp::QoS(10).reliable(),
    std::bind(&ExecutionMonitor::feedback_callback, this, std::placeholders::_1));

  status_sub_ = node.create_subscription<action_msgs::msg::GoalStatusArray>(
    execute_motion_action_name + "/_action/status",
    rclcpp::QoS(10).reliable(),
    std::bind(&ExecutionMonitor::status_callback, this, std::placeholders::_1));

  timeout_timer_ = node.create_wall_timer(
    std::chrono::milliseconds(check_period_ms > 0 ? check_period_ms : 100),
    std::bind(&ExecutionMonitor::timeout_check_callback, this));
  heartbeat_timer_ = node.create_wall_timer(
    std::chrono::milliseconds(alert_heartbeat_period_ms > 0 ? alert_heartbeat_period_ms : 1000),
    std::bind(&ExecutionMonitor::publish_heartbeat, this));

  snapshot_.current_state = state_to_string(state_);
  publish_heartbeat();

  RCLCPP_INFO(
    logger_,
    "execution_monitor attached to %s; nominal_duration_sec=%.3f timeout_multiplier=%.2f",
    execute_motion_action_name.c_str(),
    nominal_duration_sec_,
    timeout_multiplier_);
}

ExecutionMonitorSnapshot ExecutionMonitor::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

void ExecutionMonitor::send_goal_request_callback(const ExecuteMotionSendGoalRequest::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  const auto received_time = clock_->now();
  const auto tracked_goal = build_tracked_goal(msg->goal_id, msg->goal, received_time);

  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  pending_goal_ = tracked_goal;
  snapshot_.active_goal_id = tracked_goal.goal_id;
  snapshot_.active_velocity_scale = tracked_goal.velocity_scale;
  snapshot_.expected_duration_sec = tracked_goal.expected_duration_sec;
  snapshot_.allowed_duration_sec = tracked_goal.allowed_duration_sec;
  snapshot_.timeout_alert_active = false;
  snapshot_.current_state = state_to_string(state_);
}

void ExecutionMonitor::send_goal_response_callback(const ExecuteMotionSendGoalResponse::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  bool finalize_as_failure = false;
  std::string failure_reason;
  rclcpp::Time completed_time = clock_->now();

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!pending_goal_.has_value())
    {
      return;
    }

    if (!msg->accepted)
    {
      state_ = ExecutionMonitorState::kFailed;
      snapshot_.current_state = state_to_string(state_);
      snapshot_.active_goal_id = pending_goal_->goal_id;
      snapshot_.active_velocity_scale = pending_goal_->velocity_scale;
      snapshot_.expected_duration_sec = pending_goal_->expected_duration_sec;
      snapshot_.allowed_duration_sec = pending_goal_->allowed_duration_sec;
      finalize_as_failure = true;
      failure_reason = "goal rejected before execution";
      completed_time = clock_->now();
    }
    else
    {
      const auto accepted_time =
        (msg->stamp.sec == 0 && msg->stamp.nanosec == 0) ?
        clock_->now() :
        rclcpp::Time(msg->stamp);
      promote_pending_goal_to_active(accepted_time);
    }
  }

  if (finalize_as_failure)
  {
    finalize_goal(false, failure_reason, completed_time);
  }
}

void ExecutionMonitor::feedback_callback(const ExecuteMotionFeedbackMessage::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.seen_feedback = true;
  snapshot_.last_progress = msg->feedback.progress;
  snapshot_.last_feedback_state = msg->feedback.current_state;
}

void ExecutionMonitor::status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  bool should_finalize = false;
  bool final_success = false;
  std::string final_reason;
  const auto now = clock_->now();

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.seen_status = true;
    // GoalStatusArray can retain terminal history for older goals, so only
    // count statuses that are still active when checking for duplicate goals.
    snapshot_.active_goal_count = static_cast<std::size_t>(std::count_if(
        msg->status_list.begin(),
        msg->status_list.end(),
        [](const action_msgs::msg::GoalStatus & status) {
          return is_active_status(status.status);
        }));

    if (snapshot_.active_goal_count > 1U)
    {
      RCLCPP_WARN(
        logger_,
        "execution_monitor observed %zu active action goals; GP4 execution is expected to remain single-goal.",
        snapshot_.active_goal_count);
    }

    for (const auto & status : msg->status_list)
    {
      const auto goal_id = goal_id_to_string(status.goal_info.goal_id);

      if (pending_goal_.has_value() && pending_goal_->goal_id == goal_id &&
        is_active_status(status.status))
      {
        promote_pending_goal_to_active(now);
      }

      if (!active_goal_.has_value() || active_goal_->goal_id != goal_id)
      {
        if (!active_goal_.has_value() && is_active_status(status.status))
        {
          active_goal_ = build_default_goal(status.goal_info.goal_id, now);
          state_ = ExecutionMonitorState::kExecuting;
          snapshot_.current_state = state_to_string(state_);
          snapshot_.active_goal_id = active_goal_->goal_id;
          snapshot_.active_velocity_scale = active_goal_->velocity_scale;
          snapshot_.expected_duration_sec = active_goal_->expected_duration_sec;
          snapshot_.allowed_duration_sec = active_goal_->allowed_duration_sec;
          snapshot_.timeout_alert_active = false;
        }
        else
        {
          continue;
        }
      }

      if (is_active_status(status.status))
      {
        state_ = ExecutionMonitorState::kExecuting;
        snapshot_.current_state = state_to_string(state_);
        snapshot_.active_goal_id = active_goal_->goal_id;
        snapshot_.active_velocity_scale = active_goal_->velocity_scale;
        snapshot_.expected_duration_sec = active_goal_->expected_duration_sec;
        snapshot_.allowed_duration_sec = active_goal_->allowed_duration_sec;
        continue;
      }

      if (is_terminal_status(status.status))
      {
        final_success = status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
        final_reason = status_code_to_string(status.status);
        should_finalize = true;
        break;
      }
    }
  }

  if (should_finalize)
  {
    finalize_goal(final_success, final_reason, now);
  }
}

void ExecutionMonitor::timeout_check_callback()
{
  bool publish_timeout = false;

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!active_goal_.has_value() || active_goal_->timeout_alert_emitted)
    {
      return;
    }

    const auto elapsed_sec = (clock_->now() - active_goal_->accepted_time).seconds();
    if (elapsed_sec <= active_goal_->allowed_duration_sec)
    {
      return;
    }

    active_goal_->timeout_alert_emitted = true;
    snapshot_.timeout_alert_active = true;
    publish_timeout = true;
  }

  if (publish_timeout)
  {
    publish_alert(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "execute_motion exceeded allowed duration",
      "elapsed wall time exceeded 2x expected duration");
  }
}

void ExecutionMonitor::promote_pending_goal_to_active(const rclcpp::Time & accepted_time)
{
  if (!pending_goal_.has_value())
  {
    return;
  }

  active_goal_ = pending_goal_;
  active_goal_->accepted_time = accepted_time;
  active_goal_->timeout_alert_emitted = false;
  pending_goal_.reset();

  state_ = ExecutionMonitorState::kExecuting;
  snapshot_.current_state = state_to_string(state_);
  snapshot_.active_goal_id = active_goal_->goal_id;
  snapshot_.active_velocity_scale = active_goal_->velocity_scale;
  snapshot_.expected_duration_sec = active_goal_->expected_duration_sec;
  snapshot_.allowed_duration_sec = active_goal_->allowed_duration_sec;
  snapshot_.timeout_alert_active = false;
}

void ExecutionMonitor::finalize_goal(
  const bool success,
  const std::string & reason,
  const rclcpp::Time & completed_time)
{
  bool publish_consecutive_failure_warning = false;
  uint32_t failure_count = 0U;

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    auto tracked_goal = active_goal_.has_value() ? active_goal_ : pending_goal_;
    if (!tracked_goal.has_value())
    {
      return;
    }

    const auto start_time =
      tracked_goal->accepted_time.nanoseconds() > 0 ?
      tracked_goal->accepted_time :
      tracked_goal->requested_time;
    const auto elapsed_sec = std::max(0.0, (completed_time - start_time).seconds());

    state_ = success ? ExecutionMonitorState::kSuccess : ExecutionMonitorState::kFailed;
    snapshot_.current_state = state_to_string(state_);
    snapshot_.active_goal_id = tracked_goal->goal_id;
    snapshot_.active_velocity_scale = tracked_goal->velocity_scale;
    snapshot_.expected_duration_sec = tracked_goal->expected_duration_sec;
    snapshot_.allowed_duration_sec = tracked_goal->allowed_duration_sec;
    snapshot_.last_execution_time_sec = elapsed_sec;
    snapshot_.last_result_success = success;

    if (success)
    {
      snapshot_.consecutive_failure_count = 0U;
      snapshot_.timeout_alert_active = false;
    }
    else
    {
      ++snapshot_.consecutive_failure_count;
      failure_count = snapshot_.consecutive_failure_count;
      publish_consecutive_failure_warning = failure_count >= 3U;
      snapshot_.timeout_alert_active = tracked_goal->timeout_alert_emitted;
    }

    active_goal_.reset();
    pending_goal_.reset();
  }

  publish_alert(
    success ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    success ? "execute_motion completed successfully" : "execute_motion failed",
    reason);

  if (publish_consecutive_failure_warning)
  {
    publish_alert(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "execute_motion consecutive failure threshold reached",
      "consecutive_failure_count=" + std::to_string(failure_count));
  }

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    state_ = ExecutionMonitorState::kIdle;
    snapshot_.current_state = state_to_string(state_);
    snapshot_.active_goal_id.clear();
    snapshot_.active_velocity_scale = 0.0;
    snapshot_.expected_duration_sec = 0.0;
    snapshot_.allowed_duration_sec = 0.0;
    snapshot_.timeout_alert_active = false;
  }
}

void ExecutionMonitor::publish_heartbeat()
{
  uint8_t heartbeat_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string heartbeat_message = "idle";
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (state_ == ExecutionMonitorState::kIdle)
    {
      snapshot_.last_alert_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      snapshot_.last_alert_message = "idle";
    }
    else if (snapshot_.last_alert_message.empty())
    {
      snapshot_.last_alert_message = "idle";
    }

    heartbeat_level = snapshot_.last_alert_level;
    heartbeat_message = snapshot_.last_alert_message;
  }

  publish_alert(
    heartbeat_level,
    heartbeat_message,
    "heartbeat");
}

void ExecutionMonitor::publish_alert(
  const uint8_t level,
  const std::string & message,
  const std::string & reason)
{
  diagnostic_msgs::msg::DiagnosticStatus alert;
  alert.level = level;
  alert.name = "supervisor/execution_monitor";
  alert.message = message;
  alert.hardware_id = "gp4_yrc1000micro";

  ExecutionMonitorSnapshot local_snapshot;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.last_alert_level = level;
    snapshot_.last_alert_message = message;
    local_snapshot = snapshot_;
  }

  alert.values.reserve(8);
  alert.values.push_back(make_key_value("reason", reason));
  alert.values.push_back(make_key_value("state", local_snapshot.current_state));
  alert.values.push_back(make_key_value("active_goal_id", local_snapshot.active_goal_id));
  alert.values.push_back(make_key_value(
      "consecutive_failure_count",
      std::to_string(local_snapshot.consecutive_failure_count)));
  alert.values.push_back(make_key_value(
      "velocity_scale",
      std::to_string(local_snapshot.active_velocity_scale)));
  alert.values.push_back(make_key_value(
      "expected_duration_sec",
      std::to_string(local_snapshot.expected_duration_sec)));
  alert.values.push_back(make_key_value(
      "allowed_duration_sec",
      std::to_string(local_snapshot.allowed_duration_sec)));
  alert.values.push_back(make_key_value(
      "last_execution_time_sec",
      std::to_string(local_snapshot.last_execution_time_sec)));

  alert_pub_->publish(alert);
}

ExecutionMonitor::TrackedGoal ExecutionMonitor::build_tracked_goal(
  const unique_identifier_msgs::msg::UUID & goal_id,
  const ExecuteMotionGoal & goal,
  const rclcpp::Time & received_time) const
{
  TrackedGoal tracked_goal;
  tracked_goal.goal_id = goal_id_to_string(goal_id);
  tracked_goal.velocity_scale = sanitize_velocity_scale(goal.velocity_scale);
  tracked_goal.expected_duration_sec = nominal_duration_sec_ / tracked_goal.velocity_scale;
  tracked_goal.allowed_duration_sec = tracked_goal.expected_duration_sec * timeout_multiplier_;
  tracked_goal.requested_time = received_time;
  tracked_goal.accepted_time = received_time;
  return tracked_goal;
}

ExecutionMonitor::TrackedGoal ExecutionMonitor::build_default_goal(
  const unique_identifier_msgs::msg::UUID & goal_id,
  const rclcpp::Time & accepted_time) const
{
  ExecuteMotionGoal synthetic_goal;
  synthetic_goal.velocity_scale = 1.0;
  return build_tracked_goal(goal_id, synthetic_goal, accepted_time);
}

std::string ExecutionMonitor::goal_id_to_string(const unique_identifier_msgs::msg::UUID & goal_id)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (const auto byte : goal_id.uuid)
  {
    oss << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

bool ExecutionMonitor::is_terminal_status(const int8_t status_code)
{
  return status_code == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
         status_code == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
         status_code == action_msgs::msg::GoalStatus::STATUS_ABORTED;
}

bool ExecutionMonitor::is_active_status(const int8_t status_code)
{
  return status_code == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status_code == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
         status_code == action_msgs::msg::GoalStatus::STATUS_CANCELING;
}

std::string ExecutionMonitor::status_code_to_string(const int8_t status_code)
{
  switch (status_code)
  {
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      return "STATUS_ACCEPTED";
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      return "STATUS_EXECUTING";
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      return "STATUS_CANCELING";
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      return "STATUS_SUCCEEDED";
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      return "STATUS_CANCELED";
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      return "STATUS_ABORTED";
    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
    default:
      return "STATUS_UNKNOWN";
  }
}

double ExecutionMonitor::sanitize_velocity_scale(const double velocity_scale) const
{
  if (!std::isfinite(velocity_scale))
  {
    return 1.0;
  }

  return std::clamp(velocity_scale, min_velocity_scale_, 1.0);
}
}  // namespace supervisor
