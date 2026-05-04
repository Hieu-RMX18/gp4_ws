// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <action_msgs/msg/goal_status_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/action/execute_motion.hpp"

namespace supervisor {
enum class ExecutionMonitorState : uint8_t {
  kIdle = 0U,
  kExecuting,
  kSuccess,
  kFailed,
};

struct ExecutionMonitorSnapshot {
  bool seen_feedback = false;
  bool seen_status = false;
  double last_progress = 0.0;
  std::string current_state = "IDLE";
  std::string last_feedback_state;
  std::size_t active_goal_count = 0U;
  std::string active_goal_id;
  double active_velocity_scale = 0.0;
  double expected_duration_sec = 0.0;
  double allowed_duration_sec = 0.0;
  double last_execution_time_sec = 0.0;
  bool last_result_success = false;
  bool timeout_alert_active = false;
  uint32_t consecutive_failure_count = 0U;
  uint8_t last_alert_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string last_alert_message = "idle";
};

class ExecutionMonitor {
public:
  explicit ExecutionMonitor(rclcpp::Node &node);

  ExecutionMonitorSnapshot snapshot() const;

private:
  using ExecuteMotionGoal = interfaces::action::ExecuteMotion_Goal;
  using ExecuteMotionFeedbackMessage =
      interfaces::action::ExecuteMotion::Impl::FeedbackMessage;
  using ExecuteMotionSendGoalRequest =
      interfaces::action::ExecuteMotion_SendGoal_Request;
  using ExecuteMotionSendGoalResponse =
      interfaces::action::ExecuteMotion_SendGoal_Response;

  struct TrackedGoal {
    std::string goal_id;
    double velocity_scale = 1.0;
    double expected_duration_sec = 0.0;
    double allowed_duration_sec = 0.0;
    rclcpp::Time requested_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time accepted_time{0, 0, RCL_ROS_TIME};
    bool timeout_alert_emitted = false;
  };

  static std::string state_to_string(ExecutionMonitorState state);

  void
  send_goal_request_callback(const ExecuteMotionSendGoalRequest::SharedPtr msg);
  void send_goal_response_callback(
      const ExecuteMotionSendGoalResponse::SharedPtr msg);
  void feedback_callback(const ExecuteMotionFeedbackMessage::SharedPtr msg);
  void status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  void timeout_check_callback();
  void promote_pending_goal_to_active(const rclcpp::Time &accepted_time);
  void finalize_goal(bool success, const std::string &reason,
                     const rclcpp::Time &completed_time);
  void publish_heartbeat();
  void publish_alert(uint8_t level, const std::string &message,
                     const std::string &reason);
  TrackedGoal
  build_tracked_goal(const unique_identifier_msgs::msg::UUID &goal_id,
                     const ExecuteMotionGoal &goal,
                     const rclcpp::Time &received_time) const;
  TrackedGoal
  build_default_goal(const unique_identifier_msgs::msg::UUID &goal_id,
                     const rclcpp::Time &accepted_time) const;
  static std::string
  goal_id_to_string(const unique_identifier_msgs::msg::UUID &goal_id);
  static bool is_terminal_status(int8_t status_code);
  static bool is_active_status(int8_t status_code);
  static std::string status_code_to_string(int8_t status_code);
  double sanitize_velocity_scale(double velocity_scale) const;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  double nominal_duration_sec_ = 5.0;
  double timeout_multiplier_ = 2.0;
  double min_velocity_scale_ = 0.05;
  mutable std::mutex snapshot_mutex_;
  ExecutionMonitorSnapshot snapshot_;
  ExecutionMonitorState state_ = ExecutionMonitorState::kIdle;
  std::optional<TrackedGoal> pending_goal_;
  std::optional<TrackedGoal> active_goal_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      alert_pub_;
  rclcpp::Subscription<ExecuteMotionSendGoalRequest>::SharedPtr
      send_goal_request_sub_;
  rclcpp::Subscription<ExecuteMotionSendGoalResponse>::SharedPtr
      send_goal_response_sub_;
  rclcpp::Subscription<ExecuteMotionFeedbackMessage>::SharedPtr feedback_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
      status_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};
} // namespace supervisor
