#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "interfaces/action/dispatch_trajectory.hpp"
#include "interfaces/action/execute_motion.hpp"
#include "interfaces/srv/alarm_reset.hpp"
#include "interfaces/srv/io_set.hpp"
#include "motion_core/execution_orchestrator.hpp"

namespace motion_core {
class NonMotionPrimitiveExecutor {
public:
  using ExecuteMotion = interfaces::action::ExecuteMotion;
  using GoalHandleExecuteMotion =
      rclcpp_action::ServerGoalHandle<ExecuteMotion>;
  using DispatchTrajectory = interfaces::action::DispatchTrajectory;
  using AlarmReset = interfaces::srv::AlarmReset;
  using IoSet = interfaces::srv::IoSet;

  using PublishFeedbackFn = std::function<void(double, const std::string &)>;
  using MessageFn = std::function<void(const std::string &)>;

  NonMotionPrimitiveExecutor(
      rclcpp::Logger logger, ExecutionOrchestrator &execution_orchestrator,
      rclcpp_action::Client<DispatchTrajectory>::SharedPtr dispatch_client,
      rclcpp::Client<AlarmReset>::SharedPtr alarm_reset_client,
      std::string alarm_reset_service_name,
      rclcpp::Client<IoSet>::SharedPtr io_set_client,
      std::string io_set_service_name,
      std::function<void()> stop_move_group_fn);

  bool handle_stop_primitive(
      const std::string &primitive, const std::string &goal_id,
      const std::shared_ptr<GoalHandleExecuteMotion> &goal_handle,
      const std::chrono::steady_clock::time_point &started_at);

  bool handle_non_motion_primitive(
      const std::string &primitive, std::uint64_t goal_sequence,
      const std::shared_ptr<const ExecuteMotion::Goal> &goal,
      const std::shared_ptr<GoalHandleExecuteMotion> &goal_handle,
      const std::chrono::steady_clock::time_point &started_at,
      const PublishFeedbackFn &publish_feedback,
      const MessageFn &abort_with_message,
      const MessageFn &cancel_with_message);

private:
  static void
  set_result_timing(const std::chrono::steady_clock::time_point &started_at,
                    ExecuteMotion::Result &result);

  rclcpp::Logger logger_;
  ExecutionOrchestrator &execution_orchestrator_;
  rclcpp_action::Client<DispatchTrajectory>::SharedPtr dispatch_client_;
  rclcpp::Client<AlarmReset>::SharedPtr alarm_reset_client_;
  std::string alarm_reset_service_name_;
  rclcpp::Client<IoSet>::SharedPtr io_set_client_;
  std::string io_set_service_name_;
  std::function<void()> stop_move_group_fn_;
};
} // namespace motion_core
