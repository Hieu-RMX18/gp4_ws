// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <interfaces/action/dispatch_trajectory.hpp>
#include <interfaces/msg/robot_readiness.hpp>
#include <interfaces/srv/alarm_reset.hpp>
#include <interfaces/srv/io_set.hpp>

#include "hw_adapter/backend_capabilities.hpp"
#include "hw_adapter/motoros2_session_manager.hpp"
#include "hw_adapter/robot_status_monitor.hpp"
#include "hw_adapter/tool_state_monitor.hpp"
#include "hw_adapter/recovery_state_machine.hpp"
#include "hw_adapter/trajectory_executor.hpp"

namespace hw_adapter
{
struct HwAdapterOrchestrationSnapshot
{
  bool robot_ready = false;
  bool session_ready = false;
  bool tool_state_available = false;
  bool execution_allowed = false;
  bool execution_in_progress = false;
  bool last_execution_success = false;
  bool last_error_was_fatal = false;
  std::string status_message = "hw_adapter orchestrator initialized";
};

struct HwAdapterExecutionReport
{
  bool success = false;
  bool blocked = false;
  bool fatal_error = false;
  bool stop_motion_attempted = false;
  bool stop_motion_succeeded = false;
  std::string message = "trajectory not executed";
};

class HwAdapterNode final : public rclcpp::Node
{
public:
  using DispatchTrajectory = interfaces::action::DispatchTrajectory;
  using GoalHandleDispatchTrajectory = rclcpp_action::ServerGoalHandle<DispatchTrajectory>;
  using AlarmReset = interfaces::srv::AlarmReset;
  using IoSet = interfaces::srv::IoSet;

  explicit HwAdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const BackendCapabilities & backend_capabilities() const;
  const RobotStatusMonitor & robot_status_monitor() const;
  const Motoros2SessionManager & session_manager() const;
  const TrajectoryExecutor & trajectory_executor() const;
  const ToolStateMonitor & tool_state_monitor() const;
  bool is_ready_for_execution(std::string & reason) const;
  HwAdapterOrchestrationSnapshot orchestration_snapshot() const;
  std::string orchestration_status() const;
  HwAdapterExecutionReport execute_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::chrono::milliseconds timeout = std::chrono::seconds(30));

  /// V4 J4-Recovery: attempt deterministic recovery after fatal execution failure.
  RecoveryResult attempt_recovery();

private:
  // DispatchTrajectory action server callbacks
  rclcpp_action::GoalResponse handle_dispatch_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DispatchTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_dispatch_cancel(
    const std::shared_ptr<GoalHandleDispatchTrajectory> goal_handle);
  void handle_dispatch_accepted(
    const std::shared_ptr<GoalHandleDispatchTrajectory> goal_handle);
  void execute_dispatch(
    const std::shared_ptr<GoalHandleDispatchTrajectory> goal_handle);
  void publish_sim_readiness();
  HwAdapterExecutionReport execute_trajectory_internal(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::chrono::milliseconds timeout,
    bool dispatch_reservation_expected);

  HwAdapterOrchestrationSnapshot build_snapshot_locked() const;
  bool should_stop_motion_on_failure(const TrajectoryExecutionResult & result) const;
  static std::string goal_uuid_to_string(const rclcpp_action::GoalUUID & goal_id);

  // Step 4.1: AlarmReset service handler
  void handle_alarm_reset(
    const std::shared_ptr<AlarmReset::Request> request,
    std::shared_ptr<AlarmReset::Response> response);

  // Step 4.2: IoSet service handler
  void handle_io_set(
    const std::shared_ptr<IoSet::Request> request,
    std::shared_ptr<IoSet::Response> response);

  BackendCapabilities backend_capabilities_;
  std::unique_ptr<RobotStatusMonitor> robot_status_monitor_;
  std::unique_ptr<Motoros2SessionManager> session_manager_;
  std::unique_ptr<TrajectoryExecutor> trajectory_executor_;
  std::unique_ptr<ToolStateMonitor> tool_state_monitor_;
  rclcpp::Publisher<interfaces::msg::RobotReadiness>::SharedPtr sim_readiness_pub_;
  rclcpp::TimerBase::SharedPtr sim_readiness_timer_;

  // DispatchTrajectory action server
  rclcpp_action::Server<DispatchTrajectory>::SharedPtr dispatch_action_server_;

  // Step 4.1/4.2: AlarmReset and IoSet service servers
  rclcpp::Service<AlarmReset>::SharedPtr alarm_reset_service_;
  rclcpp::Service<IoSet>::SharedPtr io_set_service_;

  mutable std::mutex orchestration_mutex_;
  bool sim_mode_ = false;
  bool dispatch_goal_reserved_ = false;
  bool execution_in_progress_ = false;
  bool last_execution_success_ = false;
  bool last_error_was_fatal_ = false;
  std::size_t trajectory_safe_budget_points_ = 180U;
  std::size_t trajectory_hard_limit_points_ = 200U;
  std::string last_status_message_ = "hw_adapter orchestrator initialized";
};
}  // namespace hw_adapter
