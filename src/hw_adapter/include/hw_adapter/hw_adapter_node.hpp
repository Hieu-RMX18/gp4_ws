// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "hw_adapter/backend_capabilities.hpp"
#include "hw_adapter/motoros2_session_manager.hpp"
#include "hw_adapter/robot_status_monitor.hpp"
#include "hw_adapter/tool_state_monitor.hpp"
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

private:
  HwAdapterOrchestrationSnapshot build_snapshot_locked() const;
  bool should_stop_motion_on_failure(const TrajectoryExecutionResult & result) const;

  BackendCapabilities backend_capabilities_;
  std::unique_ptr<RobotStatusMonitor> robot_status_monitor_;
  std::unique_ptr<Motoros2SessionManager> session_manager_;
  std::unique_ptr<TrajectoryExecutor> trajectory_executor_;
  std::unique_ptr<ToolStateMonitor> tool_state_monitor_;
  mutable std::mutex orchestration_mutex_;
  bool execution_in_progress_ = false;
  bool last_execution_success_ = false;
  bool last_error_was_fatal_ = false;
  std::string last_status_message_ = "hw_adapter orchestrator initialized";
};
}  // namespace hw_adapter
