// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/hw_adapter_node.hpp"

#include <chrono>
#include <string>
#include <utility>

namespace hw_adapter
{
HwAdapterNode::HwAdapterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("hw_adapter_node", options),
  backend_capabilities_(*this)
{
  const auto robot_status_topic =
    declare_parameter<std::string>("robot_status_topic", "/yaskawa/robot_status");
  const auto trajectory_action_name = declare_parameter<std::string>(
    "follow_joint_trajectory_action",
    backend_capabilities_.snapshot().names.trajectory_action);
  const auto start_traj_mode_service =
    declare_parameter<std::string>("start_traj_mode_service", "/yaskawa/start_traj_mode");
  const auto reset_error_service =
    declare_parameter<std::string>("reset_error_service", "/yaskawa/reset_error");
  const auto stop_motion_service =
    declare_parameter<std::string>("stop_motion_service", std::string());
  const auto read_single_io_service =
    declare_parameter<std::string>("read_single_io_service", std::string());
  const auto write_single_io_service =
    declare_parameter<std::string>("write_single_io_service", std::string());
  const auto tool_io_address = declare_parameter<int64_t>("tool_io_address", 0);
  const auto tool_poll_period_ms = declare_parameter<int64_t>("tool_poll_period_ms", 250);
  const auto dispatch_action_name =
    declare_parameter<std::string>("dispatch_action_name", "/hw_adapter/dispatch_trajectory");

  robot_status_monitor_ = std::make_unique<RobotStatusMonitor>(*this, robot_status_topic);
  session_manager_ = std::make_unique<Motoros2SessionManager>(
    *this,
    SessionServiceNames{
      start_traj_mode_service,
      reset_error_service,
      stop_motion_service,
      trajectory_action_name});
  trajectory_executor_ = std::make_unique<TrajectoryExecutor>(
    *this,
    trajectory_action_name,
    [this](std::string & reason) {
      return robot_status_monitor_->is_ready_for_motion(reason);
    },
    [this](std::string & reason) {
      if (session_manager_->is_session_ready())
      {
        reason.clear();
        return true;
      }
      reason = session_manager_->snapshot().status_message;
      return false;
    });
  tool_state_monitor_ = std::make_unique<ToolStateMonitor>(
    *this,
    ToolServiceNames{read_single_io_service, write_single_io_service},
    tool_io_address > 0 ? static_cast<uint32_t>(tool_io_address) : 0U,
    std::chrono::milliseconds(tool_poll_period_ms > 0 ? tool_poll_period_ms : 0));

  // --- DispatchTrajectory action server ---
  // This is the single ROS interface through which motion_core sends
  // validated trajectories for execution on the real robot controller.
  dispatch_action_server_ = rclcpp_action::create_server<DispatchTrajectory>(
    this,
    dispatch_action_name,
    std::bind(&HwAdapterNode::handle_dispatch_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&HwAdapterNode::handle_dispatch_cancel, this,
      std::placeholders::_1),
    std::bind(&HwAdapterNode::handle_dispatch_accepted, this,
      std::placeholders::_1));

  {
    std::lock_guard<std::mutex> lock(orchestration_mutex_);
    last_status_message_ =
      "hw_adapter orchestrator initialized; waiting for robot readiness before execution";
  }

  RCLCPP_INFO(
    get_logger(),
    "hw_adapter_node initialized for %s on %s; DispatchTrajectory action: %s",
    backend_capabilities_.controller_variant().c_str(),
    trajectory_action_name.c_str(),
    dispatch_action_name.c_str());
}

// --- DispatchTrajectory action callbacks ---

rclcpp_action::GoalResponse HwAdapterNode::handle_dispatch_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const DispatchTrajectory::Goal> goal)
{
  if (!goal || goal->trajectory.points.empty())
  {
    RCLCPP_WARN(get_logger(), "Rejecting DispatchTrajectory: empty trajectory.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(orchestration_mutex_);
    if (execution_in_progress_)
    {
      RCLCPP_WARN(get_logger(),
        "Rejecting DispatchTrajectory: execution already in progress.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HwAdapterNode::handle_dispatch_cancel(
  const std::shared_ptr<GoalHandleDispatchTrajectory>)
{
  RCLCPP_INFO(get_logger(), "DispatchTrajectory cancel requested.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void HwAdapterNode::handle_dispatch_accepted(
  const std::shared_ptr<GoalHandleDispatchTrajectory> goal_handle)
{
  std::thread([this, goal_handle]() { execute_dispatch(goal_handle); }).detach();
}

void HwAdapterNode::execute_dispatch(
  const std::shared_ptr<GoalHandleDispatchTrajectory> goal_handle)
{
  const auto started_at = std::chrono::steady_clock::now();
  const auto goal = goal_handle->get_goal();

  // Publish feedback: readiness check
  auto feedback = std::make_shared<DispatchTrajectory::Feedback>();
  feedback->state = "readiness_check";
  goal_handle->publish_feedback(feedback);

  const double timeout_sec = (goal->timeout_sec > 0.0) ? goal->timeout_sec : 30.0;
  const auto timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(timeout_sec));

  // Delegate to the existing execute_trajectory orchestration
  const auto report = execute_trajectory(goal->trajectory, timeout_ms);

  const auto ended_at = std::chrono::steady_clock::now();
  const double execution_time_sec =
    std::chrono::duration_cast<std::chrono::duration<double>>(ended_at - started_at).count();

  auto result = std::make_shared<DispatchTrajectory::Result>();
  result->success = report.success;
  result->message = report.message;
  result->execution_time_sec = execution_time_sec;

  if (report.success)
  {
    feedback->state = "done";
    goal_handle->publish_feedback(feedback);
    goal_handle->succeed(result);
  }
  else
  {
    feedback->state = "failed";
    goal_handle->publish_feedback(feedback);
    goal_handle->abort(result);
  }
}

const BackendCapabilities & HwAdapterNode::backend_capabilities() const
{
  return backend_capabilities_;
}

const RobotStatusMonitor & HwAdapterNode::robot_status_monitor() const
{
  return *robot_status_monitor_;
}

const Motoros2SessionManager & HwAdapterNode::session_manager() const
{
  return *session_manager_;
}

const TrajectoryExecutor & HwAdapterNode::trajectory_executor() const
{
  return *trajectory_executor_;
}

const ToolStateMonitor & HwAdapterNode::tool_state_monitor() const
{
  return *tool_state_monitor_;
}

bool HwAdapterNode::is_ready_for_execution(std::string & reason) const
{
  {
    std::lock_guard<std::mutex> lock(orchestration_mutex_);
    if (execution_in_progress_)
    {
      reason = "hw_adapter is already executing; asynchronous motion is unsupported";
      return false;
    }
  }

  return robot_status_monitor_->is_ready_for_motion(reason);
}

HwAdapterOrchestrationSnapshot HwAdapterNode::orchestration_snapshot() const
{
  std::lock_guard<std::mutex> lock(orchestration_mutex_);
  return build_snapshot_locked();
}

std::string HwAdapterNode::orchestration_status() const
{
  return orchestration_snapshot().status_message;
}

HwAdapterExecutionReport HwAdapterNode::execute_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::chrono::milliseconds timeout)
{
  HwAdapterExecutionReport report;

  {
    std::lock_guard<std::mutex> lock(orchestration_mutex_);
    if (execution_in_progress_)
    {
      report.blocked = true;
      report.message = "hw_adapter is already executing; asynchronous motion is unsupported";
      last_status_message_ = report.message;
      return report;
    }

    execution_in_progress_ = true;
    last_execution_success_ = false;
    last_error_was_fatal_ = false;
    last_status_message_ = "hw_adapter checking robot readiness before trajectory execution";
  }

  auto finalize = [this, &report](const std::string & status_message) {
      std::lock_guard<std::mutex> lock(orchestration_mutex_);
      execution_in_progress_ = false;
      last_execution_success_ = report.success;
      last_error_was_fatal_ = report.fatal_error;
      last_status_message_ = status_message;
    };

  std::string reason;
  if (!robot_status_monitor_->is_ready_for_motion(reason))
  {
    report.blocked = true;
    report.message = reason.empty() ? "robot is not ready for execution" : std::move(reason);
    finalize("execution blocked: " + report.message);
    RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
    return report;
  }

  if (!trajectory_executor_->validate_trajectory_request(trajectory, reason))
  {
    report.message = reason;
    finalize("execution rejected before dispatch: " + report.message);
    RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
    return report;
  }

  if (!backend_capabilities_.validate_joint_names(trajectory.joint_names, reason))
  {
    report.message = reason;
    finalize("execution rejected before dispatch: " + report.message);
    RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
    return report;
  }

  {
    std::lock_guard<std::mutex> lock(orchestration_mutex_);
    last_status_message_ = "hw_adapter starting MotoROS2 trajectory mode";
  }

  if (!session_manager_->ensure_trajectory_mode(reason))
  {
    report.message = reason.empty() ? "failed to enter trajectory mode" : std::move(reason);
    finalize("execution blocked: " + report.message);
    RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
    return report;
  }

  {
    std::lock_guard<std::mutex> lock(orchestration_mutex_);
    last_status_message_ = "hw_adapter executing validated trajectory";
  }

  const auto execution_result = trajectory_executor_->execute_blocking(
    TrajectoryExecutionRequest{trajectory, timeout});
  report.success = execution_result.success;
  report.message = execution_result.message.empty() ?
    "trajectory execution completed successfully" :
    execution_result.message;
  report.fatal_error = should_stop_motion_on_failure(execution_result);

  if (report.success)
  {
    finalize("trajectory execution completed successfully");
    RCLCPP_INFO(get_logger(), "Trajectory execution completed successfully.");
    return report;
  }

  if (report.fatal_error)
  {
    std::string stop_reason;
    report.stop_motion_attempted = true;
    report.stop_motion_succeeded = session_manager_->stop_motion(stop_reason);
    if (!report.stop_motion_succeeded && !stop_reason.empty())
    {
      report.message += " | stop_motion failed: " + stop_reason;
    }

    // V4 J4-Recovery: attempt deterministic recovery after fatal failure
    RCLCPP_WARN(get_logger(), "Fatal execution failure — initiating J4-Recovery");
    const auto recovery_result = attempt_recovery();
    if (recovery_result.recovered)
    {
      finalize("fatal failure recovered in " +
        std::to_string(recovery_result.elapsed_sec) + "s");
    }
    else
    {
      finalize(
        "fatal execution failure: recovery FAILED at " +
        std::string(recovery_state_name(recovery_result.final_state)));
    }

    RCLCPP_ERROR(get_logger(), "%s", report.message.c_str());
    return report;
  }

  finalize("execution failed before motion completed: " + report.message);
  RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
  return report;
}

HwAdapterOrchestrationSnapshot HwAdapterNode::build_snapshot_locked() const
{
  HwAdapterOrchestrationSnapshot snapshot;
  snapshot.robot_ready = robot_status_monitor_ && robot_status_monitor_->is_ready();
  snapshot.session_ready = session_manager_ && session_manager_->is_session_ready();
  snapshot.tool_state_available = tool_state_monitor_ && tool_state_monitor_->has_tool_state();
  snapshot.execution_in_progress = execution_in_progress_;
  snapshot.execution_allowed = snapshot.robot_ready && !snapshot.execution_in_progress;
  snapshot.last_execution_success = last_execution_success_;
  snapshot.last_error_was_fatal = last_error_was_fatal_;
  snapshot.status_message = last_status_message_;
  return snapshot;
}

bool HwAdapterNode::should_stop_motion_on_failure(
  const TrajectoryExecutionResult & result) const
{
  return !result.success && (result.accepted || result.completed);
}

RecoveryResult HwAdapterNode::attempt_recovery()
{
  RecoveryCallbacks callbacks;

  callbacks.stop_motion = [this](std::string & reason) {
    return session_manager_->stop_motion(reason);
  };

  callbacks.reset_error = [this](std::string & reason) {
    return session_manager_->reset_error(reason);
  };

  callbacks.verify_joint_state = [this](std::string & reason) {
    if (!robot_status_monitor_->has_status())
    {
      reason = "no robot status received since startup";
      return false;
    }
    reason.clear();
    return true;
  };

  callbacks.is_ready_for_motion = [this](std::string & reason) {
    return robot_status_monitor_->is_ready_for_motion(reason);
  };

  RecoveryStateMachine fsm(
    get_logger(),
    std::move(callbacks),
    std::chrono::seconds(5),   // per-step timeout
    std::chrono::seconds(20)); // total recovery timeout

  return fsm.execute();
}
}  // namespace hw_adapter
