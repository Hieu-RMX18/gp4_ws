// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/hw_adapter_node.hpp"

#include <atomic>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace hw_adapter
{
std::string HwAdapterNode::goal_uuid_to_string(const rclcpp_action::GoalUUID & goal_id)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : goal_id)
  {
    stream << std::setw(2) << static_cast<int>(byte);
  }
  return stream.str();
}

HwAdapterNode::HwAdapterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("hw_adapter_node", options),
  backend_capabilities_(*this)
{
  const auto sim_mode = declare_parameter<bool>("sim_mode", false);
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
  const auto trajectory_safe_budget_param =
    declare_parameter<int64_t>("trajectory_safe_budget_points", 180);
  const auto trajectory_hard_limit_param =
    declare_parameter<int64_t>("trajectory_hard_limit_points", 200);
  sim_mode_ = sim_mode;

  trajectory_safe_budget_points_ =
    trajectory_safe_budget_param > 1 ? static_cast<std::size_t>(trajectory_safe_budget_param) : 180U;
  trajectory_hard_limit_points_ =
    trajectory_hard_limit_param > 1 ? static_cast<std::size_t>(trajectory_hard_limit_param) : 200U;
  if (trajectory_hard_limit_points_ < trajectory_safe_budget_points_)
  {
    RCLCPP_WARN(
      get_logger(),
      "trajectory_hard_limit_points (%zu) < trajectory_safe_budget_points (%zu); "
      "forcing hard limit to safe budget.",
      trajectory_hard_limit_points_,
      trajectory_safe_budget_points_);
    trajectory_hard_limit_points_ = trajectory_safe_budget_points_;
  }

  if (!sim_mode_)
  {
    robot_status_monitor_ = std::make_unique<RobotStatusMonitor>(*this, robot_status_topic);
  }
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
      if (sim_mode_)
      {
        reason.clear();
        return true;
      }
      if (!robot_status_monitor_)
      {
        reason = "robot_status_monitor is not initialized";
        return false;
      }
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
    last_status_message_ = sim_mode_ ?
      "simulation mode: robot status bypassed" :
      "hw_adapter orchestrator initialized; waiting for robot readiness before execution";
  }

  if (sim_mode_)
  {
    sim_readiness_pub_ = create_publisher<interfaces::msg::RobotReadiness>(
      "/hw_adapter/ready",
      rclcpp::QoS(1).reliable().transient_local());
    sim_readiness_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&HwAdapterNode::publish_sim_readiness, this));
    publish_sim_readiness();
    RCLCPP_INFO(
      get_logger(),
      "hw_adapter_node running in SIM MODE: robot_status readiness bypass enabled");
  }

  // --- Step 4.1: AlarmReset service server ---
  // Delegates to session_manager_->reset_error() which calls MotoROS2 reset_error.
  alarm_reset_service_ = create_service<AlarmReset>(
    "/hw_adapter/alarm_reset",
    std::bind(&HwAdapterNode::handle_alarm_reset, this,
      std::placeholders::_1, std::placeholders::_2));

  // --- Step 4.2: IoSet service server ---
  // Delegates to MotoROS2 WriteSingleIO if available at build time.
  io_set_service_ = create_service<IoSet>(
    "/hw_adapter/io_set",
    std::bind(&HwAdapterNode::handle_io_set, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "hw_adapter_node initialized for %s on %s; DispatchTrajectory action: %s; "
    "trajectory budget safe=%zu hard=%zu",
    backend_capabilities_.controller_variant().c_str(),
    trajectory_action_name.c_str(),
    dispatch_action_name.c_str(),
    trajectory_safe_budget_points_,
    trajectory_hard_limit_points_);
}

// --- DispatchTrajectory action callbacks ---

rclcpp_action::GoalResponse HwAdapterNode::handle_dispatch_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const DispatchTrajectory::Goal> goal)
{
  if (!goal || goal->trajectory.points.empty())
  {
    RCLCPP_WARN(get_logger(), "Rejecting DispatchTrajectory: empty trajectory.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(orchestration_mutex_);
    if (execution_in_progress_ || dispatch_goal_reserved_)
    {
      last_status_message_ = "dispatch rejected: execution already in progress";
      RCLCPP_WARN(get_logger(),
        "Rejecting DispatchTrajectory goal_id=%s: execution already in progress or reserved.",
        goal_uuid_to_string(uuid).c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    dispatch_goal_reserved_ = true;
    last_status_message_ = "dispatch goal accepted; reserving single execution slot";
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted DispatchTrajectory goal_id=%s points=%zu",
    goal_uuid_to_string(uuid).c_str(),
    goal->trajectory.points.size());

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HwAdapterNode::handle_dispatch_cancel(
  const std::shared_ptr<GoalHandleDispatchTrajectory> goal_handle)
{
  RCLCPP_INFO(
    get_logger(),
    "DispatchTrajectory cancel requested for goal_id=%s.",
    goal_uuid_to_string(goal_handle->get_goal_id()).c_str());
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
  const std::string dispatch_goal_id = goal_uuid_to_string(goal_handle->get_goal_id());

  // Publish feedback: readiness check
  auto feedback = std::make_shared<DispatchTrajectory::Feedback>();
  feedback->state = "readiness_check";
  goal_handle->publish_feedback(feedback);

  const double timeout_sec = (goal->timeout_sec > 0.0) ? goal->timeout_sec : 30.0;
  const auto timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(timeout_sec));
  RCLCPP_INFO(
    get_logger(),
    "DispatchTrajectory goal_id=%s dispatch_start timeout=%.2fs points=%zu",
    dispatch_goal_id.c_str(),
    timeout_sec,
    goal->trajectory.points.size());

  if (goal_handle->is_canceling())
  {
    {
      std::lock_guard<std::mutex> lock(orchestration_mutex_);
      dispatch_goal_reserved_ = false;
      last_status_message_ = "dispatch canceled before execution start";
    }

    auto result = std::make_shared<DispatchTrajectory::Result>();
    result->success = false;
    result->message = "DispatchTrajectory canceled before execution start";
    result->execution_time_sec = 0.0;
    feedback->state = "canceled";
    goal_handle->publish_feedback(feedback);
    goal_handle->canceled(result);
    return;
  }

  std::atomic<bool> execution_finished{false};
  std::atomic<bool> cancel_stop_requested{false};
  std::thread cancel_watcher([this, goal_handle, &execution_finished, &cancel_stop_requested, dispatch_goal_id]() {
    while (!execution_finished.load())
    {
      if (goal_handle->is_canceling())
      {
        cancel_stop_requested.store(true);
        std::string stop_reason;
        const bool stopped = session_manager_ && session_manager_->stop_motion(stop_reason);
        RCLCPP_WARN(
          get_logger(),
          "DispatchTrajectory goal_id=%s cancel watcher stop_motion attempted=%s success=%s detail=%s",
          dispatch_goal_id.c_str(),
          session_manager_ ? "true" : "false",
          stopped ? "true" : "false",
          stop_reason.empty() ? "<none>" : stop_reason.c_str());
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  // Delegate to the existing execute_trajectory orchestration
  const auto report = execute_trajectory_internal(goal->trajectory, timeout_ms, true);
  execution_finished.store(true);
  if (cancel_watcher.joinable())
  {
    cancel_watcher.join();
  }

  const auto ended_at = std::chrono::steady_clock::now();
  const double execution_time_sec =
    std::chrono::duration_cast<std::chrono::duration<double>>(ended_at - started_at).count();

  auto result = std::make_shared<DispatchTrajectory::Result>();
  result->success = report.success;
  result->message = report.message;
  result->execution_time_sec = execution_time_sec;

  if (goal_handle->is_canceling() || cancel_stop_requested.load())
  {
    result->success = false;
    if (result->message.empty())
    {
      result->message = "DispatchTrajectory canceled";
    }
    feedback->state = "canceled";
    goal_handle->publish_feedback(feedback);
    goal_handle->canceled(result);
    return;
  }

  if (report.success)
  {
    feedback->state = "done";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(
      get_logger(),
      "DispatchTrajectory goal_id=%s dispatch_end success=true message=%s",
      dispatch_goal_id.c_str(),
      report.message.c_str());
    goal_handle->succeed(result);
  }
  else
  {
    feedback->state = "failed";
    goal_handle->publish_feedback(feedback);
    RCLCPP_WARN(
      get_logger(),
      "DispatchTrajectory goal_id=%s dispatch_end success=false message=%s",
      dispatch_goal_id.c_str(),
      report.message.c_str());
    goal_handle->abort(result);
  }
}

void HwAdapterNode::publish_sim_readiness()
{
  if (!sim_mode_ || !sim_readiness_pub_)
  {
    return;
  }

  interfaces::msg::RobotReadiness msg;
  msg.ready = true;
  msg.status_message = "simulation mode: robot status bypassed";
  sim_readiness_pub_->publish(msg);
}

const BackendCapabilities & HwAdapterNode::backend_capabilities() const
{
  return backend_capabilities_;
}

const RobotStatusMonitor & HwAdapterNode::robot_status_monitor() const
{
  if (!robot_status_monitor_)
  {
    throw std::runtime_error("robot_status_monitor is unavailable when sim_mode=true");
  }
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
    if (execution_in_progress_ || dispatch_goal_reserved_)
    {
      reason = "hw_adapter is already executing; asynchronous motion is unsupported";
      return false;
    }
  }

  if (sim_mode_)
  {
    reason.clear();
    return true;
  }
  if (!robot_status_monitor_)
  {
    reason = "robot_status_monitor is not initialized";
    return false;
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
  return execute_trajectory_internal(trajectory, timeout, false);
}

HwAdapterExecutionReport HwAdapterNode::execute_trajectory_internal(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::chrono::milliseconds timeout,
  const bool dispatch_reservation_expected)
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

    if (dispatch_reservation_expected)
    {
      if (!dispatch_goal_reserved_)
      {
        report.blocked = true;
        report.message = "dispatch execution reservation was lost before the worker thread started";
        last_status_message_ = report.message;
        return report;
      }
      dispatch_goal_reserved_ = false;
    }
    else if (dispatch_goal_reserved_)
    {
      report.blocked = true;
      report.message = "hw_adapter already has a reserved dispatch goal awaiting execution";
      last_status_message_ = report.message;
      return report;
    }

    execution_in_progress_ = true;
    last_execution_success_ = false;
    last_error_was_fatal_ = false;
    last_status_message_ = sim_mode_ ?
      "hw_adapter SIM MODE: robot_status readiness bypassed for trajectory execution" :
      "hw_adapter checking robot readiness before trajectory execution";
  }

  auto finalize = [this, &report](const std::string & status_message) {
      std::lock_guard<std::mutex> lock(orchestration_mutex_);
      dispatch_goal_reserved_ = false;
      execution_in_progress_ = false;
      last_execution_success_ = report.success;
      last_error_was_fatal_ = report.fatal_error;
      last_status_message_ = status_message;
    };

  std::string reason;
  if (!sim_mode_ && !robot_status_monitor_)
  {
    report.blocked = true;
    report.message = "robot_status_monitor is not initialized";
    finalize("execution blocked: " + report.message);
    RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
    return report;
  }
  if (!sim_mode_ && !robot_status_monitor_->is_ready_for_motion(reason))
  {
    report.blocked = true;
    report.message = reason.empty() ? "robot is not ready for execution" : std::move(reason);
    finalize("execution blocked: " + report.message);
    RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
    return report;
  }

  const std::size_t point_count = trajectory.points.size();
  if (point_count > trajectory_hard_limit_points_)
  {
    report.blocked = true;
    report.message =
      "trajectory has " + std::to_string(point_count) +
      " points, exceeding hard limit " + std::to_string(trajectory_hard_limit_points_);
    finalize("execution rejected before dispatch: " + report.message);
    RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
    return report;
  }

  if (point_count > trajectory_safe_budget_points_)
  {
    RCLCPP_WARN(
      get_logger(),
      "trajectory has %zu points above safe budget %zu (hard limit %zu); "
      "expect upstream mitigation policy (downsample or split).",
      point_count,
      trajectory_safe_budget_points_,
      trajectory_hard_limit_points_);
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
    if (sim_mode_)
    {
      // In sim_mode, MotoROS2 session services are unavailable by design.
      // Bypass the trajectory mode gate and continue with execution.
      RCLCPP_WARN(
        get_logger(),
        "SIM MODE: bypassing ensure_trajectory_mode failure: %s",
        report.message.c_str());
      report.message.clear();
      // Fall through to trajectory execution below
    }
    else
    {
      finalize("execution blocked: " + report.message);
      RCLCPP_WARN(get_logger(), "%s", report.message.c_str());
      return report;
    }
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
  snapshot.robot_ready = sim_mode_ || (robot_status_monitor_ && robot_status_monitor_->is_ready());
  snapshot.session_ready = session_manager_ && session_manager_->is_session_ready();
  snapshot.tool_state_available = tool_state_monitor_ && tool_state_monitor_->has_tool_state();
  snapshot.execution_in_progress = execution_in_progress_ || dispatch_goal_reserved_;
  snapshot.execution_allowed = snapshot.robot_ready && !snapshot.execution_in_progress;
  snapshot.last_execution_success = last_execution_success_;
  snapshot.last_error_was_fatal = last_error_was_fatal_;
  snapshot.status_message = last_status_message_;
  if (sim_mode_ && !snapshot.execution_in_progress)
  {
    const bool has_active_issue =
      snapshot.last_error_was_fatal ||
      snapshot.status_message.find("failed") != std::string::npos ||
      snapshot.status_message.find("blocked") != std::string::npos ||
      snapshot.status_message.find("rejected") != std::string::npos;
    if (!has_active_issue)
    {
      snapshot.status_message = "simulation mode: robot status bypassed";
    }
  }
  return snapshot;
}

bool HwAdapterNode::should_stop_motion_on_failure(
  const TrajectoryExecutionResult & result) const
{
  return !result.success && !result.canceled && (result.accepted || result.completed);
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
    if (sim_mode_)
    {
      reason.clear();
      return true;
    }
    if (!robot_status_monitor_)
    {
      reason = "robot_status_monitor is not initialized";
      return false;
    }
    if (!robot_status_monitor_->has_status())
    {
      reason = "no robot status received since startup";
      return false;
    }
    reason.clear();
    return true;
  };

  callbacks.is_ready_for_motion = [this](std::string & reason) {
    if (sim_mode_)
    {
      reason.clear();
      return true;
    }
    if (!robot_status_monitor_)
    {
      reason = "robot_status_monitor is not initialized";
      return false;
    }
    return robot_status_monitor_->is_ready_for_motion(reason);
  };

  RecoveryStateMachine fsm(
    get_logger(),
    std::move(callbacks),
    std::chrono::seconds(5),   // per-step timeout
    std::chrono::seconds(20)); // total recovery timeout

  return fsm.execute();
}

// --- Step 4.1: AlarmReset service handler ---
// Delegates to session_manager_->reset_error() which wraps MotoROS2 reset_error service.
void HwAdapterNode::handle_alarm_reset(
  const std::shared_ptr<AlarmReset::Request> /*request*/,
  std::shared_ptr<AlarmReset::Response> response)
{
  if (!session_manager_)
  {
    response->success = false;
    response->message = "session_manager not initialized";
    RCLCPP_ERROR(get_logger(), "ALARM_RESET: %s", response->message.c_str());
    return;
  }

  std::string reason;
  const bool ok = session_manager_->reset_error(reason);
  response->success = ok;
  response->message = ok
    ? (reason.empty() ? "alarm reset succeeded" : reason)
    : (reason.empty() ? "alarm reset failed" : reason);

  if (ok)
  {
    RCLCPP_INFO(get_logger(), "ALARM_RESET: %s", response->message.c_str());
  }
  else
  {
    RCLCPP_WARN(get_logger(), "ALARM_RESET: %s", response->message.c_str());
  }
}

// --- Step 4.2: IoSet service handler ---
// Delegates to MotoROS2 WriteSingleIO if motoros2_interfaces was available at build time.
// If not available, returns a graceful "unavailable" response.
void HwAdapterNode::handle_io_set(
  const std::shared_ptr<IoSet::Request> request,
  std::shared_ptr<IoSet::Response> response)
{
#if HW_ADAPTER_HAS_TOOL_IO_INTERFACES
  if (!tool_state_monitor_ || !tool_state_monitor_->io_services_configured())
  {
    response->success = false;
    response->message =
      "IO_SET unavailable: WriteSingleIO service is not configured in hw_adapter";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Use the write_single_io service name from the tool_state_monitor config
  const auto & svc_names = tool_state_monitor_->snapshot();
  if (!svc_names.write_service_configured)
  {
    response->success = false;
    response->message =
      "IO_SET unavailable: write_single_io service name is empty";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Reuse the write_single_io_service parameter already declared in constructor.
  using WriteSingleIO = motoros2_interfaces::srv::WriteSingleIO;
  std::string write_svc_name;
  get_parameter("write_single_io_service", write_svc_name);
  if (write_svc_name.empty())
  {
    response->success = false;
    response->message =
      "IO_SET unavailable: write_single_io_service parameter is empty";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  auto write_client = create_client<WriteSingleIO>(write_svc_name);

  // If the parameter was empty or the service isn't ready, fail gracefully
  if (!write_client || !write_client->wait_for_service(std::chrono::seconds(3)))
  {
    response->success = false;
    response->message =
      "IO_SET unavailable: WriteSingleIO service is not reachable";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto io_request = std::make_shared<WriteSingleIO::Request>();
  io_request->address = request->address;
  io_request->value = request->value;

  auto future = write_client->async_send_request(io_request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    response->success = false;
    response->message =
      "IO_SET timed out waiting for WriteSingleIO response";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto io_response = future.get();
  if (!io_response)
  {
    response->success = false;
    response->message = "IO_SET: WriteSingleIO returned null response";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->success = io_response->success;
  response->message = io_response->success
    ? "IO_SET: address=" + std::to_string(request->address) +
      " value=" + std::to_string(request->value) + " written successfully"
    : "IO_SET failed: " + io_response->message;

  if (response->success)
  {
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }
  else
  {
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }
#else
  // motoros2_interfaces was not available at build time — graceful unavailable
  (void)request;
  response->success = false;
  response->message =
    "IO_SET unavailable: motoros2_interfaces was not found at build time; "
    "rebuild hw_adapter with motoros2_interfaces installed to enable IO control";
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
#endif
}
}  // namespace hw_adapter
