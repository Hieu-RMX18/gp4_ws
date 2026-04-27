// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/hw_adapter_node.hpp"

#include <atomic>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace hw_adapter
{
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
  cleanup_dispatch_workers();
  std::future<void> worker = std::async(
    std::launch::async,
    [this, goal_handle]()
    {
      if (shutdown_requested_.load())
      {
        return;
      }
      execute_dispatch(goal_handle);
    });
  std::lock_guard<std::mutex> lock(dispatch_worker_mutex_);
  dispatch_worker_futures_.emplace_back(std::move(worker));
}

void HwAdapterNode::cleanup_dispatch_workers()
{
  std::lock_guard<std::mutex> lock(dispatch_worker_mutex_);
  auto it = dispatch_worker_futures_.begin();
  while (it != dispatch_worker_futures_.end())
  {
    if (it->valid() && it->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      try
      {
        it->get();
      }
      catch (const std::exception & ex)
      {
        RCLCPP_ERROR(get_logger(), "Dispatch worker ended with exception: %s", ex.what());
      }
      catch (...)
      {
        RCLCPP_ERROR(get_logger(), "Dispatch worker ended with unknown exception.");
      }
      it = dispatch_worker_futures_.erase(it);
      continue;
    }
    ++it;
  }
}

void HwAdapterNode::wait_for_dispatch_workers()
{
  std::vector<std::future<void>> workers;
  {
    std::lock_guard<std::mutex> lock(dispatch_worker_mutex_);
    workers.swap(dispatch_worker_futures_);
  }
  for (auto & worker : workers)
  {
    if (!worker.valid())
    {
      continue;
    }
    try
    {
      worker.get();
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(get_logger(), "Dispatch worker join failed: %s", ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(get_logger(), "Dispatch worker join failed with unknown exception.");
    }
  }
}

void HwAdapterNode::execute_dispatch(
  const std::shared_ptr<GoalHandleDispatchTrajectory> goal_handle)
{
  if (shutdown_requested_.load())
  {
    return;
  }

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
    result->failure_stage = "dispatch_canceled";
    result->controller_error_code = 0;
    result->controller_error_name = "SUCCESSFUL";
    result->controller_error_string.clear();
    result->max_start_state_abs_delta = 0.0;
    result->start_state_l2_delta = 0.0;
    feedback->state = "canceled";
    goal_handle->publish_feedback(feedback);
    goal_handle->canceled(result);
    return;
  }

  std::atomic<bool> execution_finished{false};
  std::atomic<bool> cancel_stop_requested{false};
  std::thread cancel_watcher([this, goal_handle, &execution_finished, &cancel_stop_requested, dispatch_goal_id]() {
    while (!execution_finished.load() && !shutdown_requested_.load())
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

  TrajectoryExecutionRequest execution_request;
  execution_request.trajectory = goal->trajectory;
  execution_request.result_timeout = timeout_ms;
  execution_request.command_id = goal->command_id;
  execution_request.primitive = goal->primitive;
  execution_request.planner_id = goal->planner_id;
  execution_request.source_joint_state_stamp = goal->source_joint_state_stamp;
  execution_request.expected_start_positions = goal->expected_start_positions;
  execution_request.enforce_start_state_match = goal->enforce_start_state_match;

  // Delegate to the existing execute_trajectory orchestration.
  const auto report = execute_trajectory_internal(execution_request, true);
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
  result->failure_stage = report.failure_stage;
  result->controller_error_code = report.controller_error_code;
  result->controller_error_name = report.controller_error_name;
  result->controller_error_string = report.controller_error_string;
  result->max_start_state_abs_delta = report.max_start_state_abs_delta;
  result->start_state_l2_delta = report.start_state_l2_delta;

  if (goal_handle->is_canceling() || cancel_stop_requested.load())
  {
    result->success = false;
    result->failure_stage = "dispatch_canceled";
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
}  // namespace hw_adapter
