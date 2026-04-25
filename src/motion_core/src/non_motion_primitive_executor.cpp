#include "motion_core/non_motion_primitive_executor.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace motion_core
{
NonMotionPrimitiveExecutor::NonMotionPrimitiveExecutor(
  rclcpp::Logger logger,
  ExecutionOrchestrator & execution_orchestrator,
  rclcpp_action::Client<DispatchTrajectory>::SharedPtr dispatch_client,
  rclcpp::Client<AlarmReset>::SharedPtr alarm_reset_client,
  std::string alarm_reset_service_name,
  rclcpp::Client<IoSet>::SharedPtr io_set_client,
  std::string io_set_service_name,
  std::function<void()> stop_move_group_fn)
: logger_(logger),
  execution_orchestrator_(execution_orchestrator),
  dispatch_client_(std::move(dispatch_client)),
  alarm_reset_client_(std::move(alarm_reset_client)),
  alarm_reset_service_name_(std::move(alarm_reset_service_name)),
  io_set_client_(std::move(io_set_client)),
  io_set_service_name_(std::move(io_set_service_name)),
  stop_move_group_fn_(std::move(stop_move_group_fn))
{
}

void NonMotionPrimitiveExecutor::set_result_timing(
  const std::chrono::steady_clock::time_point & started_at,
  ExecuteMotion::Result & result)
{
  const auto ended_at = std::chrono::steady_clock::now();
  result.execution_time_sec =
    std::chrono::duration_cast<std::chrono::duration<double>>(ended_at - started_at).count();
}

bool NonMotionPrimitiveExecutor::handle_stop_primitive(
  const std::string & primitive,
  const std::string & goal_id,
  const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
  const std::chrono::steady_clock::time_point & started_at)
{
  if (primitive != "STOP")
  {
    return false;
  }

  std::string stop_reason;
  const bool had_active_goal = execution_orchestrator_.request_stop(stop_reason);
  RCLCPP_WARN(
    logger_,
    "STOP primitive received for goal_id=%s — %s",
    goal_id.c_str(),
    had_active_goal ? stop_reason.c_str() : "no active execute_motion goal to stop");

  if (dispatch_client_)
  {
    dispatch_client_->async_cancel_all_goals();
  }
  if (stop_move_group_fn_)
  {
    stop_move_group_fn_();
  }

  auto result = std::make_shared<ExecuteMotion::Result>();
  result->success = true;
  result->message = had_active_goal ?
    ("STOP: motion halt requested, dispatch cancel issued (" + stop_reason + ")") :
    "STOP: no active execute_motion goal was running";
  set_result_timing(started_at, *result);
  goal_handle->succeed(result);
  return true;
}

bool NonMotionPrimitiveExecutor::handle_non_motion_primitive(
  const std::string & primitive,
  const std::uint64_t goal_sequence,
  const std::shared_ptr<const ExecuteMotion::Goal> & goal,
  const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
  const std::chrono::steady_clock::time_point & started_at,
  const PublishFeedbackFn & publish_feedback,
  const MessageFn & abort_with_message,
  const MessageFn & cancel_with_message)
{
  if (primitive == "SET_SPEED")
  {
    execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "set_speed");
    const double requested_scale = goal->velocity_scale;
    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = true;
    std::ostringstream msg;
    msg << "SET_SPEED acknowledged: goal_seq=" << goal_sequence
        << ", velocity_scale=" << requested_scale
        << ". NOTE: this is stateless — subsequent motion commands must "
           "include their own velocity_scale field to take effect.";
    result->message = msg.str();
    set_result_timing(started_at, *result);
    RCLCPP_INFO(logger_, "%s", result->message.c_str());
    goal_handle->succeed(result);
    return true;
  }

  if (primitive == "WAIT")
  {
    execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "wait");
    const double wait_sec = std::max(goal->wait_duration_sec, 0.0);
    RCLCPP_INFO(
      logger_,
      "WAIT goal_seq=%lu: pausing for %.3f seconds.",
      static_cast<unsigned long>(goal_sequence),
      wait_sec);

    if (publish_feedback)
    {
      publish_feedback(0.1, "wait_started");
    }

    const auto wait_start = std::chrono::steady_clock::now();
    const auto wait_duration = std::chrono::duration<double>(wait_sec);
    constexpr auto poll_interval = std::chrono::milliseconds(50);

    while (true)
    {
      const auto elapsed = std::chrono::steady_clock::now() - wait_start;
      if (elapsed >= wait_duration)
      {
        break;
      }
      if (goal_handle->is_canceling())
      {
        if (cancel_with_message)
        {
          cancel_with_message("WAIT: cancelled during wait");
        }
        return true;
      }
      if (execution_orchestrator_.stop_requested(goal_sequence))
      {
        if (cancel_with_message)
        {
          cancel_with_message("WAIT: STOP requested during wait");
        }
        return true;
      }

      if (publish_feedback)
      {
        const double progress = std::min(
          0.1 + 0.8 * (std::chrono::duration<double>(elapsed).count() / wait_sec),
          0.9);
        publish_feedback(progress, "waiting");
      }
      std::this_thread::sleep_for(poll_interval);
    }

    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = true;
    result->message = "WAIT: completed " + std::to_string(wait_sec) + " seconds";
    set_result_timing(started_at, *result);
    goal_handle->succeed(result);
    return true;
  }

  if (primitive == "ALARM_RESET")
  {
    execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "alarm_reset");
    RCLCPP_INFO(
      logger_,
      "ALARM_RESET goal_seq=%lu: sending reset request to %s",
      static_cast<unsigned long>(goal_sequence),
      alarm_reset_service_name_.c_str());

    if (!alarm_reset_client_ || !alarm_reset_client_->wait_for_service(std::chrono::seconds(5)))
    {
      if (abort_with_message)
      {
        abort_with_message("ALARM_RESET: service unavailable at " + alarm_reset_service_name_);
      }
      return true;
    }

    auto request = std::make_shared<AlarmReset::Request>();
    auto future = alarm_reset_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      if (abort_with_message)
      {
        abort_with_message(
          "ALARM_RESET: timed out waiting for response from " + alarm_reset_service_name_);
      }
      return true;
    }

    auto response = future.get();
    if (!response)
    {
      if (abort_with_message)
      {
        abort_with_message("ALARM_RESET: empty response from " + alarm_reset_service_name_);
      }
      return true;
    }

    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = response->success;
    result->message = response->success ?
      ("ALARM_RESET: " + response->message) :
      ("ALARM_RESET failed: " + response->message);
    set_result_timing(started_at, *result);

    if (response->success)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }
    return true;
  }

  if (primitive == "IO_SET")
  {
    execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "io_set");
    RCLCPP_INFO(
      logger_,
      "IO_SET goal_seq=%lu: address=%u, value=%d -> %s",
      static_cast<unsigned long>(goal_sequence),
      goal->io_address,
      goal->io_value,
      io_set_service_name_.c_str());

    if (!io_set_client_ || !io_set_client_->wait_for_service(std::chrono::seconds(5)))
    {
      if (abort_with_message)
      {
        abort_with_message("IO_SET: service unavailable at " + io_set_service_name_);
      }
      return true;
    }

    auto request = std::make_shared<IoSet::Request>();
    request->address = goal->io_address;
    request->value = goal->io_value;
    auto future = io_set_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      if (abort_with_message)
      {
        abort_with_message(
          "IO_SET: timed out waiting for response from " + io_set_service_name_);
      }
      return true;
    }

    auto response = future.get();
    if (!response)
    {
      if (abort_with_message)
      {
        abort_with_message("IO_SET: empty response from " + io_set_service_name_);
      }
      return true;
    }

    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = response->success;
    result->message = response->success ?
      ("IO_SET: " + response->message) :
      ("IO_SET failed: " + response->message);
    set_result_timing(started_at, *result);

    if (response->success)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }
    return true;
  }

  return false;
}
}  // namespace motion_core
