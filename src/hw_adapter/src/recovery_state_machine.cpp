// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/recovery_state_machine.hpp"

#include <thread>

namespace hw_adapter
{

const char * recovery_state_name(RecoveryState state)
{
  switch (state)
  {
    case RecoveryState::IDLE:                   return "IDLE";
    case RecoveryState::FAILED:                 return "FAILED";
    case RecoveryState::STOPPING:               return "STOPPING";
    case RecoveryState::RESETTING_ERROR:        return "RESETTING_ERROR";
    case RecoveryState::VERIFYING_JOINT_STATE:  return "VERIFYING_JOINT_STATE";
    case RecoveryState::READY:                  return "READY";
    case RecoveryState::RECOVERY_FAILED:        return "RECOVERY_FAILED";
  }
  return "UNKNOWN";
}

RecoveryStateMachine::RecoveryStateMachine(
  rclcpp::Logger logger,
  RecoveryCallbacks callbacks,
  std::chrono::milliseconds step_timeout,
  std::chrono::milliseconds total_timeout)
: logger_(logger),
  callbacks_(std::move(callbacks)),
  step_timeout_(step_timeout),
  total_timeout_(total_timeout)
{
}

RecoveryResult RecoveryStateMachine::execute()
{
  const auto recovery_start = std::chrono::steady_clock::now();
  RecoveryResult result;
  state_ = RecoveryState::FAILED;

  RCLCPP_WARN(logger_,
    "J4-Recovery: starting recovery sequence (step_timeout=%ldms, total_timeout=%ldms)",
    static_cast<long>(step_timeout_.count()),
    static_cast<long>(total_timeout_.count()));

  // Macro-like lambda — check total timeout at each step
  auto total_elapsed_ms = [&]() -> std::chrono::milliseconds {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - recovery_start);
  };

  auto check_total_timeout = [&]() -> bool {
    if (total_elapsed_ms() >= total_timeout_)
    {
      RCLCPP_ERROR(logger_,
        "J4-Recovery: total timeout exceeded at state %s",
        recovery_state_name(state_));
      state_ = RecoveryState::RECOVERY_FAILED;
      result.message = "recovery total timeout exceeded at " +
        std::string(recovery_state_name(state_));
      return true;
    }
    return false;
  };

  // ---- Step 1: STOP ----
  {
    state_ = RecoveryState::STOPPING;
    RCLCPP_INFO(logger_, "J4-Recovery: [1/4] STOPPING — calling stop_motion");
    std::string reason;
    if (!transition_stop(reason))
    {
      // stop_motion failure is non-fatal for recovery:
      // the controller may already be stopped, or stop_motion may not be configured.
      RCLCPP_WARN(logger_,
        "J4-Recovery: stop_motion returned failure: %s (continuing recovery)",
        reason.c_str());
    }
    else
    {
      RCLCPP_INFO(logger_, "J4-Recovery: stop_motion succeeded");
    }
    result.steps_completed = 1;
    if (check_total_timeout()) { goto finalize; }
  }

  // Brief pause between steps to let the controller settle
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // ---- Step 2: RESET_ERROR ----
  {
    state_ = RecoveryState::RESETTING_ERROR;
    RCLCPP_INFO(logger_, "J4-Recovery: [2/4] RESETTING_ERROR — calling reset_error");
    std::string reason;
    if (!transition_reset_error(reason))
    {
      RCLCPP_ERROR(logger_,
        "J4-Recovery: reset_error FAILED: %s", reason.c_str());
      state_ = RecoveryState::RECOVERY_FAILED;
      result.message = "reset_error failed: " + reason;
      goto finalize;
    }
    RCLCPP_INFO(logger_, "J4-Recovery: reset_error succeeded");
    result.steps_completed = 2;
    if (check_total_timeout()) { goto finalize; }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // ---- Step 3: VERIFY_JOINT_STATE ----
  {
    state_ = RecoveryState::VERIFYING_JOINT_STATE;
    RCLCPP_INFO(logger_, "J4-Recovery: [3/4] VERIFYING_JOINT_STATE");
    std::string reason;
    if (!transition_verify_joint_state(reason))
    {
      RCLCPP_ERROR(logger_,
        "J4-Recovery: joint_state verification FAILED: %s", reason.c_str());
      state_ = RecoveryState::RECOVERY_FAILED;
      result.message = "verify_joint_state failed: " + reason;
      goto finalize;
    }
    RCLCPP_INFO(logger_, "J4-Recovery: joint_state verification passed");
    result.steps_completed = 3;
    if (check_total_timeout()) { goto finalize; }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // ---- Step 4: READY check ----
  {
    RCLCPP_INFO(logger_, "J4-Recovery: [4/4] READY check — is_ready_for_motion");
    std::string reason;
    if (!transition_ready_check(reason))
    {
      RCLCPP_ERROR(logger_,
        "J4-Recovery: READY check FAILED: %s", reason.c_str());
      state_ = RecoveryState::RECOVERY_FAILED;
      result.message = "ready check failed after recovery: " + reason;
      goto finalize;
    }
    state_ = RecoveryState::READY;
    result.recovered = true;
    result.steps_completed = 4;
    RCLCPP_INFO(logger_, "J4-Recovery: recovery COMPLETE — robot ready");
  }

finalize:
  result.final_state = state_;
  result.elapsed_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - recovery_start).count();

  if (result.recovered)
  {
    RCLCPP_INFO(logger_,
      "J4-Recovery: SUCCESS in %.2fs (%d/4 steps)",
      result.elapsed_sec, result.steps_completed);
  }
  else
  {
    RCLCPP_ERROR(logger_,
      "J4-Recovery: FAILED at state %s after %.2fs (%d/4 steps): %s",
      recovery_state_name(state_),
      result.elapsed_sec, result.steps_completed,
      result.message.c_str());
  }

  return result;
}

bool RecoveryStateMachine::transition_stop(std::string & reason)
{
  if (!callbacks_.stop_motion)
  {
    reason = "stop_motion callback not configured";
    return false;
  }
  return callbacks_.stop_motion(reason);
}

bool RecoveryStateMachine::transition_reset_error(std::string & reason)
{
  if (!callbacks_.reset_error)
  {
    reason = "reset_error callback not configured";
    return false;
  }
  return callbacks_.reset_error(reason);
}

bool RecoveryStateMachine::transition_verify_joint_state(std::string & reason)
{
  if (!callbacks_.verify_joint_state)
  {
    reason = "verify_joint_state callback not configured";
    return false;
  }

  // Retry with polling up to step_timeout_
  const auto deadline = std::chrono::steady_clock::now() + step_timeout_;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (callbacks_.verify_joint_state(reason))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  reason = "joint state verification timed out after " +
    std::to_string(step_timeout_.count()) + "ms";
  return false;
}

bool RecoveryStateMachine::transition_ready_check(std::string & reason)
{
  if (!callbacks_.is_ready_for_motion)
  {
    reason = "is_ready_for_motion callback not configured";
    return false;
  }

  // Retry with polling up to step_timeout_
  const auto deadline = std::chrono::steady_clock::now() + step_timeout_;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (callbacks_.is_ready_for_motion(reason))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;
}

}  // namespace hw_adapter
