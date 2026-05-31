// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
//
// V4 J4-Recovery: deterministic recovery sequence after execution failure.
// State transitions:
//   FAILED → STOP → RESET_ERROR → VERIFY_JOINT_STATE → READY
//
// Each step has an explicit timeout. If any step fails, the machine
// stays in that state and returns the failure reason.
// The caller (hw_adapter_node) is responsible for logging and deciding
// whether to retry or escalate.
#pragma once

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace hw_adapter {
/// Recovery states for the J4-Recovery state machine.
enum class RecoveryState : uint8_t {
  IDLE,                  // No recovery needed
  FAILED,                // Execution failed — entry point
  STOPPING,              // Calling stop_motion
  RESETTING_ERROR,       // Calling reset_error
  VERIFYING_JOINT_STATE, // Waiting for valid /joint_states
  READY,                 // Recovery complete — ready for next execution
  RECOVERY_FAILED        // Unrecoverable — requires operator intervention
};

/// Human-readable state name for logging.
const char *recovery_state_name(RecoveryState state);

/// Callbacks the recovery FSM needs from the hw_adapter subsystems.
struct RecoveryCallbacks {
  /// Stop motion — returns true if stop_motion succeeded.
  std::function<bool(std::string &reason)> stop_motion;

  /// Reset controller error — returns true if reset_error succeeded.
  std::function<bool(std::string &reason)> reset_error;

  /// Verify joint state is fresh — returns true if /joint_states is recently
  /// received.
  std::function<bool(std::string &reason)> verify_joint_state;

  /// Check if robot is ready for motion — returns true if ready.
  std::function<bool(std::string &reason)> is_ready_for_motion;
};

/// Result of a single recovery attempt.
struct RecoveryResult {
  bool recovered = false;
  RecoveryState final_state = RecoveryState::RECOVERY_FAILED;
  std::string message;
  int steps_completed = 0;
  double elapsed_sec = 0.0;
};

class RecoveryStateMachine {
public:
  /// @param logger Logger for all recovery events.
  /// @param callbacks Subsystem callbacks (stop, reset, verify, ready check).
  /// @param step_timeout Maximum time for each individual step.
  /// @param total_timeout Maximum total recovery time before giving up.
  RecoveryStateMachine(
      rclcpp::Logger logger, RecoveryCallbacks callbacks,
      std::chrono::milliseconds step_timeout = std::chrono::seconds(5),
      std::chrono::milliseconds total_timeout = std::chrono::seconds(20));

  /// Execute the full recovery sequence: FAILED → STOP → RESET → VERIFY →
  /// READY. Blocking call. Returns when recovery succeeds or fails.
  RecoveryResult execute();

  /// Current state (for diagnostics).
  RecoveryState current_state() const { return state_; }

private:
  bool transition_stop(std::string &reason);
  bool transition_reset_error(std::string &reason);
  bool transition_verify_joint_state(std::string &reason);
  bool transition_ready_check(std::string &reason);

  rclcpp::Logger logger_;
  RecoveryCallbacks callbacks_;
  std::chrono::milliseconds step_timeout_;
  std::chrono::milliseconds total_timeout_;
  RecoveryState state_{RecoveryState::IDLE};
};
} // namespace hw_adapter
