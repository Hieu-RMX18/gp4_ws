// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "motion_core/joint_position_guard.hpp"

namespace hw_adapter {
struct TrajectoryExecutionRequest {
  trajectory_msgs::msg::JointTrajectory trajectory;
  std::chrono::milliseconds result_timeout{std::chrono::seconds(30)};
  std::string command_id;
  std::string primitive;
  std::string planner_id;
  builtin_interfaces::msg::Time source_joint_state_stamp;
  std::vector<double> expected_start_positions;
  bool enforce_start_state_match = false;
  bool extended_mode = false;
};

struct TrajectoryExecutionResult {
  bool accepted = false;
  bool completed = false;
  bool canceled = false;
  bool success = false;
  bool preflight_snapshot_available = false;
  bool commit_snapshot_available = false;
  std::string failure_stage = "none";
  int32_t action_result_code = -1;
  std::string action_result_name = "NOT_SENT";
  int32_t controller_error_code = 0;
  std::string controller_error_name = "SUCCESSFUL";
  std::string controller_error_string;
  double max_start_state_abs_delta = 0.0;
  double start_state_l2_delta = 0.0;
  double commit_drift_max_abs_delta = 0.0;
  double commit_drift_l2_delta = 0.0;
  std::chrono::milliseconds preflight_joint_state_age{0};
  std::chrono::milliseconds commit_joint_state_age{0};
  std::chrono::milliseconds preflight_robot_status_age{0};
  std::chrono::milliseconds commit_robot_status_age{0};
  bool preflight_session_ready = false;
  bool commit_session_ready = false;
  std::string preflight_session_status;
  std::string commit_session_status;
  std::string message = "trajectory not executed";
};

struct ExecutionRuntimeSnapshot {
  bool joint_state_valid = false;
  std::vector<double> current_joint_positions;
  builtin_interfaces::msg::Time joint_state_stamp;
  std::chrono::milliseconds joint_state_age{0};
  bool robot_ready = false;
  std::chrono::milliseconds robot_status_age{0};
  bool session_ready = false;
  std::string session_status;
  std::string failure_reason;
};

class TrajectoryExecutor {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using RuntimeSnapshotProvider = std::function<ExecutionRuntimeSnapshot()>;
  static constexpr std::size_t kMaxTrajectoryPoints = 200;

  explicit TrajectoryExecutor(
      rclcpp::Node &node,
      std::string action_name = "/yaskawa/follow_joint_trajectory",
      RuntimeSnapshotProvider runtime_snapshot_provider = {},
      std::chrono::milliseconds default_timeout = std::chrono::seconds(30),
      std::vector<std::string> canonical_joint_names = {},
      std::chrono::milliseconds trajectory_header_max_age =
          std::chrono::milliseconds(200),
      double start_state_max_abs_delta_rad = 0.01,
      double start_state_max_l2_delta_rad = 0.02,
      motion_core::JointPositionGuard joint_position_guard =
          motion_core::JointPositionGuard{});

  const std::string &action_name() const;
  bool wait_for_server(std::chrono::milliseconds timeout) const;
  bool
  validate_trajectory_request(const trajectory_msgs::msg::JointTrajectory &traj,
                              std::string &reason) const;
  bool execute(const trajectory_msgs::msg::JointTrajectory &traj,
               std::string &reason);
  bool execute_with_timeout(const trajectory_msgs::msg::JointTrajectory &traj,
                            double timeout_sec, std::string &reason);
  TrajectoryExecutionResult
  execute_blocking(const TrajectoryExecutionRequest &request);

private:
  struct StartStateGateResult {
    bool valid = false;
    double max_abs_delta = 0.0;
    double l2_delta = 0.0;
    std::string reason;
  };

  bool validate_trajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                           std::string &reason) const;
  bool validate_runtime_snapshot(const ExecutionRuntimeSnapshot &snapshot,
                                 std::string &reason) const;
  StartStateGateResult
  evaluate_start_state_gate(const std::vector<double> &expected,
                            const std::vector<double> &actual) const;
  void log_dispatch_diagnostics(const TrajectoryExecutionRequest &request,
                                const TrajectoryExecutionResult &result) const;
  static std::string decode_action_result_name(int32_t result_code);
  static std::string decode_controller_error_name(int32_t error_code);

  rclcpp::Logger logger_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::string action_name_;
  RuntimeSnapshotProvider runtime_snapshot_provider_;
  std::chrono::milliseconds default_timeout_;
  std::vector<std::string> canonical_joint_names_;
  std::chrono::milliseconds trajectory_header_max_age_;
  double start_state_max_abs_delta_rad_;
  double start_state_max_l2_delta_rad_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  motion_core::JointPositionGuard joint_position_guard_;
};
} // namespace hw_adapter
