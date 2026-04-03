// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace hw_adapter
{
struct TrajectoryExecutionRequest
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  std::chrono::milliseconds result_timeout{std::chrono::seconds(30)};
};

struct TrajectoryExecutionResult
{
  bool accepted = false;
  bool completed = false;
  bool success = false;
  int32_t error_code = 0;
  std::string message = "trajectory not executed";
};

class TrajectoryExecutor
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using ReadyCheck = std::function<bool(std::string &)>;
  static constexpr std::size_t kMaxTrajectoryPoints = 200;

  explicit TrajectoryExecutor(
    rclcpp::Node & node,
    std::string action_name = "/yaskawa/follow_joint_trajectory",
    ReadyCheck robot_ready_check = {},
    ReadyCheck session_ready_check = {},
    std::chrono::milliseconds default_timeout = std::chrono::seconds(30));

  const std::string & action_name() const;
  bool wait_for_server(std::chrono::milliseconds timeout) const;
  bool validate_trajectory_request(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::string & reason) const;
  bool execute(const trajectory_msgs::msg::JointTrajectory & traj, std::string & reason);
  bool execute_with_timeout(
    const trajectory_msgs::msg::JointTrajectory & traj,
    double timeout_sec,
    std::string & reason);
  TrajectoryExecutionResult execute_blocking(const TrajectoryExecutionRequest & request);

private:
  bool validate_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::string & reason) const;
  bool validate_readiness(std::string & reason) const;

  rclcpp::Logger logger_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::string action_name_;
  ReadyCheck robot_ready_check_;
  ReadyCheck session_ready_check_;
  std::chrono::milliseconds default_timeout_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};
}  // namespace hw_adapter
