// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/trajectory_executor.hpp"

#include <future>
#include <sstream>
#include <utility>

namespace hw_adapter
{
namespace
{
std::string timeout_reason(
  const std::string & contract_type,
  const std::string & contract_name,
  const std::chrono::milliseconds timeout)
{
  std::ostringstream oss;
  oss << "Timed out after " << timeout.count() << " ms waiting for " << contract_type << " '"
      << contract_name << "'";
  return oss.str();
}
}  // namespace

TrajectoryExecutor::TrajectoryExecutor(
  rclcpp::Node & node,
  std::string action_name,
  ReadyCheck robot_ready_check,
  ReadyCheck session_ready_check,
  std::chrono::milliseconds default_timeout)
: logger_(node.get_logger()),
  action_name_(std::move(action_name)),
  robot_ready_check_(std::move(robot_ready_check)),
  session_ready_check_(std::move(session_ready_check)),
  default_timeout_(default_timeout)
{
  rclcpp::NodeOptions client_node_options;
  client_node_options.context(node.get_node_base_interface()->get_context());
  client_node_ = std::make_shared<rclcpp::Node>(
    std::string(node.get_name()) + "_trajectory_executor",
    node.get_namespace(),
    client_node_options);

  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(client_node_, action_name_);
}

const std::string & TrajectoryExecutor::action_name() const
{
  return action_name_;
}

bool TrajectoryExecutor::wait_for_server(std::chrono::milliseconds timeout) const
{
  return action_client_->wait_for_action_server(timeout);
}

bool TrajectoryExecutor::validate_trajectory_request(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::string & reason) const
{
  return validate_trajectory(traj, reason);
}

bool TrajectoryExecutor::execute(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::string & reason)
{
  return execute_with_timeout(traj, std::chrono::duration<double>(default_timeout_).count(), reason);
}

bool TrajectoryExecutor::execute_with_timeout(
  const trajectory_msgs::msg::JointTrajectory & traj,
  const double timeout_sec,
  std::string & reason)
{
  if (timeout_sec <= 0.0)
  {
    reason = "timeout_sec must be greater than zero";
    return false;
  }

  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(timeout_sec));
  const auto result = execute_blocking(TrajectoryExecutionRequest{traj, timeout});
  reason = result.message;
  return result.success;
}

TrajectoryExecutionResult TrajectoryExecutor::execute_blocking(
  const TrajectoryExecutionRequest & request)
{
  TrajectoryExecutionResult result;
  if (request.result_timeout.count() <= 0)
  {
    result.message = "result_timeout must be greater than zero";
    result.error_code = 1;
    return result;
  }

  if (!validate_trajectory(request.trajectory, result.message))
  {
    result.error_code = 1;
    return result;
  }

  if (!validate_readiness(result.message))
  {
    result.error_code = 1;
    return result;
  }

  if (!wait_for_server(request.result_timeout))
  {
    result.message = timeout_reason("action server", action_name_, request.result_timeout);
    result.error_code = 1;
    return result;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = request.trajectory;

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [](rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr,
      const std::shared_ptr<const FollowJointTrajectory::Feedback>)
    {
      // MotoROS2 may report zero effort; feedback is informational and not treated as failure.
    };

  auto goal_future = action_client_->async_send_goal(goal, send_goal_options);
  const auto goal_future_state = rclcpp::spin_until_future_complete(
    client_node_->get_node_base_interface(), goal_future, request.result_timeout);
  if (goal_future_state != rclcpp::FutureReturnCode::SUCCESS)
  {
    result.message = goal_future_state == rclcpp::FutureReturnCode::TIMEOUT ?
      timeout_reason("goal response from", action_name_, request.result_timeout) :
      "Interrupted while waiting for goal response from '" + action_name_ + "'";
    result.error_code = 1;
    return result;
  }

  const auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    result.message = "FollowJointTrajectory goal was rejected by '" + action_name_ + "'";
    result.error_code = 1;
    return result;
  }

  result.accepted = true;

  auto result_future = action_client_->async_get_result(goal_handle);
  const auto result_future_state = rclcpp::spin_until_future_complete(
    client_node_->get_node_base_interface(), result_future, request.result_timeout);
  if (result_future_state != rclcpp::FutureReturnCode::SUCCESS)
  {
    result.message = result_future_state == rclcpp::FutureReturnCode::TIMEOUT ?
      timeout_reason("result from", action_name_, request.result_timeout) :
      "Interrupted while waiting for result from '" + action_name_ + "'";
    result.error_code = 1;
    return result;
  }

  result.completed = true;

  const auto wrapped_result = result_future.get();
  if (!wrapped_result.result)
  {
    result.message = "FollowJointTrajectory returned a null result";
    result.error_code = 1;
    return result;
  }

  result.error_code = wrapped_result.result->error_code;

  switch (wrapped_result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (wrapped_result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL)
      {
        result.success = true;
        result.message.clear();
        return result;
      }
      result.message = "FollowJointTrajectory reported error_code " +
        std::to_string(wrapped_result.result->error_code);
      if (!wrapped_result.result->error_string.empty())
      {
        result.message += ": " + wrapped_result.result->error_string;
      }
      return result;

    case rclcpp_action::ResultCode::ABORTED:
      result.message = "FollowJointTrajectory goal aborted";
      if (!wrapped_result.result->error_string.empty())
      {
        result.message += ": " + wrapped_result.result->error_string;
      }
      return result;

    case rclcpp_action::ResultCode::CANCELED:
      result.message = "FollowJointTrajectory goal canceled";
      if (!wrapped_result.result->error_string.empty())
      {
        result.message += ": " + wrapped_result.result->error_string;
      }
      return result;

    default:
      result.message = "FollowJointTrajectory returned an unknown result code";
      if (result.error_code == FollowJointTrajectory::Result::SUCCESSFUL)
      {
        result.error_code = 1;
      }
      return result;
  }

  return result;
}

bool TrajectoryExecutor::validate_trajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::string & reason) const
{
  if (traj.points.empty())
  {
    reason = "trajectory is empty";
    return false;
  }

  if (traj.joint_names.empty())
  {
    reason = "trajectory has no joint names";
    return false;
  }

  if (traj.points.size() > kMaxTrajectoryPoints)
  {
    reason =
      "trajectory has " + std::to_string(traj.points.size()) +
      " points; MotoROS2 YRC1000micro limit is 200 and downsampling must happen earlier";
    return false;
  }

  reason.clear();
  return true;
}

bool TrajectoryExecutor::validate_readiness(std::string & reason) const
{
  if (robot_ready_check_ && !robot_ready_check_(reason))
  {
    if (reason.empty())
    {
      reason = "robot_status_monitor reported not ready";
    }
    return false;
  }

  if (session_ready_check_ && !session_ready_check_(reason))
  {
    if (reason.empty())
    {
      reason = "motoros2_session_manager reported session not ready";
    }
    return false;
  }

  reason.clear();
  return true;
}
}  // namespace hw_adapter
