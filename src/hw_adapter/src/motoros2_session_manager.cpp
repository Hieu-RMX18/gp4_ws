// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/motoros2_session_manager.hpp"

#include <action_msgs/srv/cancel_goal.hpp>

#include <sstream>
#include <utility>

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

template<typename ServiceT>
bool wait_for_service_or_timeout(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const std::string & service_name,
  const std::chrono::milliseconds timeout,
  std::string & reason)
{
  if (!client)
  {
    reason = "Service client is not configured for '" + service_name + "'";
    return false;
  }

  if (!client->wait_for_service(timeout))
  {
    reason = timeout_reason("service", service_name, timeout);
    return false;
  }

  reason.clear();
  return true;
}

template<typename FutureT>
bool wait_for_future_or_timeout(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const FutureT & future,
  const std::chrono::milliseconds timeout,
  const std::string & contract_name,
  std::string & reason)
{
  const auto result = rclcpp::spin_until_future_complete(node_base, future, timeout);
  if (result == rclcpp::FutureReturnCode::SUCCESS)
  {
    reason.clear();
    return true;
  }

  if (result == rclcpp::FutureReturnCode::TIMEOUT)
  {
    reason = timeout_reason("response from", contract_name, timeout);
    return false;
  }

  reason = "Interrupted while waiting for '" + contract_name + "'";
  return false;
}
}  // namespace

namespace hw_adapter
{
Motoros2SessionManager::Motoros2SessionManager(
  rclcpp::Node & node,
  SessionServiceNames service_names,
  std::chrono::milliseconds operation_timeout)
: logger_(node.get_logger()),
  service_names_(std::move(service_names)),
  operation_timeout_(operation_timeout)
{
  rclcpp::NodeOptions client_node_options;
  client_node_options.context(node.get_node_base_interface()->get_context());
  client_node_ = std::make_shared<rclcpp::Node>(
    std::string(node.get_name()) + "_motoros2_session_manager",
    node.get_namespace(),
    client_node_options);

  snapshot_.motoros2_interfaces_available = HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES != 0;

#if HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES
  if (!service_names_.start_traj_mode.empty())
  {
    start_traj_mode_client_ = client_node_->create_client<StartTrajMode>(service_names_.start_traj_mode);
  }
  if (!service_names_.reset_error.empty())
  {
    reset_error_client_ = client_node_->create_client<ResetError>(service_names_.reset_error);
  }
#endif

#if HW_ADAPTER_HAS_STOP_MOTION_SERVICE
  if (!service_names_.stop_motion.empty())
  {
    stop_motion_client_ = client_node_->create_client<StopMotion>(service_names_.stop_motion);
  }
#endif

  if (!service_names_.follow_joint_trajectory_action.empty())
  {
    follow_joint_trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      client_node_,
      service_names_.follow_joint_trajectory_action);
  }

  snapshot_.start_traj_mode_configured =
#if HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES
    start_traj_mode_client_ != nullptr;
#else
    false;
#endif
  snapshot_.reset_error_configured =
#if HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES
    reset_error_client_ != nullptr;
#else
    false;
#endif
  snapshot_.stop_motion_configured =
#if HW_ADAPTER_HAS_STOP_MOTION_SERVICE
    stop_motion_client_ != nullptr || follow_joint_trajectory_client_ != nullptr;
#else
    follow_joint_trajectory_client_ != nullptr;
#endif
  snapshot_.stop_motion_uses_action_cancel =
#if HW_ADAPTER_HAS_STOP_MOTION_SERVICE
    stop_motion_client_ == nullptr && follow_joint_trajectory_client_ != nullptr;
#else
    follow_joint_trajectory_client_ != nullptr;
#endif
  snapshot_.status_message =
    snapshot_.motoros2_interfaces_available ?
    "MotoROS2 session manager initialized; session not ready" :
    "MotoROS2 session services are unavailable in this build environment";
}

SessionManagerSnapshot Motoros2SessionManager::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

bool Motoros2SessionManager::wait_for_required_services(
  std::chrono::milliseconds timeout,
  std::string & reason) const
{
  std::lock_guard<std::mutex> call_lock(call_mutex_);

#if !HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES
  reason = "MotoROS2 session services are unavailable in this build environment";
  return false;
#else
  if (!wait_for_service_or_timeout<StartTrajMode>(
      start_traj_mode_client_,
      service_names_.start_traj_mode,
      timeout,
      reason))
  {
    return false;
  }

  if (!wait_for_service_or_timeout<ResetError>(
      reset_error_client_,
      service_names_.reset_error,
      timeout,
      reason))
  {
    return false;
  }
#endif

#if HW_ADAPTER_HAS_STOP_MOTION_SERVICE
  if (stop_motion_client_ != nullptr &&
    !wait_for_service_or_timeout<StopMotion>(
      stop_motion_client_,
      service_names_.stop_motion,
      timeout,
      reason))
  {
    return false;
  }
#endif

  if (follow_joint_trajectory_client_ != nullptr &&
    !follow_joint_trajectory_client_->wait_for_action_server(timeout))
  {
    reason = timeout_reason(
      "action server",
      service_names_.follow_joint_trajectory_action,
      timeout);
    return false;
  }

  reason.clear();
  return true;
}

bool Motoros2SessionManager::start_traj_mode(std::string & reason)
{
  std::lock_guard<std::mutex> call_lock(call_mutex_);

#if !HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES
  reason = "StartTrajMode is unavailable because motoros2_interfaces was not found at build time";
  update_status_message(reason);
  return false;
#else
  {
    std::lock_guard<std::mutex> state_lock(snapshot_mutex_);
    if (snapshot_.session_ready)
    {
      reason.clear();
      snapshot_.status_message = "Trajectory mode already active for this adapter session";
      return true;
    }
  }

  if (!wait_for_service_or_timeout<StartTrajMode>(
      start_traj_mode_client_,
      service_names_.start_traj_mode,
      operation_timeout_,
      reason))
  {
    update_status_message(reason);
    return false;
  }

  auto request = std::make_shared<StartTrajMode::Request>();
  auto future = start_traj_mode_client_->async_send_request(request);
  if (!wait_for_future_or_timeout(
      client_node_->get_node_base_interface(),
      future,
      operation_timeout_,
      service_names_.start_traj_mode,
      reason))
  {
    update_status_message(reason);
    return false;
  }

  const auto response = future.get();
  if (!response)
  {
    reason = "StartTrajMode returned a null response";
    update_status_message(reason);
    return false;
  }

  if (response->result_code.value == motoros2_interfaces::msg::MotionReadyEnum::READY)
  {
    set_session_ready(true, response->message.empty() ? "Trajectory mode ready" : response->message);
    reason.clear();
    return true;
  }

  std::ostringstream oss;
  oss << "StartTrajMode failed with code " << response->result_code.value;
  if (!response->message.empty())
  {
    oss << ": " << response->message;
  }
  if (response->result_code.value == motoros2_interfaces::msg::MotionReadyEnum::NOT_READY_ESTOP)
  {
    oss << ". E-stop active: prefer stop_motion before retrying start_traj_mode.";
  }
  reason = oss.str();
  update_status_message(reason);
  return false;
#endif
}

bool Motoros2SessionManager::ensure_trajectory_mode(std::string & reason)
{
  return start_traj_mode(reason);
}

bool Motoros2SessionManager::reset_error(std::string & reason)
{
  std::lock_guard<std::mutex> call_lock(call_mutex_);

#if !HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES
  reason = "ResetError is unavailable because motoros2_interfaces was not found at build time";
  update_status_message(reason);
  return false;
#else
  if (!wait_for_service_or_timeout<ResetError>(
      reset_error_client_,
      service_names_.reset_error,
      operation_timeout_,
      reason))
  {
    update_status_message(reason);
    return false;
  }

  auto request = std::make_shared<ResetError::Request>();
  auto future = reset_error_client_->async_send_request(request);
  if (!wait_for_future_or_timeout(
      client_node_->get_node_base_interface(),
      future,
      operation_timeout_,
      service_names_.reset_error,
      reason))
  {
    update_status_message(reason);
    return false;
  }

  const auto response = future.get();
  if (!response)
  {
    reason = "ResetError returned a null response";
    update_status_message(reason);
    return false;
  }

  if (response->result_code.value == motoros2_interfaces::msg::MotionReadyEnum::READY)
  {
    set_session_ready(false, response->message.empty() ? "ResetError succeeded" : response->message);
    reason.clear();
    return true;
  }

  std::ostringstream oss;
  oss << "ResetError failed with code " << response->result_code.value;
  if (!response->message.empty())
  {
    oss << ": " << response->message;
  }
  reason = oss.str();
  update_status_message(reason);
  return false;
#endif
}

bool Motoros2SessionManager::stop_motion(std::string & reason)
{
#if HW_ADAPTER_HAS_STOP_MOTION_SERVICE
  {
    std::lock_guard<std::mutex> call_lock(call_mutex_);

    if (stop_motion_client_ != nullptr)
    {
      if (!wait_for_service_or_timeout<StopMotion>(
          stop_motion_client_,
          service_names_.stop_motion,
          operation_timeout_,
          reason))
      {
        update_status_message(reason);
        return false;
      }

      auto request = std::make_shared<StopMotion::Request>();
      auto future = stop_motion_client_->async_send_request(request);
      if (!wait_for_future_or_timeout(
          client_node_->get_node_base_interface(),
          future,
          operation_timeout_,
          service_names_.stop_motion,
          reason))
      {
        update_status_message(reason);
        return false;
      }

      const auto response = future.get();
      if (!response)
      {
        reason = "StopMotion returned a null response";
        update_status_message(reason);
        return false;
      }

      if (response->code.val == industrial_msgs::msg::ServiceReturnCode::SUCCESS)
      {
        set_session_ready(false, "StopMotion succeeded");
        reason.clear();
        return true;
      }

      std::ostringstream oss;
      oss << "StopMotion failed with return code " << static_cast<int>(response->code.val);
      reason = oss.str();
      update_status_message(reason);
      return false;
    }
  }
#endif

  std::lock_guard<std::mutex> call_lock(call_mutex_);
  if (follow_joint_trajectory_client_ == nullptr)
  {
    reason =
      "stop_motion is unavailable: no StopMotion service client and no FollowJointTrajectory action "
      "client are configured";
    update_status_message(reason);
    return false;
  }

  if (!follow_joint_trajectory_client_->wait_for_action_server(operation_timeout_))
  {
    reason = timeout_reason(
      "action server",
      service_names_.follow_joint_trajectory_action,
      operation_timeout_);
    update_status_message(reason);
    return false;
  }

  auto future = follow_joint_trajectory_client_->async_cancel_all_goals();
  if (!wait_for_future_or_timeout(
      client_node_->get_node_base_interface(),
      future,
      operation_timeout_,
      service_names_.follow_joint_trajectory_action + "/_action/cancel_goal",
      reason))
  {
    update_status_message(reason);
    return false;
  }

  const auto response = future.get();
  if (!response)
  {
    reason = "FollowJointTrajectory cancel_all_goals returned a null response";
    update_status_message(reason);
    return false;
  }

  if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE ||
    response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_GOAL_TERMINATED)
  {
    set_session_ready(false, "stop_motion completed via FollowJointTrajectory cancel");
    reason.clear();
    return true;
  }

  std::ostringstream oss;
  oss << "stop_motion failed via FollowJointTrajectory cancel with return code "
      << static_cast<int>(response->return_code);
  reason = oss.str();
  update_status_message(reason);
  return false;
}

bool Motoros2SessionManager::is_session_ready() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_.session_ready;
}

void Motoros2SessionManager::update_status_message(const std::string & message)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.status_message = message;
}

void Motoros2SessionManager::set_session_ready(const bool ready, const std::string & message)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.session_ready = ready;
  snapshot_.status_message = message;
}
}  // namespace hw_adapter
