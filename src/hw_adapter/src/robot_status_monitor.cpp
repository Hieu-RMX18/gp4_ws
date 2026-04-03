// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/robot_status_monitor.hpp"

#include <sstream>
#include <utility>

namespace
{
bool tri_state_is_true(const int8_t value)
{
  return value > 0;
}

bool tri_state_is_unknown(const int8_t value)
{
  return value < 0;
}

std::string tri_state_to_string(const int8_t value)
{
  if (tri_state_is_unknown(value))
  {
    return "UNKNOWN";
  }

  return tri_state_is_true(value) ? "TRUE" : "FALSE";
}

std::string robot_mode_to_string(const int8_t value)
{
  switch (value)
  {
    case industrial_msgs::msg::RobotMode::AUTO:
      return "AUTO";
    case industrial_msgs::msg::RobotMode::MANUAL:
      return "MANUAL";
    case industrial_msgs::msg::RobotMode::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

std::string build_status_summary(
  const industrial_msgs::msg::RobotStatus & status,
  const bool ready,
  const std::size_t error_code_count)
{
  std::ostringstream oss;
  oss << (ready ? "ready" : "not ready")
      << ": mode=" << robot_mode_to_string(status.mode.val)
      << ", e_stop=" << tri_state_to_string(status.e_stopped.val)
      << ", drives_powered=" << tri_state_to_string(status.drives_powered.val)
      << ", motion_possible=" << tri_state_to_string(status.motion_possible.val)
      << ", in_motion=" << tri_state_to_string(status.in_motion.val)
      << ", in_error=" << tri_state_to_string(status.in_error.val);
  if (error_code_count > 0U)
  {
    oss << ", error_codes=" << error_code_count;
  }
  return oss.str();
}
}  // namespace

namespace hw_adapter
{
RobotStatusMonitor::RobotStatusMonitor(rclcpp::Node & node, std::string topic_name)
: logger_(node.get_logger())
{
  readiness_pub_ = node.create_publisher<interfaces::msg::RobotReadiness>(
    "/hw_adapter/ready",
    rclcpp::QoS(1).reliable().transient_local());
  status_sub_ = node.create_subscription<industrial_msgs::msg::RobotStatus>(
    std::move(topic_name),
    rclcpp::SensorDataQoS(),
    std::bind(&RobotStatusMonitor::status_callback, this, std::placeholders::_1));
  publish_readiness();
}

RobotStatusSnapshot RobotStatusMonitor::latest_snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

bool RobotStatusMonitor::has_status() const
{
  return latest_snapshot().has_status;
}

bool RobotStatusMonitor::is_ready() const
{
  return latest_snapshot().ready;
}

bool RobotStatusMonitor::is_estop_active() const
{
  return latest_snapshot().e_stopped;
}

std::string RobotStatusMonitor::status_summary() const
{
  return latest_snapshot().status_message;
}

interfaces::msg::RobotReadiness RobotStatusMonitor::readiness_msg() const
{
  const auto snapshot = latest_snapshot();
  interfaces::msg::RobotReadiness message;
  message.ready = snapshot.ready;
  message.status_message = snapshot.status_message;
  return message;
}

bool RobotStatusMonitor::is_ready_for_motion(std::string & reason) const
{
  if (is_ready())
  {
    reason.clear();
    return true;
  }

  reason = status_summary();
  return false;
}

void RobotStatusMonitor::publish_readiness()
{
  readiness_pub_->publish(readiness_msg());
}

void RobotStatusMonitor::status_callback(const industrial_msgs::msg::RobotStatus::SharedPtr msg)
{
  if (!msg)
  {
    RCLCPP_WARN(logger_, "Received null RobotStatus message.");
    return;
  }

  RobotStatusSnapshot next_snapshot;
  next_snapshot.has_status = true;
  next_snapshot.e_stopped = tri_state_is_true(msg->e_stopped.val);
  next_snapshot.drives_powered = tri_state_is_true(msg->drives_powered.val);
  next_snapshot.motion_possible = tri_state_is_true(msg->motion_possible.val);
  next_snapshot.in_motion = tri_state_is_true(msg->in_motion.val);
  next_snapshot.in_error = tri_state_is_true(msg->in_error.val);
  next_snapshot.mode = msg->mode.val;
  next_snapshot.error_codes = msg->error_codes;
  const bool mode_is_auto = msg->mode.val == industrial_msgs::msg::RobotMode::AUTO;
  const bool tri_state_known =
    !tri_state_is_unknown(msg->e_stopped.val) &&
    !tri_state_is_unknown(msg->drives_powered.val) &&
    !tri_state_is_unknown(msg->motion_possible.val) &&
    !tri_state_is_unknown(msg->in_motion.val) &&
    !tri_state_is_unknown(msg->in_error.val);
  next_snapshot.ready =
    tri_state_known &&
    mode_is_auto &&
    !next_snapshot.e_stopped &&
    next_snapshot.drives_powered &&
    next_snapshot.motion_possible &&
    !next_snapshot.in_error;
  next_snapshot.status_message = build_status_summary(*msg, next_snapshot.ready, next_snapshot.error_codes.size());

  if (tri_state_is_unknown(msg->e_stopped.val) || tri_state_is_unknown(msg->in_error.val))
  {
    next_snapshot.status_message +=
      " (contains UNKNOWN safety state, treated as not ready)";
  }
  else if (tri_state_is_true(msg->e_stopped.val))
  {
    next_snapshot.status_message += " (E-STOP active)";
  }
  else if (tri_state_is_true(msg->in_error.val))
  {
    next_snapshot.status_message += " (controller error active)";
  }
  else if (!mode_is_auto)
  {
    next_snapshot.status_message += " (robot not in AUTO mode)";
  }

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_ = std::move(next_snapshot);
  }

  publish_readiness();
}
}  // namespace hw_adapter
