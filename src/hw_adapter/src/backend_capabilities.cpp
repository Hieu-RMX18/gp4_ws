// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/backend_capabilities.hpp"

#include <sstream>

namespace
{
#if __has_include(<motoros2_interfaces/srv/start_traj_mode.hpp>)
constexpr bool kMotoros2InterfacesAvailable = true;
#else
constexpr bool kMotoros2InterfacesAvailable = false;
#endif

template<typename T>
T declare_or_get_parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (node.has_parameter(name))
  {
    return node.get_parameter(name).get_value<T>();
  }

  return node.declare_parameter<T>(name, default_value);
}
}  // namespace

namespace hw_adapter
{
BackendCapabilities::BackendCapabilities()
{
  initialize_defaults();
}

BackendCapabilities::BackendCapabilities(rclcpp::Node & node)
{
  initialize_defaults();
  detect_runtime_properties(node);
}

const BackendCapabilitiesSnapshot & BackendCapabilities::snapshot() const
{
  return snapshot_;
}

bool BackendCapabilities::supports_async_motion() const
{
  return snapshot_.supports_async_execution;
}

bool BackendCapabilities::expects_zero_effort_feedback() const
{
  return snapshot_.effort_feedback_zero_expected;
}

bool BackendCapabilities::supports_open_loop_control() const
{
  return snapshot_.open_loop_control_supported;
}

std::string BackendCapabilities::controller_variant() const
{
  return snapshot_.controller_model;
}

bool BackendCapabilities::validate_joint_names(
  const std::vector<std::string> & joint_names,
  std::string & reason) const
{
  if (joint_names == snapshot_.joint_names)
  {
    reason.clear();
    return true;
  }

  std::ostringstream oss;
  oss << "joint name set does not match GP4/YRC1000micro ordering";
  reason = oss.str();
  return false;
}

bool BackendCapabilities::validate_joint_state(
  const sensor_msgs::msg::JointState & joint_state,
  std::string & reason) const
{
  if (!validate_joint_names(joint_state.name, reason))
  {
    return false;
  }

  if (joint_state.position.size() != snapshot_.joint_names.size())
  {
    reason = "joint state position array size does not match GP4 joint count";
    return false;
  }

  reason.clear();
  return true;
}

void BackendCapabilities::initialize_defaults()
{
  snapshot_.joint_names = {
    "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};
  snapshot_.controller_model = "YRC1000micro";
  snapshot_.effort_feedback_zero_expected = true;
  snapshot_.open_loop_control_supported = true;
  snapshot_.supports_async_execution = false;
  snapshot_.motoros2_interfaces_available = kMotoros2InterfacesAvailable;
}

void BackendCapabilities::detect_runtime_properties(rclcpp::Node & node)
{
  snapshot_.controller_model = declare_or_get_parameter<std::string>(
    node, "backend.controller_variant", snapshot_.controller_model);
  snapshot_.effort_feedback_zero_expected = declare_or_get_parameter<bool>(
    node,
    "backend.expects_zero_effort_feedback",
    snapshot_.effort_feedback_zero_expected);
  snapshot_.open_loop_control_supported = declare_or_get_parameter<bool>(
    node,
    "backend.supports_open_loop_control",
    snapshot_.open_loop_control_supported);
  snapshot_.supports_async_execution = declare_or_get_parameter<bool>(
    node,
    "backend.supports_async_motion",
    snapshot_.supports_async_execution);
}
}  // namespace hw_adapter
