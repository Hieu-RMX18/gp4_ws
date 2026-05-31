// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hw_adapter {
struct BackendTransportLimits {
  std::size_t max_trajectory_points = 200;
  bool blocking_execution_only = true;
};

struct BackendNames {
  std::string planning_group = "gp4_arm";
  std::string trajectory_action = "/yaskawa/follow_joint_trajectory";
  std::string robot_status_topic = "/yaskawa/robot_status";
  std::string joint_states_topic = "/yaskawa/joint_states";
};

struct BackendCapabilitiesSnapshot {
  std::string robot_model = "Yaskawa Motoman GP4";
  std::string controller_model = "YRC1000micro";
  std::vector<std::string> joint_names;
  BackendTransportLimits transport_limits;
  BackendNames names;
  bool requires_micro_ros_agent = true;
  bool fastdds_required = true;
  bool effort_feedback_zero_expected = true;
  bool open_loop_control_supported = true;
  bool supports_async_execution = false;
  bool motoros2_interfaces_available = false;
};

class BackendCapabilities {
public:
  BackendCapabilities();
  explicit BackendCapabilities(rclcpp::Node &node);

  const BackendCapabilitiesSnapshot &snapshot() const;
  bool supports_async_motion() const;
  bool expects_zero_effort_feedback() const;
  bool supports_open_loop_control() const;
  std::string controller_variant() const;
  bool validate_joint_names(const std::vector<std::string> &joint_names,
                            std::string &reason) const;
  bool validate_joint_state(const sensor_msgs::msg::JointState &joint_state,
                            std::string &reason) const;

private:
  void initialize_defaults();
  void detect_runtime_properties(rclcpp::Node &node);

  BackendCapabilitiesSnapshot snapshot_;
};
} // namespace hw_adapter
