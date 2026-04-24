// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hw_adapter
{
struct JointStateSnapshot
{
  bool has_message = false;
  bool fresh = false;
  bool valid = false;
  builtin_interfaces::msg::Time header_stamp;
  std::chrono::milliseconds age{0};
  std::vector<double> ordered_positions;
  std::string status_message = "unknown: no joint state received";
};

class JointStateMonitor
{
public:
  explicit JointStateMonitor(
    rclcpp::Node & node,
    std::vector<std::string> expected_joint_names,
    std::string topic_name = "/joint_states",
    std::chrono::milliseconds max_age = std::chrono::milliseconds(200));

  JointStateSnapshot latest_snapshot() const;

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::vector<std::string> expected_joint_names_;
  std::chrono::milliseconds max_age_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  mutable std::mutex snapshot_mutex_;
  bool has_message_{false};
  rclcpp::Time receive_time_{0, 0, RCL_ROS_TIME};
  JointStateSnapshot snapshot_;
};
}  // namespace hw_adapter
