// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <industrial_msgs/msg/robot_status.hpp>
#include <interfaces/msg/robot_readiness.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hw_adapter {
struct RobotStatusSnapshot {
  bool has_status = false;
  bool ready = false;
  bool e_stopped = false;
  bool drives_powered = false;
  bool motion_possible = false;
  bool in_motion = false;
  bool in_error = false;
  int8_t mode = -1;
  bool fresh = false;
  builtin_interfaces::msg::Time header_stamp;
  std::chrono::milliseconds age{0};
  std::vector<int32_t> error_codes;
  std::string status_message = "unknown: no robot status received";
};

class RobotStatusMonitor {
public:
  explicit RobotStatusMonitor(
      rclcpp::Node &node, std::string topic_name = "/yaskawa/robot_status",
      std::chrono::milliseconds max_age = std::chrono::milliseconds(200));

  RobotStatusSnapshot latest_snapshot() const;
  bool has_status() const;
  bool is_ready() const;
  bool is_estop_active() const;
  std::string status_summary() const;
  interfaces::msg::RobotReadiness readiness_msg() const;
  bool is_ready_for_motion(std::string &reason) const;

private:
  void publish_readiness();
  void status_callback(const industrial_msgs::msg::RobotStatus::SharedPtr msg);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::chrono::milliseconds max_age_;
  rclcpp::Subscription<industrial_msgs::msg::RobotStatus>::SharedPtr
      status_sub_;
  rclcpp::Publisher<interfaces::msg::RobotReadiness>::SharedPtr readiness_pub_;

  mutable std::mutex snapshot_mutex_;
  bool has_status_{false};
  rclcpp::Time receive_time_{0, 0, RCL_ROS_TIME};
  RobotStatusSnapshot snapshot_;
};
} // namespace hw_adapter
