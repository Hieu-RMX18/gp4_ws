// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "supervisor/audit_logger.hpp"
#include "supervisor/execution_monitor.hpp"

namespace
{
diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("supervisor_node");
  auto heartbeat_rate_hz = node->declare_parameter<double>("heartbeat_rate_hz", 1.0);
  if (heartbeat_rate_hz <= 0.0)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "heartbeat_rate_hz must be > 0.0, falling back to 1.0 Hz");
    heartbeat_rate_hz = 1.0;
  }

  supervisor::AuditLogger audit_logger(*node);
  supervisor::ExecutionMonitor execution_monitor(*node);
  auto diagnostics_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    rclcpp::QoS(10).reliable());
  auto heartbeat_timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / heartbeat_rate_hz)),
    [node, diagnostics_pub, &audit_logger, &execution_monitor]() {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = node->now();

      const auto execution_snapshot = execution_monitor.snapshot();

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.level = execution_snapshot.last_alert_level;
      status.name = "/supervisor/heartbeat";
      status.message = execution_snapshot.last_alert_message.empty() ?
        std::string("alive") :
        execution_snapshot.last_alert_message;
      status.hardware_id = "gp4_yrc1000micro";
      status.values.reserve(5);
      status.values.push_back(make_key_value("state", execution_snapshot.current_state));
      status.values.push_back(make_key_value(
          "consecutive_failure_count",
          std::to_string(execution_snapshot.consecutive_failure_count)));
      status.values.push_back(make_key_value(
          "timeout_alert_active",
          execution_snapshot.timeout_alert_active ? "true" : "false"));
      status.values.push_back(make_key_value(
          "audit_written_messages",
          std::to_string(audit_logger.written_message_count())));
      status.values.push_back(make_key_value(
          "audit_received_messages",
          std::to_string(audit_logger.received_message_count())));

      array.status.push_back(std::move(status));
      diagnostics_pub->publish(array);
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  heartbeat_timer.reset();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
