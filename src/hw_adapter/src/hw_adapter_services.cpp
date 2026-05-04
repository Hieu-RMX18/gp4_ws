// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/hw_adapter_node.hpp"

#include <chrono>
#include <string>

namespace hw_adapter {
void HwAdapterNode::handle_alarm_reset(
    const std::shared_ptr<AlarmReset::Request> /*request*/,
    std::shared_ptr<AlarmReset::Response> response) {
  if (!session_manager_) {
    response->success = false;
    response->message = "session_manager not initialized";
    RCLCPP_ERROR(get_logger(), "ALARM_RESET: %s", response->message.c_str());
    return;
  }

  std::string reason;
  const bool ok = session_manager_->reset_error(reason);
  response->success = ok;
  response->message = ok ? (reason.empty() ? "alarm reset succeeded" : reason)
                         : (reason.empty() ? "alarm reset failed" : reason);

  if (ok) {
    RCLCPP_INFO(get_logger(), "ALARM_RESET: %s", response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "ALARM_RESET: %s", response->message.c_str());
  }
}

// --- Step 4.2: IoSet service handler ---
// Delegates to MotoROS2 WriteSingleIO if motoros2_interfaces was available at
// build time. If not available, returns a graceful "unavailable" response.
void HwAdapterNode::handle_io_set(const std::shared_ptr<IoSet::Request> request,
                                  std::shared_ptr<IoSet::Response> response) {
#if HW_ADAPTER_HAS_TOOL_IO_INTERFACES
  if (!tool_state_monitor_ || !tool_state_monitor_->io_services_configured()) {
    response->success = false;
    response->message = "IO_SET unavailable: WriteSingleIO service is not "
                        "configured in hw_adapter";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Use the write_single_io service name from the tool_state_monitor config
  const auto &svc_names = tool_state_monitor_->snapshot();
  if (!svc_names.write_service_configured) {
    response->success = false;
    response->message =
        "IO_SET unavailable: write_single_io service name is empty";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Reuse the write_single_io_service parameter already declared in
  // constructor.
  using WriteSingleIO = motoros2_interfaces::srv::WriteSingleIO;
  std::string write_svc_name;
  get_parameter("write_single_io_service", write_svc_name);
  if (write_svc_name.empty()) {
    response->success = false;
    response->message =
        "IO_SET unavailable: write_single_io_service parameter is empty";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  auto write_client = create_client<WriteSingleIO>(write_svc_name);

  // If the parameter was empty or the service isn't ready, fail gracefully
  if (!write_client ||
      !write_client->wait_for_service(std::chrono::seconds(3))) {
    response->success = false;
    response->message =
        "IO_SET unavailable: WriteSingleIO service is not reachable";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto io_request = std::make_shared<WriteSingleIO::Request>();
  io_request->address = request->address;
  io_request->value = request->value;

  auto future = write_client->async_send_request(io_request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    response->success = false;
    response->message = "IO_SET timed out waiting for WriteSingleIO response";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto io_response = future.get();
  if (!io_response) {
    response->success = false;
    response->message = "IO_SET: WriteSingleIO returned null response";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->success = io_response->success;
  response->message =
      io_response->success
          ? "IO_SET: address=" + std::to_string(request->address) +
                " value=" + std::to_string(request->value) +
                " written successfully"
          : "IO_SET failed: " + io_response->message;

  if (response->success) {
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }
#else
  // motoros2_interfaces was not available at build time — graceful unavailable
  (void)request;
  response->success = false;
  response->message =
      "IO_SET unavailable: motoros2_interfaces was not found at build time; "
      "rebuild hw_adapter with motoros2_interfaces installed to enable IO "
      "control";
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
#endif
}
} // namespace hw_adapter
