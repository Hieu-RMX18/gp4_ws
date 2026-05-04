// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#if __has_include(<motoros2_interfaces/srv/read_single_io.hpp>) && \
  __has_include(<motoros2_interfaces/srv/write_single_io.hpp>)
#include <motoros2_interfaces/srv/read_single_io.hpp>
#include <motoros2_interfaces/srv/write_single_io.hpp>
#define HW_ADAPTER_HAS_TOOL_IO_INTERFACES 1
#else
#define HW_ADAPTER_HAS_TOOL_IO_INTERFACES 0
#endif

namespace hw_adapter {
struct ToolServiceNames {
  std::string read_single_io;
  std::string write_single_io;
};

struct ToolState {
  uint32_t address = 0U;
  int32_t raw_value = 0;
  bool active = false;
  std::string detail = "tool IO state available";
};

struct ToolStateSnapshot {
  bool motoros2_interfaces_available = false;
  bool read_service_configured = false;
  bool write_service_configured = false;
  bool has_state = false;
  bool output_state = false;
  uint32_t address = 0U;
  std::string status_message = "tool IO is not configured";
};

class ToolStateMonitor {
public:
  explicit ToolStateMonitor(
      rclcpp::Node &node, ToolServiceNames service_names = {},
      uint32_t tool_io_address = 0U,
      std::chrono::milliseconds poll_period = std::chrono::milliseconds(250));

  ToolStateSnapshot snapshot() const;
  bool io_services_configured() const;
  bool has_tool_state() const;
  std::optional<ToolState> current_tool_state() const;

private:
#if HW_ADAPTER_HAS_TOOL_IO_INTERFACES
  using ReadSingleIO = motoros2_interfaces::srv::ReadSingleIO;
  using WriteSingleIO = motoros2_interfaces::srv::WriteSingleIO;
  using ReadSingleIOClient = rclcpp::Client<ReadSingleIO>;
#endif

  void polling_timer_callback();
#if HW_ADAPTER_HAS_TOOL_IO_INTERFACES
  void handle_read_response(uint64_t request_sequence,
                            ReadSingleIOClient::SharedFuture future);
#endif
  void set_unknown_state(const std::string &status_message);

  rclcpp::Logger logger_;
  ToolServiceNames service_names_;
  uint32_t tool_io_address_;
  std::chrono::milliseconds poll_period_;
  std::chrono::milliseconds request_timeout_;

  mutable std::mutex state_mutex_;
  ToolStateSnapshot snapshot_;
  std::optional<ToolState> current_tool_state_;
  bool request_in_flight_ = false;
  std::optional<int64_t> pending_request_id_;
  std::optional<uint64_t> pending_request_sequence_;
  uint64_t next_request_sequence_ = 0U;
  std::chrono::steady_clock::time_point pending_request_started_at_{};

#if HW_ADAPTER_HAS_TOOL_IO_INTERFACES
  ReadSingleIOClient::SharedPtr read_client_;
  rclcpp::Client<WriteSingleIO>::SharedPtr write_client_;
#endif
  rclcpp::TimerBase::SharedPtr poll_timer_;
};
} // namespace hw_adapter
