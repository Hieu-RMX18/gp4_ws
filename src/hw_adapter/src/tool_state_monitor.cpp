// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/tool_state_monitor.hpp"

#include <algorithm>
#include <functional>
#include <sstream>
#include <utility>

#if HW_ADAPTER_HAS_TOOL_IO_INTERFACES
#include <motoros2_interfaces/msg/io_result_codes.hpp>
#endif

namespace
{
std::string unavailable_reason(const std::string & service_name)
{
  return "tool IO read service is unavailable: '" + service_name + "'";
}
}  // namespace

namespace hw_adapter
{
ToolStateMonitor::ToolStateMonitor(
  rclcpp::Node & node,
  ToolServiceNames service_names,
  uint32_t tool_io_address,
  std::chrono::milliseconds poll_period)
: logger_(node.get_logger()),
  service_names_(std::move(service_names)),
  tool_io_address_(tool_io_address),
  poll_period_(poll_period),
  request_timeout_(std::max(std::chrono::milliseconds(250), poll_period * 2))
{
  snapshot_.motoros2_interfaces_available = HW_ADAPTER_HAS_TOOL_IO_INTERFACES != 0;
  snapshot_.read_service_configured = !service_names_.read_single_io.empty();
  snapshot_.write_service_configured = !service_names_.write_single_io.empty();
  snapshot_.address = tool_io_address_;

#if !HW_ADAPTER_HAS_TOOL_IO_INTERFACES
  snapshot_.status_message =
    "tool IO monitoring unavailable: motoros2_interfaces was not found at build time";
  (void)node;
#else
  if (!snapshot_.read_service_configured)
  {
    snapshot_.status_message =
      "tool IO monitoring unavailable: read_single_io service is not configured";
    return;
  }

  if (tool_io_address_ == 0U)
  {
    snapshot_.status_message =
      "tool IO monitoring unavailable: tool_io_address must be greater than zero";
    return;
  }

  if (poll_period_.count() <= 0)
  {
    snapshot_.status_message =
      "tool IO monitoring unavailable: tool_poll_period_ms must be greater than zero";
    return;
  }

  read_client_ = node.create_client<ReadSingleIO>(service_names_.read_single_io);
  if (snapshot_.write_service_configured)
  {
    write_client_ = node.create_client<WriteSingleIO>(service_names_.write_single_io);
  }

  snapshot_.status_message =
    "tool IO monitoring initialized; waiting for first sampled state";

  poll_timer_ = node.create_wall_timer(
    poll_period_,
    std::bind(&ToolStateMonitor::polling_timer_callback, this));
#endif
}

ToolStateSnapshot ToolStateMonitor::snapshot() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return snapshot_;
}

bool ToolStateMonitor::io_services_configured() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return snapshot_.read_service_configured;
}

bool ToolStateMonitor::has_tool_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_tool_state_.has_value();
}

std::optional<ToolState> ToolStateMonitor::current_tool_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_tool_state_;
}

void ToolStateMonitor::polling_timer_callback()
{
#if !HW_ADAPTER_HAS_TOOL_IO_INTERFACES
  return;
#else
  if (!read_client_)
  {
    set_unknown_state("tool IO monitoring unavailable: read client is not initialized");
    return;
  }

  std::optional<int64_t> expired_request_id;
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (request_in_flight_)
    {
      if ((now - pending_request_started_at_) < request_timeout_)
      {
        return;
      }

      expired_request_id = pending_request_id_;
      request_in_flight_ = false;
      pending_request_id_.reset();
      pending_request_sequence_.reset();
      current_tool_state_.reset();
      snapshot_.has_state = false;
      snapshot_.output_state = false;

      std::ostringstream oss;
      oss << "tool IO read timed out after " << request_timeout_.count() << " ms";
      snapshot_.status_message = oss.str();
    }
  }

  if (expired_request_id.has_value())
  {
    read_client_->remove_pending_request(*expired_request_id);
  }

  if (!read_client_->service_is_ready())
  {
    set_unknown_state(unavailable_reason(service_names_.read_single_io));
    return;
  }

  auto request = std::make_shared<ReadSingleIO::Request>();
  request->address = tool_io_address_;

  try
  {
    uint64_t request_sequence = 0U;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      request_in_flight_ = true;
      pending_request_id_.reset();
      request_sequence = ++next_request_sequence_;
      pending_request_sequence_ = request_sequence;
      pending_request_started_at_ = now;
    }

    const auto pending_request = read_client_->async_send_request(
      request,
      [this, request_sequence](ReadSingleIOClient::SharedFuture future) {
        handle_read_response(request_sequence, future);
      });

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (request_in_flight_ && pending_request_sequence_.has_value() &&
      *pending_request_sequence_ == request_sequence)
    {
      pending_request_id_ = pending_request.request_id;
    }
  }
  catch (const std::exception & ex)
  {
    set_unknown_state(std::string("tool IO read request failed: ") + ex.what());
  }
#endif
}

#if HW_ADAPTER_HAS_TOOL_IO_INTERFACES
void ToolStateMonitor::handle_read_response(
  uint64_t request_sequence,
  ReadSingleIOClient::SharedFuture future)
{
  ReadSingleIO::Response::SharedPtr response;
  try
  {
    response = future.get();
  }
  catch (const std::exception & ex)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!pending_request_sequence_.has_value() || *pending_request_sequence_ != request_sequence)
    {
      return;
    }

    request_in_flight_ = false;
    pending_request_id_.reset();
    pending_request_sequence_.reset();
    current_tool_state_.reset();
    snapshot_.has_state = false;
    snapshot_.output_state = false;
    snapshot_.status_message = std::string("tool IO read callback failed: ") + ex.what();
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!pending_request_sequence_.has_value() || *pending_request_sequence_ != request_sequence)
  {
    return;
  }

  request_in_flight_ = false;
  pending_request_id_.reset();
  pending_request_sequence_.reset();

  if (!response)
  {
    current_tool_state_.reset();
    snapshot_.has_state = false;
    snapshot_.output_state = false;
    snapshot_.status_message = "tool IO read returned a null response";
    return;
  }

  if (!response->success || response->result_code != motoros2_interfaces::msg::IoResultCodes::OK)
  {
    current_tool_state_.reset();
    snapshot_.has_state = false;
    snapshot_.output_state = false;

    std::ostringstream oss;
    oss << "tool IO read failed with code " << response->result_code;
    if (!response->message.empty())
    {
      oss << ": " << response->message;
    }
    snapshot_.status_message = oss.str();
    return;
  }

  ToolState state;
  state.address = tool_io_address_;
  state.raw_value = response->value;
  state.active = response->value != 0;
  state.detail =
    response->message.empty() ? "tool IO state available" : response->message;

  current_tool_state_ = state;
  snapshot_.has_state = true;
  snapshot_.output_state = state.active;
  snapshot_.address = state.address;
  snapshot_.status_message = state.detail;
}
#endif

void ToolStateMonitor::set_unknown_state(const std::string & status_message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_tool_state_.reset();
  request_in_flight_ = false;
  pending_request_id_.reset();
  pending_request_sequence_.reset();
  snapshot_.has_state = false;
  snapshot_.output_state = false;
  snapshot_.status_message = status_message;
}
}  // namespace hw_adapter
