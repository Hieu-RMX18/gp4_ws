// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <motoros2_interfaces/msg/io_result_codes.hpp>
#include <motoros2_interfaces/srv/read_single_io.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hw_adapter/tool_state_monitor.hpp"

namespace {
using namespace std::chrono_literals;

class ExecutorThread {
public:
  explicit ExecutorThread(rclcpp::executors::SingleThreadedExecutor &executor)
      : executor_(executor), thread_([this]() { executor_.spin(); }) {}

  ~ExecutorThread() {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor &executor_;
  std::thread thread_;
};

bool wait_until(const std::function<bool()> &predicate,
                const std::chrono::milliseconds timeout,
                const std::chrono::milliseconds poll_period = 10ms) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(poll_period);
  }

  return predicate();
}

class ToolStateMonitorTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};
} // namespace

TEST_F(ToolStateMonitorTest, no_state_yet_case) {
  auto client_node =
      std::make_shared<rclcpp::Node>("tool_state_monitor_no_state_client");
  hw_adapter::ToolStateMonitor monitor(
      *client_node,
      hw_adapter::ToolServiceNames{"/test_hw_adapter/tool_read_no_state", ""},
      42U, 20ms);

  EXPECT_FALSE(monitor.has_tool_state());
  EXPECT_FALSE(monitor.current_tool_state().has_value());

  const auto snapshot = monitor.snapshot();
  EXPECT_TRUE(snapshot.motoros2_interfaces_available);
  EXPECT_TRUE(snapshot.read_service_configured);
  EXPECT_FALSE(snapshot.has_state);
  EXPECT_FALSE(snapshot.output_state);
  EXPECT_EQ(snapshot.address, 42U);
  EXPECT_NE(snapshot.status_message.find("waiting for first sampled state"),
            std::string::npos);
}

TEST_F(ToolStateMonitorTest, state_update_case) {
  const std::string read_service = "/test_hw_adapter/tool_read_state_update";
  auto server_node =
      std::make_shared<rclcpp::Node>("tool_state_monitor_state_server");
  std::atomic<int> read_requests{0};

  auto read_server = server_node->create_service<
      motoros2_interfaces::srv::ReadSingleIO>(
      read_service,
      [&read_requests](
          const std::shared_ptr<motoros2_interfaces::srv::ReadSingleIO::Request>
              request,
          std::shared_ptr<motoros2_interfaces::srv::ReadSingleIO::Response>
              response) {
        ++read_requests;
        response->result_code = motoros2_interfaces::msg::IoResultCodes::OK;
        response->message = "tool output high";
        response->success = true;
        response->value = request->address == 101U ? 1 : 0;
      });
  (void)read_server;

  auto client_node =
      std::make_shared<rclcpp::Node>("tool_state_monitor_state_client");
  hw_adapter::ToolStateMonitor monitor(
      *client_node, hw_adapter::ToolServiceNames{read_service, ""}, 101U, 20ms);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  ASSERT_TRUE(
      wait_until([&monitor]() { return monitor.has_tool_state(); }, 500ms));

  const auto state = monitor.current_tool_state();
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->address, 101U);
  EXPECT_EQ(state->raw_value, 1);
  EXPECT_TRUE(state->active);
  EXPECT_EQ(state->detail, "tool output high");
  EXPECT_GT(read_requests.load(), 0);

  const auto snapshot = monitor.snapshot();
  EXPECT_TRUE(snapshot.has_state);
  EXPECT_TRUE(snapshot.output_state);
  EXPECT_EQ(snapshot.address, 101U);
  EXPECT_EQ(snapshot.status_message, "tool output high");
}

TEST_F(ToolStateMonitorTest, unavailable_backend_case) {
  const std::string read_service = "/test_hw_adapter/tool_read_unavailable";
  auto client_node =
      std::make_shared<rclcpp::Node>("tool_state_monitor_unavailable_client");
  hw_adapter::ToolStateMonitor monitor(
      *client_node, hw_adapter::ToolServiceNames{read_service, ""}, 17U, 20ms);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  ASSERT_TRUE(wait_until(
      [&monitor, &read_service]() {
        const auto snapshot = monitor.snapshot();
        return snapshot.status_message.find("unavailable") !=
                   std::string::npos &&
               snapshot.status_message.find(read_service) != std::string::npos;
      },
      500ms));

  EXPECT_FALSE(monitor.has_tool_state());
  EXPECT_FALSE(monitor.current_tool_state().has_value());

  const auto snapshot = monitor.snapshot();
  EXPECT_FALSE(snapshot.has_state);
  EXPECT_FALSE(snapshot.output_state);
  EXPECT_EQ(snapshot.address, 17U);
}
