// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <motoros2_interfaces/msg/motion_ready_enum.hpp>
#include <motoros2_interfaces/srv/reset_error.hpp>
#include <motoros2_interfaces/srv/start_traj_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hw_adapter/motoros2_session_manager.hpp"

namespace
{
using namespace std::chrono_literals;

class ExecutorThread
{
public:
  explicit ExecutorThread(rclcpp::executors::SingleThreadedExecutor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorThread()
  {
    executor_.cancel();
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor & executor_;
  std::thread thread_;
};

class Motoros2SessionManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }
};
}  // namespace

TEST_F(Motoros2SessionManagerTest, start_traj_mode_success_path)
{
  const std::string start_service = "/test_hw_adapter/start_traj_mode_success";
  auto server_node = std::make_shared<rclcpp::Node>("motoros2_session_manager_start_server");
  std::atomic<int> start_requests{0};

  auto server = server_node->create_service<motoros2_interfaces::srv::StartTrajMode>(
    start_service,
    [&start_requests](
      const std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Request>,
      std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Response> response)
    {
      ++start_requests;
      response->result_code.value = motoros2_interfaces::msg::MotionReadyEnum::READY;
      response->message = "Ready";
    });
  (void)server;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  ExecutorThread spin_thread(executor);

  auto client_node = std::make_shared<rclcpp::Node>("motoros2_session_manager_start_client");
  hw_adapter::Motoros2SessionManager manager(
    *client_node,
    hw_adapter::SessionServiceNames{start_service, "/unused_reset_error", "", "/unused_action"},
    300ms);

  std::string reason;
  EXPECT_TRUE(manager.start_traj_mode(reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_TRUE(manager.is_session_ready());
  EXPECT_EQ(start_requests.load(), 1);
}

TEST_F(Motoros2SessionManagerTest, start_traj_mode_timeout_when_service_unavailable)
{
  auto client_node = std::make_shared<rclcpp::Node>("motoros2_session_manager_timeout_client");
  hw_adapter::Motoros2SessionManager manager(
    *client_node,
    hw_adapter::SessionServiceNames{
      "/test_hw_adapter/missing_start_traj_mode", "/unused_reset_error", "", "/unused_action"},
    50ms);

  std::string reason;
  EXPECT_FALSE(manager.start_traj_mode(reason));
  EXPECT_FALSE(manager.is_session_ready());
  EXPECT_NE(reason.find("Timed out"), std::string::npos);
  EXPECT_NE(reason.find("/test_hw_adapter/missing_start_traj_mode"), std::string::npos);
}

TEST_F(Motoros2SessionManagerTest, reset_error_call_path)
{
  const std::string start_service = "/test_hw_adapter/start_traj_mode_for_reset";
  const std::string reset_service = "/test_hw_adapter/reset_error_success";
  auto server_node = std::make_shared<rclcpp::Node>("motoros2_session_manager_reset_server");
  std::atomic<int> start_requests{0};
  std::atomic<int> reset_requests{0};

  auto start_server = server_node->create_service<motoros2_interfaces::srv::StartTrajMode>(
    start_service,
    [&start_requests](
      const std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Request>,
      std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Response> response)
    {
      ++start_requests;
      response->result_code.value = motoros2_interfaces::msg::MotionReadyEnum::READY;
      response->message = "Ready";
    });
  auto reset_server = server_node->create_service<motoros2_interfaces::srv::ResetError>(
    reset_service,
    [&reset_requests](
      const std::shared_ptr<motoros2_interfaces::srv::ResetError::Request>,
      std::shared_ptr<motoros2_interfaces::srv::ResetError::Response> response)
    {
      ++reset_requests;
      response->result_code.value = motoros2_interfaces::msg::MotionReadyEnum::READY;
      response->message = "Reset complete";
    });
  (void)start_server;
  (void)reset_server;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  ExecutorThread spin_thread(executor);

  auto client_node = std::make_shared<rclcpp::Node>("motoros2_session_manager_reset_client");
  hw_adapter::Motoros2SessionManager manager(
    *client_node,
    hw_adapter::SessionServiceNames{start_service, reset_service, "", "/unused_action"},
    300ms);

  std::string reason;
  ASSERT_TRUE(manager.start_traj_mode(reason));
  ASSERT_TRUE(manager.is_session_ready());

  EXPECT_TRUE(manager.reset_error(reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_FALSE(manager.is_session_ready());
  EXPECT_EQ(start_requests.load(), 1);
  EXPECT_EQ(reset_requests.load(), 1);
}

TEST_F(Motoros2SessionManagerTest, repeated_safe_calls_do_not_corrupt_state)
{
  const std::string start_service = "/test_hw_adapter/start_traj_mode_repeated";
  const std::string reset_service = "/test_hw_adapter/reset_error_repeated";
  auto server_node = std::make_shared<rclcpp::Node>("motoros2_session_manager_repeat_server");
  std::atomic<int> start_requests{0};
  std::atomic<int> reset_requests{0};

  auto start_server = server_node->create_service<motoros2_interfaces::srv::StartTrajMode>(
    start_service,
    [&start_requests](
      const std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Request>,
      std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Response> response)
    {
      ++start_requests;
      response->result_code.value = motoros2_interfaces::msg::MotionReadyEnum::READY;
      response->message = "Ready";
    });
  auto reset_server = server_node->create_service<motoros2_interfaces::srv::ResetError>(
    reset_service,
    [&reset_requests](
      const std::shared_ptr<motoros2_interfaces::srv::ResetError::Request>,
      std::shared_ptr<motoros2_interfaces::srv::ResetError::Response> response)
    {
      ++reset_requests;
      response->result_code.value = motoros2_interfaces::msg::MotionReadyEnum::READY;
      response->message = "Reset complete";
    });
  (void)start_server;
  (void)reset_server;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  ExecutorThread spin_thread(executor);

  auto client_node = std::make_shared<rclcpp::Node>("motoros2_session_manager_repeat_client");
  hw_adapter::Motoros2SessionManager manager(
    *client_node,
    hw_adapter::SessionServiceNames{start_service, reset_service, "", "/unused_action"},
    300ms);

  std::string reason;
  ASSERT_TRUE(manager.start_traj_mode(reason));
  EXPECT_TRUE(manager.is_session_ready());

  EXPECT_TRUE(manager.start_traj_mode(reason));
  EXPECT_TRUE(manager.is_session_ready());
  EXPECT_EQ(start_requests.load(), 1);

  EXPECT_TRUE(manager.reset_error(reason));
  EXPECT_FALSE(manager.is_session_ready());
  EXPECT_EQ(reset_requests.load(), 1);

  EXPECT_TRUE(manager.reset_error(reason));
  EXPECT_FALSE(manager.is_session_ready());
  EXPECT_EQ(reset_requests.load(), 2);
}
