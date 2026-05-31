// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <industrial_msgs/msg/robot_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hw_adapter/robot_status_monitor.hpp"

namespace {
using namespace std::chrono_literals;

class RobotStatusMonitorTest : public ::testing::Test {
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

  static industrial_msgs::msg::RobotStatus make_status(const int8_t e_stop,
                                                       const int8_t in_error) {
    industrial_msgs::msg::RobotStatus status;
    status.mode.val = industrial_msgs::msg::RobotMode::AUTO;
    status.e_stopped.val = e_stop;
    status.drives_powered.val = industrial_msgs::msg::TriState::TRUE;
    status.motion_possible.val = industrial_msgs::msg::TriState::TRUE;
    status.in_motion.val = industrial_msgs::msg::TriState::FALSE;
    status.in_error.val = in_error;
    return status;
  }

  static bool spin_until(rclcpp::executors::SingleThreadedExecutor &executor,
                         const std::function<bool()> &predicate,
                         const std::chrono::milliseconds timeout = 1500ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }

    executor.spin_some();
    return predicate();
  }
};
} // namespace

TEST_F(RobotStatusMonitorTest, readiness_false_before_first_status_message) {
  auto monitor_node =
      std::make_shared<rclcpp::Node>("robot_status_monitor_unknown_test");
  hw_adapter::RobotStatusMonitor monitor(*monitor_node,
                                         "/test_hw_adapter/status_unknown");

  EXPECT_FALSE(monitor.has_status());
  EXPECT_FALSE(monitor.is_ready());
  EXPECT_FALSE(monitor.is_estop_active());
  EXPECT_NE(monitor.status_summary().find("unknown"), std::string::npos);
}

TEST_F(RobotStatusMonitorTest, estop_transitions_update_readiness) {
  const std::string topic = "/test_hw_adapter/status_estop";
  auto monitor_node =
      std::make_shared<rclcpp::Node>("robot_status_monitor_estop_test");
  auto publisher_node =
      std::make_shared<rclcpp::Node>("robot_status_monitor_estop_pub");
  hw_adapter::RobotStatusMonitor monitor(*monitor_node, topic);

  auto publisher =
      publisher_node->create_publisher<industrial_msgs::msg::RobotStatus>(
          topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(monitor_node);
  executor.add_node(publisher_node);

  ASSERT_TRUE(spin_until(executor, [&publisher]() {
    return publisher->get_subscription_count() > 0;
  }));

  publisher->publish(make_status(industrial_msgs::msg::TriState::FALSE,
                                 industrial_msgs::msg::TriState::FALSE));
  ASSERT_TRUE(spin_until(executor, [&monitor]() {
    return monitor.has_status() && monitor.is_ready();
  }));
  EXPECT_FALSE(monitor.is_estop_active());
  EXPECT_NE(monitor.status_summary().find("ready"), std::string::npos);

  auto estop_status = make_status(industrial_msgs::msg::TriState::TRUE,
                                  industrial_msgs::msg::TriState::FALSE);
  estop_status.motion_possible.val = industrial_msgs::msg::TriState::FALSE;
  publisher->publish(estop_status);

  ASSERT_TRUE(spin_until(executor, [&monitor]() {
    return monitor.has_status() && monitor.is_estop_active() &&
           !monitor.is_ready();
  }));
  EXPECT_NE(monitor.status_summary().find("E-STOP active"), std::string::npos);
}

TEST_F(RobotStatusMonitorTest, stale_status_is_not_ready_for_motion) {
  const std::string topic = "/test_hw_adapter/status_stale";
  auto monitor_node =
      std::make_shared<rclcpp::Node>("robot_status_monitor_stale_test");
  auto publisher_node =
      std::make_shared<rclcpp::Node>("robot_status_monitor_stale_pub");
  hw_adapter::RobotStatusMonitor monitor(*monitor_node, topic, 30ms);

  auto publisher =
      publisher_node->create_publisher<industrial_msgs::msg::RobotStatus>(
          topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(monitor_node);
  executor.add_node(publisher_node);

  ASSERT_TRUE(spin_until(executor, [&publisher]() {
    return publisher->get_subscription_count() > 0;
  }));
  publisher->publish(make_status(industrial_msgs::msg::TriState::FALSE,
                                 industrial_msgs::msg::TriState::FALSE));
  ASSERT_TRUE(spin_until(executor, [&monitor]() {
    return monitor.has_status() && monitor.is_ready();
  }));

  std::this_thread::sleep_for(80ms);
  executor.spin_some();

  std::string reason;
  EXPECT_FALSE(monitor.is_ready_for_motion(reason));
  EXPECT_NE(reason.find("stale robot status"), std::string::npos);
}
