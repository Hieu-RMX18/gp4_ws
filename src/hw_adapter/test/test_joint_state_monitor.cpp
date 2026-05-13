// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hw_adapter/joint_state_monitor.hpp"

namespace {
using namespace std::chrono_literals;

std::vector<std::string> canonical_joint_names() {
  return {"joint_1_s", "joint_2_l", "joint_3_u",
          "joint_4_r", "joint_5_b", "joint_6_t"};
}

sensor_msgs::msg::JointState make_joint_state() {
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"joint_3_u", "joint_1_s", "joint_6_t",
                      "joint_4_r", "joint_2_l", "joint_5_b"};
  joint_state.position = {-0.2, 0.0, 0.5, 0.3, 0.1, -0.4};
  joint_state.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  return joint_state;
}

bool spin_until(rclcpp::executors::SingleThreadedExecutor &executor,
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

class JointStateMonitorTest : public ::testing::Test {
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

TEST_F(JointStateMonitorTest, invalid_before_first_message) {
  auto monitor_node =
      std::make_shared<rclcpp::Node>("joint_state_monitor_unknown_test");
  hw_adapter::JointStateMonitor monitor(*monitor_node, canonical_joint_names(),
                                        "/test_hw_adapter/joint_state_unknown");

  const auto snapshot = monitor.latest_snapshot();
  EXPECT_FALSE(snapshot.has_message);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(snapshot.status_message.find("unknown"), std::string::npos);
}

TEST_F(JointStateMonitorTest, subscribes_with_sensor_data_qos_for_motoros2) {
  const std::string topic = "/test_hw_adapter/joint_state_qos";
  auto monitor_node =
      std::make_shared<rclcpp::Node>("joint_state_monitor_qos_test");
  hw_adapter::JointStateMonitor monitor(*monitor_node, canonical_joint_names(),
                                        topic, 200ms);

  const auto topic_info = monitor_node->get_subscriptions_info_by_topic(topic);
  ASSERT_EQ(topic_info.size(), 1U);
  EXPECT_EQ(topic_info.front().qos_profile().reliability(),
            rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(topic_info.front().qos_profile().durability(),
            rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(JointStateMonitorTest, reorders_joint_positions_to_canonical_layout) {
  const std::string topic = "/test_hw_adapter/joint_state_reordered";
  auto monitor_node =
      std::make_shared<rclcpp::Node>("joint_state_monitor_reordered_test");
  auto publisher_node =
      std::make_shared<rclcpp::Node>("joint_state_monitor_reordered_pub");
  hw_adapter::JointStateMonitor monitor(*monitor_node, canonical_joint_names(),
                                        topic, 200ms);

  auto publisher =
      publisher_node->create_publisher<sensor_msgs::msg::JointState>(
          topic, rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(monitor_node);
  executor.add_node(publisher_node);

  ASSERT_TRUE(spin_until(executor, [&publisher]() {
    return publisher->get_subscription_count() > 0;
  }));
  publisher->publish(make_joint_state());

  ASSERT_TRUE(spin_until(
      executor, [&monitor]() { return monitor.latest_snapshot().valid; }));
  const auto snapshot = monitor.latest_snapshot();
  ASSERT_EQ(snapshot.ordered_positions.size(), 6U);
  EXPECT_DOUBLE_EQ(snapshot.ordered_positions.at(0), 0.0);
  EXPECT_DOUBLE_EQ(snapshot.ordered_positions.at(1), 0.1);
  EXPECT_DOUBLE_EQ(snapshot.ordered_positions.at(2), -0.2);
  EXPECT_DOUBLE_EQ(snapshot.ordered_positions.at(3), 0.3);
  EXPECT_DOUBLE_EQ(snapshot.ordered_positions.at(4), -0.4);
  EXPECT_DOUBLE_EQ(snapshot.ordered_positions.at(5), 0.5);
}

TEST_F(JointStateMonitorTest, stale_joint_state_is_invalid) {
  const std::string topic = "/test_hw_adapter/joint_state_stale";
  auto monitor_node =
      std::make_shared<rclcpp::Node>("joint_state_monitor_stale_test");
  auto publisher_node =
      std::make_shared<rclcpp::Node>("joint_state_monitor_stale_pub");
  hw_adapter::JointStateMonitor monitor(*monitor_node, canonical_joint_names(),
                                        topic, 30ms);

  auto publisher =
      publisher_node->create_publisher<sensor_msgs::msg::JointState>(
          topic, rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(monitor_node);
  executor.add_node(publisher_node);

  ASSERT_TRUE(spin_until(executor, [&publisher]() {
    return publisher->get_subscription_count() > 0;
  }));
  publisher->publish(make_joint_state());
  ASSERT_TRUE(spin_until(
      executor, [&monitor]() { return monitor.latest_snapshot().valid; }));

  std::this_thread::sleep_for(80ms);
  executor.spin_some();

  const auto snapshot = monitor.latest_snapshot();
  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(snapshot.status_message.find("stale joint state"),
            std::string::npos);
}
