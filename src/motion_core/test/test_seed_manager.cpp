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

#include "motion_core/seed_manager.hpp"

namespace
{
using namespace std::chrono_literals;

sensor_msgs::msg::JointState make_joint_state()
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"joint_3_u", "joint_1_s", "joint_6_t", "joint_4_r", "joint_2_l", "joint_5_b"};
  joint_state.position = {-0.2, 0.0, 0.5, 0.3, 0.1, -0.4};
  joint_state.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  return joint_state;
}

bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  const std::chrono::milliseconds timeout = 1500ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    if (predicate())
    {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }

  executor.spin_some();
  return predicate();
}

class SeedManagerTest : public ::testing::Test
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

TEST_F(SeedManagerTest, current_joint_positions_require_fresh_joint_state)
{
  auto node = std::make_shared<rclcpp::Node>("seed_manager_no_state_test");
  motion_core::SeedManager seed_manager(*node);

  std::vector<double> positions;
  EXPECT_FALSE(seed_manager.get_current_joint_positions(positions));
  EXPECT_TRUE(positions.empty());
}

TEST_F(SeedManagerTest, current_joint_positions_are_reordered_to_gp4_joint_layout)
{
  auto manager_node = std::make_shared<rclcpp::Node>("seed_manager_reorder_test");
  auto publisher_node = std::make_shared<rclcpp::Node>("seed_manager_reorder_pub");
  motion_core::SeedManager seed_manager(*manager_node);

  auto publisher = publisher_node->create_publisher<sensor_msgs::msg::JointState>(
    "/yaskawa/joint_states",
    rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manager_node);
  executor.add_node(publisher_node);

  ASSERT_TRUE(spin_until(executor, [&publisher]() {return publisher->get_subscription_count() > 0;}));
  publisher->publish(make_joint_state());

  std::vector<double> positions;
  ASSERT_TRUE(spin_until(
    executor,
    [&seed_manager, &positions]() {return seed_manager.get_current_joint_positions(positions);}));

  ASSERT_EQ(positions.size(), 6U);
  EXPECT_DOUBLE_EQ(positions.at(0), 0.0);
  EXPECT_DOUBLE_EQ(positions.at(1), 0.1);
  EXPECT_DOUBLE_EQ(positions.at(2), -0.2);
  EXPECT_DOUBLE_EQ(positions.at(3), 0.3);
  EXPECT_DOUBLE_EQ(positions.at(4), -0.4);
  EXPECT_DOUBLE_EQ(positions.at(5), 0.5);
}

TEST_F(SeedManagerTest, cached_seed_is_used_when_fallback_is_enabled)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("allow_fallback_seed", true)});
  auto node = std::make_shared<rclcpp::Node>("seed_manager_cached_seed_test", options);
  motion_core::SeedManager seed_manager(*node);

  const std::vector<double> expected_seed = {0.0, 0.2, -0.1, 0.3, -0.5, 0.1};
  seed_manager.cache_successful_seed("LIN", expected_seed);

  std::vector<double> seed;
  EXPECT_TRUE(seed_manager.get_seed_state("LIN", seed));
  EXPECT_EQ(seed, expected_seed);
}
