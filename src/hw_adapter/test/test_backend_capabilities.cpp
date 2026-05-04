// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "hw_adapter/backend_capabilities.hpp"

namespace {
class BackendCapabilitiesTest : public ::testing::Test {
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

TEST_F(BackendCapabilitiesTest, exposes_yrc1000micro_defaults) {
  auto node =
      std::make_shared<rclcpp::Node>("backend_capabilities_defaults_test");
  const hw_adapter::BackendCapabilities capabilities(*node);

  EXPECT_EQ(capabilities.controller_variant(), "YRC1000micro");
  EXPECT_TRUE(capabilities.expects_zero_effort_feedback());
  EXPECT_TRUE(capabilities.supports_open_loop_control());
  EXPECT_FALSE(capabilities.supports_async_motion());
}

TEST_F(BackendCapabilitiesTest, validates_gp4_joint_order) {
  hw_adapter::BackendCapabilities capabilities;
  std::string reason;

  EXPECT_TRUE(
      capabilities.validate_joint_names({"joint_1_s", "joint_2_l", "joint_3_u",
                                         "joint_4_r", "joint_5_b", "joint_6_t"},
                                        reason));
  EXPECT_TRUE(reason.empty());
}
