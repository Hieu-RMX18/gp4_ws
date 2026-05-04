// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <string>

#include <gtest/gtest.h>

#include "primitives/primitive_home.hpp"

namespace primitives {
namespace {
class FakeHomeBackend final : public HomeExecutionBackend {
public:
  bool server_available = true;
  bool named_target_available = true;
  PrimitiveResult plan_result;
  HomeScalingConfig scales;

  bool plan_called = false;

  bool wait_for_servers(std::string &reason) override {
    if (server_available) {
      reason.clear();
      return true;
    }

    reason = "server unavailable";
    return false;
  }

  bool set_named_target_home(std::string &reason) override {
    if (named_target_available) {
      reason.clear();
      return true;
    }

    reason = "named target 'home' missing";
    return false;
  }

  HomeScalingConfig scaling_config() const override { return scales; }

  PrimitiveResult plan_with_pipeline(double velocity_scale,
                                     double acceleration_scale) override {
    (void)velocity_scale;
    (void)acceleration_scale;
    plan_called = true;
    return plan_result;
  }
};

TEST(PrimitiveHomeTest, HomeSuccessWithNamedTargetAvailable) {
  PrimitiveHome primitive;
  FakeHomeBackend backend;

  backend.plan_result.success = true;
  backend.plan_result.reason = PrimitiveFailReason::UNKNOWN;
  backend.plan_result.message = "HOME plan ready";
  backend.plan_result.planning_time_sec = 0.12;

  const PrimitiveResult result = primitive.execute(backend);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::UNKNOWN);
  EXPECT_TRUE(backend.plan_called);
}

TEST(PrimitiveHomeTest, HomeFailureWhenNamedTargetMissing) {
  PrimitiveHome primitive;
  FakeHomeBackend backend;

  backend.named_target_available = false;

  const PrimitiveResult result = primitive.execute(backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::NAMED_TARGET_NOT_FOUND);
  EXPECT_FALSE(backend.plan_called);
}
} // namespace
} // namespace primitives
