// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "primitives/primitive_lin.hpp"

namespace primitives {
namespace {
class FakeLinearBackend final : public LinearExecutionBackend {
public:
  bool server_available = true;
  bool normalize_ok = true;
  bool current_pose_ok = true;
  bool compute_ok = true;

  double cartesian_fraction = 1.0;
  std::string compute_reason = "computeCartesianPath failed";
  LinearScalingConfig scales;
  PrimitiveResult postprocess_result;
  geometry_msgs::msg::Pose current_pose;

  bool compute_called = false;
  bool postprocess_called = false;
  std::vector<geometry_msgs::msg::Pose> last_waypoints;
  double last_velocity_scale = 0.0;
  double last_acceleration_scale = 0.0;

  bool wait_for_servers(std::string &reason) override {
    if (server_available) {
      reason.clear();
      return true;
    }

    reason = "server unavailable";
    return false;
  }

  bool normalize_pose(geometry_msgs::msg::Pose &pose,
                      std::string &reason) override {
    (void)pose;

    if (normalize_ok) {
      reason.clear();
      return true;
    }

    reason = "quaternion norm is zero";
    return false;
  }

  bool get_current_pose_world(geometry_msgs::msg::Pose &pose,
                              std::string &reason) override {
    if (!current_pose_ok) {
      reason = "current pose unavailable";
      return false;
    }

    pose = current_pose;
    reason.clear();
    return true;
  }

  bool
  compute_cartesian_path(const std::vector<geometry_msgs::msg::Pose> &waypoints,
                         double &fraction, std::string &reason) override {
    compute_called = true;
    last_waypoints = waypoints;

    if (!compute_ok) {
      reason = compute_reason;
      return false;
    }

    fraction = cartesian_fraction;
    reason.clear();
    return true;
  }

  LinearScalingConfig scaling_config() const override { return scales; }

  PrimitiveResult
  postprocess_and_validate(double velocity_scale, double acceleration_scale,
                           double cartesian_fraction_in) override {
    (void)cartesian_fraction_in;
    postprocess_called = true;
    last_velocity_scale = velocity_scale;
    last_acceleration_scale = acceleration_scale;
    return postprocess_result;
  }
};

LINGoal make_lin_goal() {
  LINGoal goal;
  goal.target_pose.position.x = 0.25;
  goal.target_pose.position.y = 0.10;
  goal.target_pose.position.z = 0.40;
  goal.target_pose.orientation.w = 1.0;
  goal.velocity_scale = 0.9;
  goal.acceleration_scale = 0.8;
  return goal;
}

TEST(PrimitiveLinTest, LinSuccessWithFractionAtLeastNinetyFivePercent) {
  PrimitiveLin primitive;
  FakeLinearBackend backend;

  backend.current_pose.orientation.w = 1.0;
  backend.current_pose.position.z = 0.30;
  backend.cartesian_fraction = 0.97;
  backend.scales.velocity_cap = 0.3;
  backend.scales.acceleration_cap = 0.2;
  backend.scales.default_velocity = 0.3;
  backend.scales.default_acceleration = 0.2;

  backend.postprocess_result.success = true;
  backend.postprocess_result.reason = PrimitiveFailReason::UNKNOWN;
  backend.postprocess_result.message = "LIN plan ready";

  const PrimitiveResult result = primitive.execute(make_lin_goal(), backend);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::UNKNOWN);
  EXPECT_TRUE(backend.compute_called);
  EXPECT_TRUE(backend.postprocess_called);
  EXPECT_EQ(backend.last_waypoints.size(), 2U);
  EXPECT_DOUBLE_EQ(backend.last_velocity_scale, 0.3);
  EXPECT_DOUBLE_EQ(backend.last_acceleration_scale, 0.2);
}

TEST(PrimitiveLinTest, LinFailureWhenFractionBelowNinetyFivePercent) {
  PrimitiveLin primitive;
  FakeLinearBackend backend;

  backend.current_pose.orientation.w = 1.0;
  backend.cartesian_fraction = 0.80;

  const PrimitiveResult result = primitive.execute(make_lin_goal(), backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::CARTESIAN_FRACTION_LOW);
  EXPECT_TRUE(backend.compute_called);
  EXPECT_FALSE(backend.postprocess_called);
}

TEST(PrimitiveLinTest, LinFailureWhenQuaternionIsInvalid) {
  PrimitiveLin primitive;
  FakeLinearBackend backend;

  backend.normalize_ok = false;

  LINGoal goal = make_lin_goal();
  goal.target_pose.orientation.x = 0.0;
  goal.target_pose.orientation.y = 0.0;
  goal.target_pose.orientation.z = 0.0;
  goal.target_pose.orientation.w = 0.0;

  const PrimitiveResult result = primitive.execute(goal, backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::INVALID_ORIENTATION);
  EXPECT_FALSE(backend.compute_called);
  EXPECT_FALSE(backend.postprocess_called);
}

TEST(PrimitiveLinTest, QualityGateRejectionPropagatesFailureReason) {
  PrimitiveLin primitive;
  FakeLinearBackend backend;

  backend.current_pose.orientation.w = 1.0;
  backend.cartesian_fraction = 1.0;
  backend.postprocess_result.success = false;
  backend.postprocess_result.reason = PrimitiveFailReason::WRIST_FLIP_DETECTED;
  backend.postprocess_result.message =
      "LIN quality gate rejected: wrist flip guard reject";

  const PrimitiveResult result = primitive.execute(make_lin_goal(), backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::WRIST_FLIP_DETECTED);
  EXPECT_TRUE(backend.postprocess_called);
}
} // namespace
} // namespace primitives
