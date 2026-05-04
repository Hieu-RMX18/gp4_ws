// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "primitives/primitive_ptp.hpp"

namespace primitives {
namespace {
class FakePtpBackend final : public PtpExecutionBackend {
public:
  bool server_available = true;
  bool planner_configured = true;
  bool set_joint_target_ok = true;
  bool normalize_pose_ok = true;
  bool solve_ik_ok = true;

  std::vector<double> ik_solution{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  PtpScalingConfig scales;
  PrimitiveResult plan_result;

  bool set_joint_target_called = false;
  bool normalize_pose_called = false;
  bool solve_ik_called = false;
  bool plan_called = false;

  std::vector<double> last_joint_target;
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

  bool configure_ptp_planner(std::string &reason) override {
    if (planner_configured) {
      reason.clear();
      return true;
    }

    reason = "planner not configured";
    return false;
  }

  bool set_joint_target(const std::vector<double> &target,
                        std::string &reason) override {
    set_joint_target_called = true;
    last_joint_target = target;

    if (set_joint_target_ok) {
      reason.clear();
      return true;
    }

    reason = "joint target rejected";
    return false;
  }

  bool normalize_pose(geometry_msgs::msg::Pose &pose,
                      std::string &reason) override {
    (void)pose;
    normalize_pose_called = true;

    if (normalize_pose_ok) {
      reason.clear();
      return true;
    }

    reason = "quaternion norm is zero";
    return false;
  }

  bool solve_pose_to_joints(const geometry_msgs::msg::Pose &pose,
                            std::vector<double> &joint_solution,
                            std::string &reason) override {
    (void)pose;
    solve_ik_called = true;

    if (solve_ik_ok) {
      joint_solution = ik_solution;
      reason.clear();
      return true;
    }

    reason = "IK failed";
    return false;
  }

  PtpScalingConfig scaling_config() const override { return scales; }

  PrimitiveResult plan_with_pipeline(double velocity_scale,
                                     double acceleration_scale) override {
    plan_called = true;
    last_velocity_scale = velocity_scale;
    last_acceleration_scale = acceleration_scale;
    return plan_result;
  }
};

PTPGoal make_joint_goal(const std::vector<double> &joints) {
  PTPGoal goal;
  goal.joint_target = joints;
  goal.velocity_scale = 0.9;
  goal.acceleration_scale = 0.8;
  return goal;
}

TEST(PrimitivePtpTest, PtpSuccessWithValidSixJointInput) {
  PrimitivePtp primitive;
  FakePtpBackend backend;

  backend.scales.velocity_cap = 0.3;
  backend.scales.acceleration_cap = 0.2;
  backend.scales.default_velocity = 0.3;
  backend.scales.default_acceleration = 0.2;

  backend.plan_result.success = true;
  backend.plan_result.reason = PrimitiveFailReason::UNKNOWN;
  backend.plan_result.message = "PTP plan ready";

  const PrimitiveResult result = primitive.execute(
      make_joint_goal({0.0, 0.1, 0.2, 0.3, 0.4, 0.5}), backend);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::UNKNOWN);
  EXPECT_TRUE(backend.set_joint_target_called);
  EXPECT_TRUE(backend.plan_called);
  EXPECT_DOUBLE_EQ(backend.last_velocity_scale, 0.3);
  EXPECT_DOUBLE_EQ(backend.last_acceleration_scale, 0.2);
}

TEST(PrimitivePtpTest, PtpFailureWhenJointListIsFiveOrSeven) {
  PrimitivePtp primitive;

  {
    FakePtpBackend backend;
    const PrimitiveResult result =
        primitive.execute(make_joint_goal({0.0, 0.1, 0.2, 0.3, 0.4}), backend);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, PrimitiveFailReason::JOINT_COUNT_MISMATCH);
    EXPECT_FALSE(backend.set_joint_target_called);
    EXPECT_FALSE(backend.plan_called);
  }

  {
    FakePtpBackend backend;
    const PrimitiveResult result = primitive.execute(
        make_joint_goal({0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6}), backend);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, PrimitiveFailReason::JOINT_COUNT_MISMATCH);
    EXPECT_FALSE(backend.set_joint_target_called);
    EXPECT_FALSE(backend.plan_called);
  }
}

TEST(PrimitivePtpTest, PtpFailureWhenPoseTargetQuaternionHasZeroNorm) {
  PrimitivePtp primitive;
  FakePtpBackend backend;

  backend.normalize_pose_ok = false;

  PTPGoal goal;
  goal.pose_target.position.x = 0.25;
  goal.pose_target.orientation.x = 0.0;
  goal.pose_target.orientation.y = 0.0;
  goal.pose_target.orientation.z = 0.0;
  goal.pose_target.orientation.w = 0.0;

  const PrimitiveResult result = primitive.execute(goal, backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::INVALID_ORIENTATION);
  EXPECT_TRUE(backend.normalize_pose_called);
  EXPECT_FALSE(backend.solve_ik_called);
  EXPECT_FALSE(backend.plan_called);
}

TEST(PrimitivePtpTest, PtpFailureWhenQualityGateRejects) {
  PrimitivePtp primitive;
  FakePtpBackend backend;

  backend.plan_result.success = false;
  backend.plan_result.reason = PrimitiveFailReason::WRIST_FLIP_DETECTED;
  backend.plan_result.message =
      "PTP quality gate rejected: wrist flip guard reject";

  const PrimitiveResult result = primitive.execute(
      make_joint_goal({0.0, 0.1, 0.2, 0.3, 0.4, 0.5}), backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::WRIST_FLIP_DETECTED);
  EXPECT_TRUE(backend.set_joint_target_called);
  EXPECT_TRUE(backend.plan_called);
}

TEST(PrimitivePtpTest, PtpFailureWhenMoveGroupServerUnavailable) {
  PrimitivePtp primitive;
  FakePtpBackend backend;

  backend.server_available = false;

  const PrimitiveResult result = primitive.execute(
      make_joint_goal({0.0, 0.1, 0.2, 0.3, 0.4, 0.5}), backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::PLANNING_TIMEOUT);
  EXPECT_FALSE(backend.set_joint_target_called);
  EXPECT_FALSE(backend.plan_called);
}
} // namespace
} // namespace primitives
