// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "primitives/primitive_approach.hpp"
#include "primitives/primitive_retract.hpp"

namespace primitives
{
namespace
{
class FakeLinearBackend final : public LinearExecutionBackend
{
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

  bool wait_for_servers(std::string & reason) override
  {
    if (server_available)
    {
      reason.clear();
      return true;
    }

    reason = "server unavailable";
    return false;
  }

  bool normalize_pose(geometry_msgs::msg::Pose & pose, std::string & reason) override
  {
    (void)pose;

    if (normalize_ok)
    {
      reason.clear();
      return true;
    }

    reason = "quaternion norm is zero";
    return false;
  }

  bool get_current_pose_world(geometry_msgs::msg::Pose & pose, std::string & reason) override
  {
    if (!current_pose_ok)
    {
      reason = "current pose unavailable";
      return false;
    }

    pose = current_pose;
    reason.clear();
    return true;
  }

  bool compute_cartesian_path(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double & fraction,
    std::string & reason) override
  {
    compute_called = true;
    last_waypoints = waypoints;

    if (!compute_ok)
    {
      reason = compute_reason;
      return false;
    }

    fraction = cartesian_fraction;
    reason.clear();
    return true;
  }

  LinearScalingConfig scaling_config() const override
  {
    return scales;
  }

  PrimitiveResult postprocess_and_validate(
    double velocity_scale,
    double acceleration_scale,
    double cartesian_fraction_in) override
  {
    (void)velocity_scale;
    (void)acceleration_scale;
    (void)cartesian_fraction_in;
    postprocess_called = true;
    return postprocess_result;
  }
};

ApproachGoal make_approach_goal(double distance)
{
  ApproachGoal goal;
  goal.target_pose.position.x = 0.3;
  goal.target_pose.position.y = 0.0;
  goal.target_pose.position.z = 0.5;
  goal.target_pose.orientation.w = 1.0;
  goal.approach_distance = distance;
  goal.velocity_scale = 0.2;
  goal.acceleration_scale = 0.1;
  return goal;
}

TEST(PrimitiveApproachTest, SuccessWithPositiveDistance)
{
  PrimitiveApproach primitive;
  FakeLinearBackend backend;

  backend.cartesian_fraction = 1.0;
  backend.postprocess_result.success = true;
  backend.postprocess_result.reason = PrimitiveFailReason::UNKNOWN;

  const PrimitiveResult result = primitive.execute(make_approach_goal(0.1), backend);

  EXPECT_TRUE(result.success);
  ASSERT_EQ(backend.last_waypoints.size(), 2U);
  EXPECT_NEAR(backend.last_waypoints[0].position.z, 0.4, 1e-9);
  EXPECT_NEAR(backend.last_waypoints[1].position.z, 0.5, 1e-9);
}

TEST(PrimitiveApproachTest, FailureWithDistanceZero)
{
  PrimitiveApproach primitive;
  FakeLinearBackend backend;

  const PrimitiveResult result = primitive.execute(make_approach_goal(0.0), backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::INVALID_DISTANCE_PARAM);
  EXPECT_FALSE(backend.compute_called);
}

TEST(PrimitiveApproachTest, FailureWithNegativeDistance)
{
  PrimitiveApproach primitive;
  FakeLinearBackend backend;

  const PrimitiveResult result = primitive.execute(make_approach_goal(-0.05), backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::INVALID_DISTANCE_PARAM);
  EXPECT_FALSE(backend.compute_called);
}

TEST(PrimitiveApproachTest, FailureWhenPreApproachPoseIsUnreachable)
{
  PrimitiveApproach primitive;
  FakeLinearBackend backend;

  backend.compute_ok = false;
  backend.compute_reason = "pre-approach pose unreachable";

  const PrimitiveResult result = primitive.execute(make_approach_goal(0.1), backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::UNKNOWN);
  EXPECT_NE(result.message.find("unreachable"), std::string::npos);
}

TEST(PrimitiveRetractTest, SuccessWithPositiveDistance)
{
  PrimitiveRetract primitive;
  FakeLinearBackend backend;

  backend.current_pose.position.x = 0.2;
  backend.current_pose.position.y = -0.1;
  backend.current_pose.position.z = 0.2;
  backend.current_pose.orientation.w = 1.0;
  backend.cartesian_fraction = 1.0;
  backend.postprocess_result.success = true;
  backend.postprocess_result.reason = PrimitiveFailReason::UNKNOWN;

  RetractGoal goal;
  goal.retract_distance = 0.15;
  goal.velocity_scale = 0.2;
  goal.acceleration_scale = 0.1;

  const PrimitiveResult result = primitive.execute(goal, backend);

  EXPECT_TRUE(result.success);
  ASSERT_EQ(backend.last_waypoints.size(), 2U);
  EXPECT_NEAR(backend.last_waypoints[0].position.z, 0.2, 1e-9);
  EXPECT_NEAR(backend.last_waypoints[1].position.z, 0.35, 1e-9);
}

TEST(PrimitiveRetractTest, FailureWhenDistanceIsNotPositive)
{
  PrimitiveRetract primitive;

  {
    FakeLinearBackend backend;
    RetractGoal goal;
    goal.retract_distance = 0.0;

    const PrimitiveResult result = primitive.execute(goal, backend);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, PrimitiveFailReason::INVALID_DISTANCE_PARAM);
    EXPECT_FALSE(backend.compute_called);
  }

  {
    FakeLinearBackend backend;
    RetractGoal goal;
    goal.retract_distance = -0.2;

    const PrimitiveResult result = primitive.execute(goal, backend);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, PrimitiveFailReason::INVALID_DISTANCE_PARAM);
    EXPECT_FALSE(backend.compute_called);
  }
}
}  // namespace
}  // namespace primitives
