// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "primitives/primitive_blended_sequence.hpp"

namespace primitives
{
namespace
{
class FakeBlendedBackend final : public BlendedSequenceExecutionBackend
{
public:
  bool sequence_available = false;
  bool sequence_called = false;
  PrimitiveResult sequence_result;

  std::vector<PrimitiveResult> staged_results;
  std::size_t staged_index = 0;
  std::vector<SequenceStep> executed_steps;

  bool sequence_action_available() override
  {
    return sequence_available;
  }

  PrimitiveResult execute_sequence_action(const std::vector<SequenceStep> & steps) override
  {
    sequence_called = true;
    (void)steps;
    return sequence_result;
  }

  PrimitiveResult execute_substep(const SequenceStep & step) override
  {
    executed_steps.push_back(step);

    if (staged_index < staged_results.size())
    {
      return staged_results[staged_index++];
    }

    PrimitiveResult success;
    success.success = true;
    success.reason = PrimitiveFailReason::UNKNOWN;
    success.message = "default staged success";
    success.trajectory_points = 0;
    return success;
  }
};

SequenceStep make_step(PrimitiveType type)
{
  SequenceStep step;
  step.type = type;
  step.target_pose.orientation.w = 1.0;
  step.auxiliary_pose.orientation.w = 1.0;
  step.blend_radius = 0.0;
  return step;
}

PrimitiveResult make_success(std::size_t points)
{
  PrimitiveResult result;
  result.success = true;
  result.reason = PrimitiveFailReason::UNKNOWN;
  result.message = "ok";
  result.trajectory_points = points;
  return result;
}

PrimitiveResult make_failure(PrimitiveFailReason reason, const std::string & message)
{
  PrimitiveResult result;
  result.success = false;
  result.reason = reason;
  result.message = message;
  return result;
}

TEST(PrimitiveBlendedSequenceTest, UsesSequenceActionWhenAvailable)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.sequence_available = true;
  backend.sequence_result = make_success(42);
  backend.sequence_result.message = "blended_sequence planned with MoveGroupSequenceAction";

  const PrimitiveResult result = primitive.execute(
    std::vector<SequenceStep>{make_step(PrimitiveType::HOME)},
    backend);

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(backend.sequence_called);
  EXPECT_TRUE(backend.executed_steps.empty());
}

TEST(PrimitiveBlendedSequenceTest, SingleElementHomeSequenceSucceedsInDegradedMode)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.sequence_available = false;
  backend.staged_results = {make_success(40)};

  const PrimitiveResult result = primitive.execute(
    std::vector<SequenceStep>{make_step(PrimitiveType::HOME)},
    backend);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::UNKNOWN);
  EXPECT_EQ(result.message, "blended_sequence executed in degraded mode (staged)");
  EXPECT_EQ(result.trajectory_points, 40U);
  EXPECT_FALSE(backend.sequence_called);
  ASSERT_EQ(backend.executed_steps.size(), 1U);
  EXPECT_EQ(backend.executed_steps.front().type, PrimitiveType::HOME);
}

TEST(PrimitiveBlendedSequenceTest, MixedSequenceSucceedsInDegradedMode)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.sequence_available = false;
  backend.staged_results = {
    make_success(20),
    make_success(30),
    make_success(25),
    make_success(15),
    make_success(10),
  };

  SequenceStep home = make_step(PrimitiveType::HOME);

  SequenceStep ptp = make_step(PrimitiveType::PTP);
  ptp.joint_target = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};

  SequenceStep approach = make_step(PrimitiveType::APPROACH);
  approach.approach_distance = 0.1;

  SequenceStep lin = make_step(PrimitiveType::LIN);
  lin.target_pose.position.x = 0.2;
  lin.target_pose.position.z = 0.3;

  SequenceStep retract = make_step(PrimitiveType::RETRACT);
  retract.retract_distance = 0.12;

  const PrimitiveResult result = primitive.execute({home, ptp, approach, lin, retract}, backend);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.message, "blended_sequence executed in degraded mode (staged)");
  EXPECT_EQ(result.trajectory_points, 100U);
  EXPECT_EQ(backend.executed_steps.size(), 5U);
}

TEST(PrimitiveBlendedSequenceTest, MixedHomePtpLinSequenceSucceeds)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.sequence_available = false;
  backend.staged_results = {
    make_success(30),
    make_success(35),
    make_success(25),
  };

  SequenceStep home = make_step(PrimitiveType::HOME);

  SequenceStep ptp = make_step(PrimitiveType::PTP);
  ptp.joint_target = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};

  SequenceStep lin = make_step(PrimitiveType::LIN);
  lin.target_pose.position.x = 0.15;
  lin.target_pose.position.z = 0.25;

  const PrimitiveResult result = primitive.execute({home, ptp, lin}, backend);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.message, "blended_sequence executed in degraded mode (staged)");
  EXPECT_EQ(result.trajectory_points, 90U);
  ASSERT_EQ(backend.executed_steps.size(), 3U);
  EXPECT_EQ(backend.executed_steps[0].type, PrimitiveType::HOME);
  EXPECT_EQ(backend.executed_steps[1].type, PrimitiveType::PTP);
  EXPECT_EQ(backend.executed_steps[2].type, PrimitiveType::LIN);
}

TEST(PrimitiveBlendedSequenceTest, StopsOnSubStepPlanningFailure)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.staged_results = {
    make_success(20),
    make_success(20),
    make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, "planner timeout"),
    make_success(20),
  };

  const PrimitiveResult result = primitive.execute(
    {
      make_step(PrimitiveType::HOME),
      make_step(PrimitiveType::PTP),
      make_step(PrimitiveType::LIN),
      make_step(PrimitiveType::RETRACT),
    },
    backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::SUB_PRIMITIVE_FAILED);
  EXPECT_NE(result.message.find("step[2] failed: PLANNING_TIMEOUT"), std::string::npos);
  EXPECT_EQ(backend.executed_steps.size(), 3U);
}

TEST(PrimitiveBlendedSequenceTest, StopsOnSubStepQualityGateFailure)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.staged_results = {
    make_success(20),
    make_success(20),
    make_success(20),
    make_failure(PrimitiveFailReason::WRIST_FLIP_DETECTED, "wrist flip guard reject"),
  };

  const PrimitiveResult result = primitive.execute(
    {
      make_step(PrimitiveType::HOME),
      make_step(PrimitiveType::PTP),
      make_step(PrimitiveType::APPROACH),
      make_step(PrimitiveType::LIN),
    },
    backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::SUB_PRIMITIVE_FAILED);
  EXPECT_NE(result.message.find("step[3] failed: WRIST_FLIP_DETECTED"), std::string::npos);
  EXPECT_EQ(backend.executed_steps.size(), 4U);
}

TEST(PrimitiveBlendedSequenceTest, RejectsNegativeBlendRadius)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  SequenceStep step = make_step(PrimitiveType::HOME);
  step.blend_radius = -0.01;

  const PrimitiveResult result = primitive.execute({step}, backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::UNKNOWN);
  EXPECT_NE(result.message.find("invalid blend_radius"), std::string::npos);
  EXPECT_FALSE(backend.sequence_called);
  EXPECT_TRUE(backend.executed_steps.empty());
}

TEST(PrimitiveBlendedSequenceTest, FailsWhenMergedStagedTrajectoryExceedsPointLimit)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.staged_results = {
    make_success(120),
    make_success(90),
  };

  const PrimitiveResult result = primitive.execute(
    {
      make_step(PrimitiveType::HOME),
      make_step(PrimitiveType::PTP),
    },
    backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::TRAJECTORY_TOO_LONG);
  EXPECT_NE(result.message.find("> 200"), std::string::npos);
}

TEST(PrimitiveBlendedSequenceTest, CircDegenerateGeometryIsPreservedInParentMessage)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.staged_results = {
    make_failure(PrimitiveFailReason::DEGENERATE_GEOMETRY, "auxiliary point equals start"),
  };

  const PrimitiveResult result = primitive.execute(
    {
      make_step(PrimitiveType::CIRC),
    },
    backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::SUB_PRIMITIVE_FAILED);
  EXPECT_NE(result.message.find("DEGENERATE_GEOMETRY"), std::string::npos);
}

TEST(PrimitiveBlendedSequenceTest, UnknownPrimitiveTypeFailsBeforePlanning)
{
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  const PrimitiveResult result = primitive.execute(
    {
      make_step(PrimitiveType::UNKNOWN),
    },
    backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::UNKNOWN);
  EXPECT_TRUE(backend.executed_steps.empty());
  EXPECT_FALSE(backend.sequence_called);
}
}  // namespace
}  // namespace primitives
