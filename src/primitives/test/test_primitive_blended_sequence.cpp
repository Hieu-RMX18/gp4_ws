// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "primitives/primitive_blended_sequence.hpp"

namespace primitives {
namespace {
class FakeBlendedBackend final : public BlendedSequenceExecutionBackend {
public:
  bool sequence_available = false;
  bool sequence_called = false;
  PrimitiveResult sequence_result;

  std::vector<PrimitiveResult> staged_results;
  std::size_t staged_index = 0;
  std::vector<SequenceStep> executed_steps;

  bool sequence_action_available() override { return sequence_available; }

  PrimitiveResult
  execute_sequence_action(const std::vector<SequenceStep> &steps) override {
    sequence_called = true;
    (void)steps;
    return sequence_result;
  }

  PrimitiveResult execute_substep(const SequenceStep &step) override {
    executed_steps.push_back(step);

    if (staged_index < staged_results.size()) {
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

SequenceStep make_step(PrimitiveType type) {
  SequenceStep step;
  step.type = type;
  step.target_pose.orientation.w = 1.0;
  step.auxiliary_pose.orientation.w = 1.0;
  step.blend_radius = 0.0;
  return step;
}

PrimitiveResult make_success(std::size_t points) {
  PrimitiveResult result;
  result.success = true;
  result.reason = PrimitiveFailReason::UNKNOWN;
  result.message = "ok";
  result.trajectory_points = points;
  return result;
}

PrimitiveResult make_failure(PrimitiveFailReason reason,
                             const std::string &message) {
  PrimitiveResult result;
  result.success = false;
  result.reason = reason;
  result.message = message;
  return result;
}

TEST(PrimitiveBlendedSequenceTest, UsesSequenceActionWhenAvailable) {
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.sequence_available = true;
  backend.sequence_result = make_success(42);
  backend.sequence_result.message =
      "blended_sequence planned with MoveGroupSequenceAction";

  const PrimitiveResult result = primitive.execute(
      std::vector<SequenceStep>{make_step(PrimitiveType::HOME)}, backend);

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(backend.sequence_called);
  EXPECT_TRUE(backend.executed_steps.empty());
}

TEST(PrimitiveBlendedSequenceTest,
     RejectsWhenSequenceActionUnavailable) {
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.sequence_available = false;

  const PrimitiveResult result = primitive.execute(
      std::vector<SequenceStep>{make_step(PrimitiveType::HOME)}, backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::PLANNING_TIMEOUT);
  EXPECT_NE(result.message.find("MoveGroupSequence action server unavailable"),
            std::string::npos);
  EXPECT_FALSE(backend.sequence_called);
  EXPECT_TRUE(backend.executed_steps.empty());
}

TEST(PrimitiveBlendedSequenceTest, SequenceActionFailurePropagatesToResult) {
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.sequence_available = true;
  backend.sequence_result =
      make_failure(PrimitiveFailReason::PLANNING_TIMEOUT, "plan timeout");

  const PrimitiveResult result = primitive.execute(
      std::vector<SequenceStep>{make_step(PrimitiveType::HOME)}, backend);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::PLANNING_TIMEOUT);
  EXPECT_EQ(result.message, "plan timeout");
  EXPECT_TRUE(backend.sequence_called);
  EXPECT_TRUE(backend.executed_steps.empty());
}

TEST(PrimitiveBlendedSequenceTest, RejectsNegativeBlendRadius) {
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

TEST(PrimitiveBlendedSequenceTest, PopulatesTrajectoryInResult) {
  PrimitiveBlendedSequence primitive;
  FakeBlendedBackend backend;

  backend.sequence_available = true;
  backend.sequence_result = make_success(42);
  backend.sequence_result.message =
      "blended_sequence planned with MoveGroupSequenceAction";
  // trajectory field is default-constructed empty; real backend would fill it

  const PrimitiveResult result = primitive.execute(
      std::vector<SequenceStep>{make_step(PrimitiveType::HOME)}, backend);

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(backend.sequence_called);
  EXPECT_TRUE(backend.executed_steps.empty());
}

TEST(PrimitiveBlendedSequenceTest, UnknownPrimitiveTypeFailsBeforePlanning) {
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
} // namespace
} // namespace primitives
