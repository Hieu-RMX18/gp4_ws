// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "primitives/primitive_base.hpp"
#include "primitives/primitive_types.hpp"

namespace primitives {
enum class GoalType { POSE = 0, JOINTS = 1, NAMED = 2 };

struct SequenceStep {
  PrimitiveType type = PrimitiveType::UNKNOWN;
  GoalType goal_type = GoalType::POSE;
  geometry_msgs::msg::Pose target_pose;    // for LIN, APPROACH, CIRC goal
  geometry_msgs::msg::Pose auxiliary_pose; // for CIRC interim point
  std::vector<double> joint_target;        // for PTP joint-space
  std::string named_target;                // e.g. "home", "start"
  double approach_distance = 0.0;          // for APPROACH
  double retract_distance = 0.0;           // for RETRACT
  double velocity_scale = 0.0;
  double acceleration_scale = 0.0;
  double blend_radius = 0.0; // used in MoveGroupSequenceAction path
};

class BlendedSequenceExecutionBackend {
public:
  virtual ~BlendedSequenceExecutionBackend() = default;

  virtual bool sequence_action_available() = 0;
  virtual PrimitiveResult
  execute_sequence_action(const std::vector<SequenceStep> &steps) = 0;
  virtual PrimitiveResult execute_substep(const SequenceStep &step) = 0;
};

class PrimitiveBlendedSequence final : public PrimitiveBase {
public:
  PrimitiveType type() const override {
    return PrimitiveType::BLENDED_SEQUENCE;
  }
  PrimitiveResult execute(const std::vector<SequenceStep> &steps,
                          MoveGroupInterface &mgi);
  PrimitiveResult execute(const std::vector<SequenceStep> &steps,
                          BlendedSequenceExecutionBackend &backend);
  PrimitiveResult execute(const ExecuteMotionGoal &goal,
                          MoveGroupInterface &mgi) override;
};
} // namespace primitives
