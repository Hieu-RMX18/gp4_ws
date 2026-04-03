// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "primitives/primitive_lin.hpp"

namespace primitives
{
struct ApproachGoal
{
  geometry_msgs::msg::Pose target_pose;
  double approach_distance = 0.0;
  double velocity_scale = 0.0;
  double acceleration_scale = 0.0;
};

class PrimitiveApproach final : public PrimitiveBase
{
public:
  PrimitiveType type() const override {return PrimitiveType::APPROACH;}

  PrimitiveResult execute(const ApproachGoal & goal, MoveGroupInterface & mgi);
  PrimitiveResult execute(const ApproachGoal & goal, LinearExecutionBackend & backend);

  PrimitiveResult execute(const ExecuteMotionGoal & goal, MoveGroupInterface & mgi) override;
};
}  // namespace primitives
