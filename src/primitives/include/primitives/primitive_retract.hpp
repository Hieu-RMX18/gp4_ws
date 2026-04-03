// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "primitives/primitive_lin.hpp"

namespace primitives
{
struct RetractGoal
{
  double retract_distance = 0.0;
  double velocity_scale = 0.0;
  double acceleration_scale = 0.0;
};

class PrimitiveRetract final : public PrimitiveBase
{
public:
  PrimitiveType type() const override {return PrimitiveType::RETRACT;}

  PrimitiveResult execute(const RetractGoal & goal, MoveGroupInterface & mgi);
  PrimitiveResult execute(const RetractGoal & goal, LinearExecutionBackend & backend);

  PrimitiveResult execute(const ExecuteMotionGoal & goal, MoveGroupInterface & mgi) override;
};
}  // namespace primitives
