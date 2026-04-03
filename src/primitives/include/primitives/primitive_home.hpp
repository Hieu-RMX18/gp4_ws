// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include "primitives/primitive_base.hpp"
#include "primitives/primitive_types.hpp"

namespace primitives
{
struct HomeScalingConfig
{
  double velocity_cap = 0.3;
  double acceleration_cap = 0.2;
  double default_velocity = 0.3;
  double default_acceleration = 0.2;
};

class HomeExecutionBackend
{
public:
  virtual ~HomeExecutionBackend() = default;

  virtual bool wait_for_servers(std::string & reason) = 0;
  virtual bool set_named_target_home(std::string & reason) = 0;
  virtual HomeScalingConfig scaling_config() const = 0;
  virtual PrimitiveResult plan_with_pipeline(double velocity_scale, double acceleration_scale) = 0;
};

class PrimitiveHome final : public PrimitiveBase
{
public:
  PrimitiveType type() const override {return PrimitiveType::HOME;}

  PrimitiveResult execute(MoveGroupInterface & mgi);
  PrimitiveResult execute(const ExecuteMotionGoal & goal, MoveGroupInterface & mgi) override;

  PrimitiveResult execute(HomeExecutionBackend & backend);
};
}  // namespace primitives
