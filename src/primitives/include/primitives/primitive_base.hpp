// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <utility>

#include <interfaces/action/execute_motion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "primitives/primitive_types.hpp"

namespace primitives
{
using ExecuteMotionGoal = interfaces::action::ExecuteMotion::Goal;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PrimitiveBase
{
public:
  virtual ~PrimitiveBase() = default;
  virtual PrimitiveType type() const = 0;
  virtual PrimitiveResult execute(const ExecuteMotionGoal & goal, MoveGroupInterface & mgi) = 0;

protected:
  static PrimitiveResult make_failure(PrimitiveFailReason reason, std::string message)
  {
    PrimitiveResult result;
    result.success = false;
    result.reason = reason;
    result.message = std::move(message);
    return result;
  }
};
}  // namespace primitives
