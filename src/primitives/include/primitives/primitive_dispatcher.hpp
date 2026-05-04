// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
#include <map>

#include "primitives/primitive_base.hpp"

namespace primitives {
class PrimitiveDispatcher final {
public:
  using DispatchHandler = std::function<PrimitiveResult(
      const ExecuteMotionGoal &, MoveGroupInterface &)>;
  using DispatchTable = std::map<PrimitiveType, DispatchHandler>;

  using GoalOnlyDispatchHandler =
      std::function<PrimitiveResult(const ExecuteMotionGoal &)>;
  using GoalOnlyDispatchTable =
      std::map<PrimitiveType, GoalOnlyDispatchHandler>;

  PrimitiveResult dispatch(const ExecuteMotionGoal &goal,
                           MoveGroupInterface &mgi) const;
  PrimitiveResult
  dispatch_with_goal_handlers(const ExecuteMotionGoal &goal,
                              const GoalOnlyDispatchTable &handlers) const;

  bool supports(PrimitiveType primitive_type) const;
};
} // namespace primitives
