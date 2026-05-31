// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#include "primitives/primitive_dispatcher.hpp"

#include <map>
#include <string>

#include "primitives/primitive_approach.hpp"
#include "primitives/primitive_blended_sequence.hpp"
#include "primitives/primitive_circ.hpp"
#include "primitives/primitive_home.hpp"
#include "primitives/primitive_lin.hpp"
#include "primitives/primitive_ptp.hpp"
#include "primitives/primitive_retract.hpp"

namespace primitives {
namespace {
PrimitiveResult dispatch_home(const ExecuteMotionGoal &goal,
                              MoveGroupInterface &mgi) {
  PrimitiveHome primitive;
  return primitive.execute(goal, mgi);
}

PrimitiveResult dispatch_ptp(const ExecuteMotionGoal &goal,
                             MoveGroupInterface &mgi) {
  PrimitivePtp primitive;
  return primitive.execute(goal, mgi);
}

PrimitiveResult dispatch_lin(const ExecuteMotionGoal &goal,
                             MoveGroupInterface &mgi) {
  PrimitiveLin primitive;
  return primitive.execute(goal, mgi);
}

PrimitiveResult dispatch_approach(const ExecuteMotionGoal &goal,
                                  MoveGroupInterface &mgi) {
  PrimitiveApproach primitive;
  return primitive.execute(goal, mgi);
}

PrimitiveResult dispatch_retract(const ExecuteMotionGoal &goal,
                                 MoveGroupInterface &mgi) {
  PrimitiveRetract primitive;
  return primitive.execute(goal, mgi);
}

PrimitiveResult dispatch_circ(const ExecuteMotionGoal &goal,
                              MoveGroupInterface &mgi) {
  PrimitiveCirc primitive;
  return primitive.execute(goal, mgi);
}

PrimitiveResult dispatch_blended_sequence(const ExecuteMotionGoal &goal,
                                          MoveGroupInterface &mgi) {
  PrimitiveBlendedSequence primitive;
  return primitive.execute(goal, mgi);
}

const PrimitiveDispatcher::DispatchTable &dispatch_table() {
  static const PrimitiveDispatcher::DispatchTable table = {
      {PrimitiveType::HOME, dispatch_home},
      {PrimitiveType::PTP, dispatch_ptp},
      {PrimitiveType::LIN, dispatch_lin},
      {PrimitiveType::APPROACH, dispatch_approach},
      {PrimitiveType::RETRACT, dispatch_retract},
      {PrimitiveType::CIRC, dispatch_circ},
      {PrimitiveType::BLENDED_SEQUENCE, dispatch_blended_sequence},
  };

  return table;
}

PrimitiveResult
make_unknown_primitive_result(const std::string &primitive_type) {
  PrimitiveResult result;
  result.success = false;
  result.reason = PrimitiveFailReason::UNKNOWN;
  result.message = "Unsupported primitive_type: '" + primitive_type + "'";
  return result;
}
} // namespace

PrimitiveResult PrimitiveDispatcher::dispatch(const ExecuteMotionGoal &goal,
                                              MoveGroupInterface &mgi) const {
  const PrimitiveType primitive_type = from_string(goal.primitive_type);
  const auto &table = dispatch_table();
  const auto table_it = table.find(primitive_type);

  if (table_it == table.end()) {
    return make_unknown_primitive_result(goal.primitive_type);
  }

  return table_it->second(goal, mgi);
}

PrimitiveResult PrimitiveDispatcher::dispatch_with_goal_handlers(
    const ExecuteMotionGoal &goal,
    const GoalOnlyDispatchTable &handlers) const {
  const PrimitiveType primitive_type = from_string(goal.primitive_type);
  const auto table_it = handlers.find(primitive_type);
  if (table_it == handlers.end()) {
    return make_unknown_primitive_result(goal.primitive_type);
  }

  return table_it->second(goal);
}

bool PrimitiveDispatcher::supports(PrimitiveType primitive_type) const {
  return dispatch_table().find(primitive_type) != dispatch_table().end();
}
} // namespace primitives
