// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <string>

#include <gtest/gtest.h>

#include "primitives/primitive_dispatcher.hpp"
#include "primitives/primitive_types.hpp"

namespace primitives
{
namespace
{
TEST(PrimitiveTypesTest, AllPrimitiveTypeStringsRoundTrip)
{
  const std::array<PrimitiveType, 7> types = {
    PrimitiveType::HOME,
    PrimitiveType::PTP,
    PrimitiveType::LIN,
    PrimitiveType::APPROACH,
    PrimitiveType::RETRACT,
    PrimitiveType::CIRC,
    PrimitiveType::BLENDED_SEQUENCE,
  };

  for (const PrimitiveType type : types)
  {
    const std::string encoded = to_string(type);
    EXPECT_EQ(from_string(encoded), type) << "round-trip failed for " << encoded;
  }
}

TEST(PrimitiveTypesTest, UnknownStringMapsToUnknownType)
{
  EXPECT_EQ(from_string("NOT_A_PRIMITIVE"), PrimitiveType::UNKNOWN);
}

TEST(PrimitiveDispatcherTest, SupportsAllSevenPrimitiveTypes)
{
  PrimitiveDispatcher dispatcher;

  EXPECT_TRUE(dispatcher.supports(PrimitiveType::HOME));
  EXPECT_TRUE(dispatcher.supports(PrimitiveType::PTP));
  EXPECT_TRUE(dispatcher.supports(PrimitiveType::LIN));
  EXPECT_TRUE(dispatcher.supports(PrimitiveType::APPROACH));
  EXPECT_TRUE(dispatcher.supports(PrimitiveType::RETRACT));
  EXPECT_TRUE(dispatcher.supports(PrimitiveType::CIRC));
  EXPECT_TRUE(dispatcher.supports(PrimitiveType::BLENDED_SEQUENCE));
  EXPECT_FALSE(dispatcher.supports(PrimitiveType::UNKNOWN));
}

TEST(PrimitiveDispatcherTest, RoutesEachTypeToMatchingHandler)
{
  PrimitiveDispatcher dispatcher;

  PrimitiveDispatcher::GoalOnlyDispatchTable handlers;

  for (const PrimitiveType type : {
         PrimitiveType::HOME,
         PrimitiveType::PTP,
         PrimitiveType::LIN,
         PrimitiveType::APPROACH,
         PrimitiveType::RETRACT,
         PrimitiveType::CIRC,
         PrimitiveType::BLENDED_SEQUENCE,
       })
  {
    handlers[type] = [type](const ExecuteMotionGoal & /*goal*/) {
      PrimitiveResult result;
      result.success = true;
      result.reason = PrimitiveFailReason::UNKNOWN;
      result.message = to_string(type);
      return result;
    };

    ExecuteMotionGoal goal;
    goal.primitive_type = to_string(type);

    const PrimitiveResult result = dispatcher.dispatch_with_goal_handlers(goal, handlers);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.message, to_string(type));
  }
}

TEST(PrimitiveDispatcherTest, UnknownTypeFailsBeforeDispatch)
{
  PrimitiveDispatcher dispatcher;
  ExecuteMotionGoal goal;
  goal.primitive_type = "UNSUPPORTED";

  const PrimitiveResult result = dispatcher.dispatch_with_goal_handlers(goal, {});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, PrimitiveFailReason::UNKNOWN);
  EXPECT_NE(result.message.find("Unsupported primitive_type"), std::string::npos);
}
}  // namespace
}  // namespace primitives
