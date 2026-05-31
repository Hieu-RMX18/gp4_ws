#include <gtest/gtest.h>

#include "motion_core/planner_router.hpp"

namespace motion_core {
namespace {
TEST(PlannerRouterTest, RoutesKnownPilzPrimitives) {
  PlannerRouter router;

  EXPECT_EQ(router.route_planner("LIN"), "PILZ_LIN");
  EXPECT_EQ(router.route_planner("ptp"), "PILZ_PTP");
  EXPECT_EQ(router.route_planner(" CIRC "), "PILZ_CIRC");
  EXPECT_EQ(router.route_planner("MOVE_REL"), "PILZ_LIN");
}

TEST(PlannerRouterTest, RoutesComplexAndObstacleToOmpl) {
  PlannerRouter router;

  EXPECT_EQ(router.route_planner("complex"), "OMPL_RRTConnect");
  EXPECT_EQ(router.route_planner("LIN", true), "OMPL_RRTConnect");
}

TEST(PlannerRouterTest, ReturnsEmptyForUnknownPrimitive) {
  PlannerRouter router;

  EXPECT_TRUE(router.route_planner("HOME").empty());
}

// Step 5.2: MOVE_JOINT and MOVE_JOINTS route to PILZ_PTP
TEST(PlannerRouterTest, RoutesMoveJointToPilzPtp) {
  PlannerRouter router;

  EXPECT_EQ(router.route_planner("MOVE_JOINT"), "PILZ_PTP");
}

TEST(PlannerRouterTest, RoutesMoveJointsToPilzPtp) {
  PlannerRouter router;

  EXPECT_EQ(router.route_planner("MOVE_JOINTS"), "PILZ_PTP");
}

// Step 5.2: Non-motion primitives return empty (no planner needed)
TEST(PlannerRouterTest, NonMotionPrimitivesReturnEmpty) {
  PlannerRouter router;

  EXPECT_TRUE(router.route_planner("STOP").empty());
  EXPECT_TRUE(router.route_planner("WAIT").empty());
  EXPECT_TRUE(router.route_planner("SET_SPEED").empty());
  EXPECT_TRUE(router.route_planner("ALARM_RESET").empty());
  EXPECT_TRUE(router.route_planner("IO_SET").empty());
}
} // namespace
} // namespace motion_core
