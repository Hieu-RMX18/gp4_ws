#include <gtest/gtest.h>

#include "motion_core/planner_router.hpp"

namespace motion_core
{
namespace
{
TEST(PlannerRouterTest, RoutesKnownPilzPrimitives)
{
  PlannerRouter router;

  EXPECT_EQ(router.route_planner("LIN"), "PILZ_LIN");
  EXPECT_EQ(router.route_planner("ptp"), "PILZ_PTP");
  EXPECT_EQ(router.route_planner(" CIRC "), "PILZ_CIRC");
}

TEST(PlannerRouterTest, RoutesComplexAndObstacleToOmpl)
{
  PlannerRouter router;

  EXPECT_EQ(router.route_planner("complex"), "OMPL_RRTConnect");
  EXPECT_EQ(router.route_planner("LIN", true), "OMPL_RRTConnect");
}

TEST(PlannerRouterTest, ReturnsEmptyForUnknownPrimitive)
{
  PlannerRouter router;

  EXPECT_TRUE(router.route_planner("HOME").empty());
}
}  // namespace
}  // namespace motion_core
