// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include <gtest/gtest.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/action/execute_motion.hpp"
#include "motion_core/ik_selector.hpp"
#include "motion_core/joint_position_guard.hpp"
#include "motion_core/orientation_filter.hpp"
#include "motion_core/planner_router.hpp"
#include "motion_core/primitive_router_dispatch.hpp"
#include "motion_core/seed_manager.hpp"
#include "motion_core/trajectory_post_processor.hpp"

namespace motion_core {
namespace {

// ------------------------------------------------------------------
// Static method tests (no ROS / MoveIt mocking required)
// ------------------------------------------------------------------

TEST(PrimitiveRouterDispatchStatic, ResolvePlannerSelectionPilzLin) {
  auto sel = PrimitiveRouterDispatch::resolve_planner_selection("LIN");
  EXPECT_EQ(sel.pipeline_id, "pilz_industrial_motion_planner");
  EXPECT_EQ(sel.planner_id, "LIN");
}

TEST(PrimitiveRouterDispatchStatic,
     ResolvePlannerSelectionNormalizesWhitespaceUnderscoreDash) {
  auto sel = PrimitiveRouterDispatch::resolve_planner_selection("pilz-lin");
  EXPECT_EQ(sel.planner_id, "LIN");

  sel = PrimitiveRouterDispatch::resolve_planner_selection("PILZ_LIN");
  EXPECT_EQ(sel.planner_id, "LIN");

  sel = PrimitiveRouterDispatch::resolve_planner_selection("  lin  ");
  EXPECT_EQ(sel.planner_id, "LIN");
}

TEST(PrimitiveRouterDispatchStatic, ResolvePlannerSelectionPilzPtp) {
  auto sel = PrimitiveRouterDispatch::resolve_planner_selection("PTP");
  EXPECT_EQ(sel.pipeline_id, "pilz_industrial_motion_planner");
  EXPECT_EQ(sel.planner_id, "PTP");
}

TEST(PrimitiveRouterDispatchStatic, ResolvePlannerSelectionPilzCirc) {
  auto sel = PrimitiveRouterDispatch::resolve_planner_selection("CIRC");
  EXPECT_EQ(sel.pipeline_id, "pilz_industrial_motion_planner");
  EXPECT_EQ(sel.planner_id, "CIRC");
}

TEST(PrimitiveRouterDispatchStatic, ResolvePlannerSelectionOmplRrtConnect) {
  auto sel = PrimitiveRouterDispatch::resolve_planner_selection("RRTConnect");
  EXPECT_EQ(sel.pipeline_id, "ompl");
  EXPECT_EQ(sel.planner_id, "RRTConnect");

  sel = PrimitiveRouterDispatch::resolve_planner_selection("ompl_rrt-connect");
  EXPECT_EQ(sel.planner_id, "RRTConnect");
}

TEST(PrimitiveRouterDispatchStatic, ResolvePlannerSelectionUnknownPassthrough) {
  auto sel =
      PrimitiveRouterDispatch::resolve_planner_selection("UNKNOWN_PLANNER");
  EXPECT_EQ(sel.pipeline_id, "");
  EXPECT_EQ(sel.planner_id, "UNKNOWN_PLANNER");
}

TEST(PrimitiveRouterDispatchStatic, IsPoseGoalRequiredLinAlways) {
  EXPECT_TRUE(PrimitiveRouterDispatch::is_pose_goal_required("LIN", false));
  EXPECT_TRUE(PrimitiveRouterDispatch::is_pose_goal_required("LIN", true));
}

TEST(PrimitiveRouterDispatchStatic, IsPoseGoalRequiredPtpConditional) {
  EXPECT_TRUE(PrimitiveRouterDispatch::is_pose_goal_required("PTP", false));
  EXPECT_FALSE(PrimitiveRouterDispatch::is_pose_goal_required("PTP", true));
}

TEST(PrimitiveRouterDispatchStatic, IsPoseGoalRequiredOthersFalse) {
  EXPECT_FALSE(
      PrimitiveRouterDispatch::is_pose_goal_required("HOME", false));
  EXPECT_FALSE(
      PrimitiveRouterDispatch::is_pose_goal_required("CIRC", false));
  EXPECT_FALSE(
      PrimitiveRouterDispatch::is_pose_goal_required("MOVE_REL", false));
}

TEST(PrimitiveRouterDispatchStatic, QuaternionNormSq) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
  EXPECT_DOUBLE_EQ(PrimitiveRouterDispatch::quaternion_norm_sq(q), 1.0);

  q.x = 1.0; q.y = 2.0; q.z = 3.0; q.w = 4.0;
  EXPECT_DOUBLE_EQ(PrimitiveRouterDispatch::quaternion_norm_sq(q), 30.0);

  q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 0.0;
  EXPECT_DOUBLE_EQ(PrimitiveRouterDispatch::quaternion_norm_sq(q), 0.0);
}

TEST(PrimitiveRouterDispatchStatic, MaxAbsValueEmpty) {
  EXPECT_DOUBLE_EQ(PrimitiveRouterDispatch::max_abs_value({}), 0.0);
}

TEST(PrimitiveRouterDispatchStatic, MaxAbsValueMixed) {
  EXPECT_DOUBLE_EQ(
      PrimitiveRouterDispatch::max_abs_value({1.0, -3.0, 2.0}), 3.0);
  EXPECT_DOUBLE_EQ(PrimitiveRouterDispatch::max_abs_value({-5.0}), 5.0);
  EXPECT_DOUBLE_EQ(
      PrimitiveRouterDispatch::max_abs_value({0.1, -0.1, 0.0}), 0.1);
}

TEST(PrimitiveRouterDispatchStatic, FormatJointVector) {
  EXPECT_EQ(PrimitiveRouterDispatch::format_joint_vector({}), "[]");
  EXPECT_EQ(PrimitiveRouterDispatch::format_joint_vector({1.0}),
            "[1.0000]");
  EXPECT_EQ(
      PrimitiveRouterDispatch::format_joint_vector({1.0, -2.5}),
      "[1.0000, -2.5000]");
  EXPECT_EQ(
      PrimitiveRouterDispatch::format_joint_vector({0.12345678}),
      "[0.1235]");
}

TEST(PrimitiveRouterDispatchStatic, MoveGroupSequenceActionNameHumble) {
  EXPECT_STREQ(PrimitiveRouterDispatch::move_group_sequence_action_name(),
               "sequence_move_group");
}

} // namespace
} // namespace motion_core
