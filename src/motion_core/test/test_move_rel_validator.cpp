#include <gtest/gtest.h>

#include "motion_core/move_rel_validator.hpp"

namespace motion_core {
namespace {

// ── validate_move_rel_frame ──

TEST(MoveRelValidatorTest, AcceptsEmptyFrame) {
  std::string reason;
  EXPECT_TRUE(validate_move_rel_frame("", reason));
}

TEST(MoveRelValidatorTest, AcceptsBaseLinkFrame) {
  std::string reason;
  EXPECT_TRUE(validate_move_rel_frame("base_link", reason));
}

TEST(MoveRelValidatorTest, RejectsUnsupportedFrame) {
  std::string reason;
  EXPECT_FALSE(validate_move_rel_frame("tool0", reason));
  EXPECT_NE(reason.find("unsupported reference_frame"), std::string::npos);
  EXPECT_NE(reason.find("tool0"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsWorldFrame) {
  std::string reason;
  EXPECT_FALSE(validate_move_rel_frame("world", reason));
}

// ── validate_move_rel_deltas ──

TEST(MoveRelValidatorTest, RejectsAllZeroDeltas) {
  std::string reason;
  EXPECT_FALSE(validate_move_rel_deltas(0.0, 0.0, 0.0, reason));
  EXPECT_NE(reason.find("all delta components are zero"), std::string::npos);
}

TEST(MoveRelValidatorTest, AcceptsSingleAxisDelta) {
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.0, 0.0, 0.03, reason));
}

TEST(MoveRelValidatorTest, AcceptsMultiAxisDelta) {
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.02, 0.0, 0.01, reason));
}

TEST(MoveRelValidatorTest, AcceptsNegativeDelta) {
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.0, 0.0, -0.02, reason));
}

TEST(MoveRelValidatorTest, AcceptsDeltaExactlyAtLimit) {
  // norm = 0.08 exactly — should pass (this pass raises limit from 0.03 to
  // 0.08)
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.08, 0.0, 0.0, reason));
}

TEST(MoveRelValidatorTest, RejectsDeltaNormExceedsLimit) {
  // norm = sqrt(0.05^2 + 0.06^2) ≈ 0.0781 < 0.08 (OK)
  // norm = sqrt(0.06^2 + 0.06^2) ≈ 0.0849 > 0.08 (reject)
  std::string reason;
  EXPECT_FALSE(validate_move_rel_deltas(0.06, 0.06, 0.0, reason));
  EXPECT_NE(reason.find("exceeds safety limit"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsLargeSingleAxisDelta) {
  // 0.09 > 0.08 limit
  std::string reason;
  EXPECT_FALSE(validate_move_rel_deltas(0.0, 0.0, 0.09, reason));
  EXPECT_NE(reason.find("exceeds safety limit"), std::string::npos);
}

// ── compute_move_rel_target ──

TEST(MoveRelValidatorTest, ComputeTargetAddsDeltas) {
  geometry_msgs::msg::Pose current;
  current.position.x = 0.3;
  current.position.y = 0.1;
  current.position.z = 0.4;
  current.orientation.x = 0.0;
  current.orientation.y = 1.0;
  current.orientation.z = 0.0;
  current.orientation.w = 0.0;

  auto target = compute_move_rel_target(current, 0.05, -0.02, 0.10);

  EXPECT_DOUBLE_EQ(target.position.x, 0.35);
  EXPECT_DOUBLE_EQ(target.position.y, 0.08);
  EXPECT_DOUBLE_EQ(target.position.z, 0.50);
}

TEST(MoveRelValidatorTest, ComputeTargetPreservesOrientation) {
  geometry_msgs::msg::Pose current;
  current.position.x = 0.3;
  current.position.y = 0.0;
  current.position.z = 0.4;
  current.orientation.x = 0.0;
  current.orientation.y = 0.707;
  current.orientation.z = 0.0;
  current.orientation.w = 0.707;

  auto target = compute_move_rel_target(current, 0.0, 0.0, 0.10);

  // Orientation must be identical — not recomputed, not zeroed
  EXPECT_DOUBLE_EQ(target.orientation.x, current.orientation.x);
  EXPECT_DOUBLE_EQ(target.orientation.y, current.orientation.y);
  EXPECT_DOUBLE_EQ(target.orientation.z, current.orientation.z);
  EXPECT_DOUBLE_EQ(target.orientation.w, current.orientation.w);
}

TEST(MoveRelValidatorTest, ComputeTargetWithZeroDeltaEqualsCurrentPosition) {
  geometry_msgs::msg::Pose current;
  current.position.x = 0.3;
  current.position.y = 0.1;
  current.position.z = 0.4;
  current.orientation.w = 1.0;

  auto target = compute_move_rel_target(current, 0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(target.position.x, current.position.x);
  EXPECT_DOUBLE_EQ(target.position.y, current.position.y);
  EXPECT_DOUBLE_EQ(target.position.z, current.position.z);
}

// ── validate_move_rel_target_bounds ──

TEST(MoveRelValidatorTest, AcceptsTargetInsideBounds) {
  geometry_msgs::msg::Pose target;
  target.position.x = 0.15;
  target.position.y = -0.10;
  target.position.z = 0.22;

  std::string reason;
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
}

TEST(MoveRelValidatorTest, AcceptsTargetAtBoundaryEdge) {
  geometry_msgs::msg::Pose target;
  target.position.x = MoveRelLimits::kXMin;
  target.position.y = MoveRelLimits::kYMin;
  target.position.z = MoveRelLimits::kZMax;

  std::string reason;
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
}

TEST(MoveRelValidatorTest, RejectsTargetBelowZMin) {
  geometry_msgs::msg::Pose target;
  target.position.x = 0.0;
  target.position.y = 0.0;
  target.position.z = 0.09; // below kZMin = 0.20

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetAboveXMax) {
  geometry_msgs::msg::Pose target;
  target.position.x = 0.39; // above kXMax = 0.38
  target.position.y = 0.0;
  target.position.z = 0.45; // inside workspace z bounds

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetBelowYMin) {
  geometry_msgs::msg::Pose target;
  target.position.x = 0.0;
  target.position.y = -0.26; // below kYMin = -0.25
  target.position.z = 0.45;  // inside workspace z bounds

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
}

TEST(MoveRelValidatorTest, RejectsTargetInsideTableClearanceGuard) {
  // The table_clearance_guard center z=0.09, half-size z=0.09 → zone
  // z_max=0.18. Workspace z_min=0.20 means the zone is entirely below the
  // workspace floor. This test verifies the point is still rejected (by
  // workspace bounds).
  geometry_msgs::msg::Pose target;
  target.position.x = 0.10;
  target.position.y = 0.05;
  target.position.z = 0.12;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetInsideAvoidLeftRegion) {
  geometry_msgs::msg::Pose target;
  target.position.x = -0.22;
  target.position.y = 0.21;
  target.position.z = 0.35;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("avoid_left_region"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetInsideWallRegion) {
  geometry_msgs::msg::Pose target;
  target.position.x = 0.34;
  target.position.y = 0.32;
  target.position.z = 0.35;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("wall_region"), std::string::npos);
}

// ── Integration: full resolve-then-validate flow ──

TEST(MoveRelValidatorTest, FullFlowValidDeltaInsideBounds) {
  // Simulate: current in free space, move up 3cm -> stays in bounds
  geometry_msgs::msg::Pose current;
  current.position.x = 0.10;
  current.position.y = -0.10;
  current.position.z = 0.22;
  current.orientation.x = 0.0;
  current.orientation.y = 1.0;
  current.orientation.z = 0.0;
  current.orientation.w = 0.0;

  std::string reason;
  ASSERT_TRUE(validate_move_rel_frame("base_link", reason));
  ASSERT_TRUE(validate_move_rel_deltas(0.0, 0.0, 0.03, reason));

  auto target = compute_move_rel_target(current, 0.0, 0.0, 0.03);

  EXPECT_DOUBLE_EQ(target.position.z, 0.25);
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
  // Orientation preserved
  EXPECT_DOUBLE_EQ(target.orientation.y, 1.0);
  EXPECT_DOUBLE_EQ(target.orientation.w, 0.0);
}

TEST(MoveRelValidatorTest, FullFlowDeltaPushesTargetOutOfBounds) {
  // Simulate: current near ceiling, move up 0.03 m -> exceeds z_max=0.56
  geometry_msgs::msg::Pose current;
  current.position.x = 0.0;
  current.position.y = 0.0;
  current.position.z = 0.54;  // just below kZMax = 0.56
  current.orientation.w = 1.0;

  std::string reason;
  ASSERT_TRUE(validate_move_rel_deltas(0.0, 0.0, 0.03, reason));

  auto target = compute_move_rel_target(current, 0.0, 0.0, 0.03);

  EXPECT_GT(target.position.z, MoveRelLimits::kZMax);
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

// ── Planner routing ──

} // namespace
} // namespace motion_core
