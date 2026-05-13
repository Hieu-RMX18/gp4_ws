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
  // norm = 0.05 exactly — should pass.
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.05, 0.0, 0.0, reason));
}

TEST(MoveRelValidatorTest, RejectsDeltaNormExceedsLimit) {
  // norm = sqrt(0.04^2 + 0.04^2) ≈ 0.0566 > 0.05 (reject)
  std::string reason;
  EXPECT_FALSE(validate_move_rel_deltas(0.04, 0.04, 0.0, reason));
  EXPECT_NE(reason.find("exceeds safety limit"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsLargeSingleAxisDelta) {
  // 0.06 > 0.05 limit
  std::string reason;
  EXPECT_FALSE(validate_move_rel_deltas(0.0, 0.0, 0.06, reason));
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
  target.position.z = 0.30;

  std::string reason;
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
}

TEST(MoveRelValidatorTest, AcceptsTargetAtBoundaryEdge) {
  geometry_msgs::msg::Pose target;
  target.position.x = MoveRelLimits::kXMax;
  target.position.y = MoveRelLimits::kYMin;
  target.position.z = MoveRelLimits::kZMax;

  std::string reason;
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
}

TEST(MoveRelValidatorTest, AcceptsTargetAtXMinBoundary) {
  geometry_msgs::msg::Pose target;
  target.position.x = MoveRelLimits::kXMin;
  target.position.y = 0.0;
  target.position.z = 0.35;

  std::string reason;
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
}

TEST(MoveRelValidatorTest, RejectsTargetBelowZMin) {
  geometry_msgs::msg::Pose target;
  target.position.x = 0.0;
  target.position.y = 0.0;
  target.position.z = 0.22; // below kZMin = 0.23

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetAboveXMax) {
  geometry_msgs::msg::Pose target;
  target.position.x = 0.46; // above kXMax = 0.45
  target.position.y = 0.0;
  target.position.z = 0.40;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetBelowYMin) {
  geometry_msgs::msg::Pose target;
  target.position.x = 0.0;
  target.position.y = -0.18; // below kYMin = -0.16
  target.position.z = 0.40;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetBelowZMinBeforeFloorClearanceGuard) {
  // floor_clearance_guard Z=[0.0, 0.20] is entirely below workspace z_min=0.23.
  // This test verifies the point is rejected by workspace bounds.
  geometry_msgs::msg::Pose target;
  target.position.x = 0.10;
  target.position.y = 0.05;
  target.position.z = 0.12;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetBelowXMin) {
  geometry_msgs::msg::Pose target;
  target.position.x = MoveRelLimits::kXMin - 0.001;
  target.position.y = 0.30;
  target.position.z = 0.35;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetBeyondFrontWall) {
  // Position beyond front wall (y < kYMin = -0.16) is rejected by workspace
  // bounds.
  geometry_msgs::msg::Pose target;
  target.position.x = 0.0;
  target.position.y = -0.197; // station front wall, below kYMin
  target.position.z = 0.35;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetBeyondRightWall) {
  // Calibrated side wall is at x=-0.482; this is below kXMin=-0.45 and must be
  // rejected.
  geometry_msgs::msg::Pose target;
  target.position.x = -0.482;
  target.position.y = 0.30;
  target.position.z = 0.40;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetNearRightWallEdge) {
  // x=-0.46 is just outside kXMin=-0.45 and must be rejected.
  geometry_msgs::msg::Pose target;
  target.position.x = -0.46;
  target.position.y = 0.30;
  target.position.z = 0.35;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, ForbiddenZoneCentersAreRejectedByWorkspaceFirst) {
  struct Case {
    const char *name;
    double x;
    double y;
    double z;
  };

  const Case cases[] = {
      {"front_wall_guard", MoveRelLimits::kFrontWallX,
       MoveRelLimits::kFrontWallY, 0.35},
      {"right_wall_guard", MoveRelLimits::kRightWallX,
       MoveRelLimits::kRightWallY, 0.35},
      {"floor_clearance_guard", MoveRelLimits::kFloorClearanceX,
       MoveRelLimits::kFloorClearanceY, MoveRelLimits::kFloorClearanceZ},
  };

  for (const auto &test_case : cases) {
    geometry_msgs::msg::Pose target;
    target.position.x = test_case.x;
    target.position.y = test_case.y;
    target.position.z = test_case.z;

    std::string reason;
    EXPECT_FALSE(validate_move_rel_target_bounds(target, reason))
        << "case: " << test_case.name;
    EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos)
        << "case: " << test_case.name;
    EXPECT_EQ(reason.find("intersects forbidden zone"), std::string::npos)
        << "case: " << test_case.name;
  }
}

// ── Integration: full resolve-then-validate flow ──

TEST(MoveRelValidatorTest, FullFlowValidDeltaInsideBounds) {
  // Simulate: current in free space, move up 2cm -> stays in bounds
  geometry_msgs::msg::Pose current;
  current.position.x = 0.10;
  current.position.y = -0.10;
  current.position.z = 0.30;
  current.orientation.x = 0.0;
  current.orientation.y = 1.0;
  current.orientation.z = 0.0;
  current.orientation.w = 0.0;

  std::string reason;
  ASSERT_TRUE(validate_move_rel_frame("base_link", reason));
  ASSERT_TRUE(validate_move_rel_deltas(0.0, 0.0, 0.02, reason));

  auto target = compute_move_rel_target(current, 0.0, 0.0, 0.02);

  EXPECT_DOUBLE_EQ(target.position.z, 0.32);
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
  // Orientation preserved
  EXPECT_DOUBLE_EQ(target.orientation.y, 1.0);
  EXPECT_DOUBLE_EQ(target.orientation.w, 0.0);
}

TEST(MoveRelValidatorTest, FullFlowDeltaPushesTargetOutOfBounds) {
  // Simulate: current near ceiling, move up 0.03 m -> exceeds z_max=0.65
  geometry_msgs::msg::Pose current;
  current.position.x = 0.0;
  current.position.y = 0.0;
  current.position.z = 0.63; // just below kZMax = 0.65
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
