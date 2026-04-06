#include <gtest/gtest.h>

#include "motion_core/move_rel_validator.hpp"

namespace motion_core
{
namespace
{

// ── validate_move_rel_frame ──

TEST(MoveRelValidatorTest, AcceptsEmptyFrame)
{
  std::string reason;
  EXPECT_TRUE(validate_move_rel_frame("", reason));
}

TEST(MoveRelValidatorTest, AcceptsBaseLinkFrame)
{
  std::string reason;
  EXPECT_TRUE(validate_move_rel_frame("base_link", reason));
}

TEST(MoveRelValidatorTest, RejectsUnsupportedFrame)
{
  std::string reason;
  EXPECT_FALSE(validate_move_rel_frame("tool0", reason));
  EXPECT_NE(reason.find("unsupported reference_frame"), std::string::npos);
  EXPECT_NE(reason.find("tool0"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsWorldFrame)
{
  std::string reason;
  EXPECT_FALSE(validate_move_rel_frame("world", reason));
}

// ── validate_move_rel_deltas ──

TEST(MoveRelValidatorTest, RejectsAllZeroDeltas)
{
  std::string reason;
  EXPECT_FALSE(validate_move_rel_deltas(0.0, 0.0, 0.0, reason));
  EXPECT_NE(reason.find("all delta components are zero"), std::string::npos);
}

TEST(MoveRelValidatorTest, AcceptsSingleAxisDelta)
{
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.0, 0.0, 0.10, reason));
}

TEST(MoveRelValidatorTest, AcceptsMultiAxisDelta)
{
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.05, 0.0, 0.02, reason));
}

TEST(MoveRelValidatorTest, AcceptsNegativeDelta)
{
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.0, 0.0, -0.10, reason));
}

TEST(MoveRelValidatorTest, AcceptsDeltaExactlyAtLimit)
{
  // norm = 0.20 exactly — should pass
  std::string reason;
  EXPECT_TRUE(validate_move_rel_deltas(0.20, 0.0, 0.0, reason));
}

TEST(MoveRelValidatorTest, RejectsDeltaNormExceedsLimit)
{
  // norm = sqrt(0.15^2 + 0.15^2) ≈ 0.2121 > 0.20
  std::string reason;
  EXPECT_FALSE(validate_move_rel_deltas(0.15, 0.15, 0.0, reason));
  EXPECT_NE(reason.find("exceeds safety limit"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsLargeSingleAxisDelta)
{
  std::string reason;
  EXPECT_FALSE(validate_move_rel_deltas(0.0, 0.0, 0.25, reason));
  EXPECT_NE(reason.find("exceeds safety limit"), std::string::npos);
}

// ── compute_move_rel_target ──

TEST(MoveRelValidatorTest, ComputeTargetAddsDeltas)
{
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

TEST(MoveRelValidatorTest, ComputeTargetPreservesOrientation)
{
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

TEST(MoveRelValidatorTest, ComputeTargetWithZeroDeltaEqualsCurrentPosition)
{
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

TEST(MoveRelValidatorTest, AcceptsTargetInsideBounds)
{
  geometry_msgs::msg::Pose target;
  target.position.x = 0.3;
  target.position.y = 0.1;
  target.position.z = 0.4;

  std::string reason;
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
}

TEST(MoveRelValidatorTest, AcceptsTargetAtBoundaryEdge)
{
  geometry_msgs::msg::Pose target;
  target.position.x = MoveRelLimits::kXMax;
  target.position.y = MoveRelLimits::kYMax;
  target.position.z = MoveRelLimits::kZMax;

  std::string reason;
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
}

TEST(MoveRelValidatorTest, RejectsTargetBelowZMin)
{
  geometry_msgs::msg::Pose target;
  target.position.x = 0.0;
  target.position.y = 0.0;
  target.position.z = 0.01;  // below kZMin = 0.02

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetAboveXMax)
{
  geometry_msgs::msg::Pose target;
  target.position.x = 0.81;  // above kXMax = 0.8
  target.position.y = 0.0;
  target.position.z = 0.5;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

TEST(MoveRelValidatorTest, RejectsTargetBelowYMin)
{
  geometry_msgs::msg::Pose target;
  target.position.x = 0.0;
  target.position.y = -0.81;  // below kYMin = -0.8
  target.position.z = 0.5;

  std::string reason;
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
}

// ── Integration: full resolve-then-validate flow ──

TEST(MoveRelValidatorTest, FullFlowValidDeltaInsideBounds)
{
  // Simulate: current at center, move up 10cm → stays in bounds
  geometry_msgs::msg::Pose current;
  current.position.x = 0.3;
  current.position.y = 0.0;
  current.position.z = 0.4;
  current.orientation.x = 0.0;
  current.orientation.y = 1.0;
  current.orientation.z = 0.0;
  current.orientation.w = 0.0;

  std::string reason;
  ASSERT_TRUE(validate_move_rel_frame("base_link", reason));
  ASSERT_TRUE(validate_move_rel_deltas(0.0, 0.0, 0.10, reason));

  auto target = compute_move_rel_target(current, 0.0, 0.0, 0.10);

  EXPECT_DOUBLE_EQ(target.position.z, 0.5);
  EXPECT_TRUE(validate_move_rel_target_bounds(target, reason));
  // Orientation preserved
  EXPECT_DOUBLE_EQ(target.orientation.y, 1.0);
  EXPECT_DOUBLE_EQ(target.orientation.w, 0.0);
}

TEST(MoveRelValidatorTest, FullFlowDeltaPushesTargetOutOfBounds)
{
  // Simulate: current near ceiling, move up 10cm → exceeds z_max
  geometry_msgs::msg::Pose current;
  current.position.x = 0.0;
  current.position.y = 0.0;
  current.position.z = 1.15;  // near kZMax = 1.2
  current.orientation.w = 1.0;

  std::string reason;
  ASSERT_TRUE(validate_move_rel_deltas(0.0, 0.0, 0.10, reason));

  auto target = compute_move_rel_target(current, 0.0, 0.0, 0.10);

  EXPECT_GT(target.position.z, MoveRelLimits::kZMax);
  EXPECT_FALSE(validate_move_rel_target_bounds(target, reason));
  EXPECT_NE(reason.find("outside workspace bounds"), std::string::npos);
}

// ── Planner routing ──

}  // namespace
}  // namespace motion_core
