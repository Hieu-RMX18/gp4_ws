#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/revolute_joint_model.h>

#include "motion_core/angle_branch_utils.hpp"

namespace motion_core {
namespace {
moveit::core::VariableBounds make_bounds(const double min_position,
                                         const double max_position) {
  moveit::core::VariableBounds bounds;
  bounds.position_bounded_ = true;
  bounds.min_position_ = min_position;
  bounds.max_position_ = max_position;
  return bounds;
}
} // namespace

TEST(AngleBranchUtilsTest, UsesLargeLimitHelperForWideJointBounds) {
  const auto result = choose_branch_preserved_angle(3.13, -3.13, -7.94, 7.94);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.helper_used, "shortest_angular_distance_with_large_limits");
  EXPECT_NEAR(result.chosen_target, 3.153185307179586, 1e-6);
  EXPECT_NEAR(result.delta_from_current, 0.0231853071795864, 1e-6);
}

TEST(AngleBranchUtilsTest, RejectsWhenCurrentStateIsOutsideBounds) {
  const auto result = choose_branch_preserved_angle(0.75, 0.10, -0.5, 0.5);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.reason.empty());
}

TEST(AngleBranchUtilsTest, PreservesNearestEquivalentAcrossTwoPiOffset) {
  const auto result =
      choose_branch_preserved_angle(0.10, (2.0 * M_PI) + 0.12, -7.94, 7.94);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.chosen_target, 0.12, 1e-6);
  EXPECT_NEAR(result.delta_from_current, 0.02, 1e-6);
}

TEST(AngleBranchUtilsTest, BuildsBranchPreservedJointVectorForRevoluteJoints) {
  moveit::core::RevoluteJointModel joint_1("joint_1_s", 0U, 0U);
  moveit::core::RevoluteJointModel joint_6("joint_6_t", 1U, 1U);
  joint_1.setVariableBounds(
      "joint_1_s", make_bounds(-2.9670597283903604, 2.9670597283903604));
  joint_6.setVariableBounds("joint_6_t",
                            make_bounds(-7.941248096574199, 7.941248096574199));

  const std::vector<const moveit::core::JointModel *> joint_models = {&joint_1,
                                                                      &joint_6};
  const std::vector<double> current = {0.10, 3.13};
  const std::vector<double> requested = {0.12, -3.13};

  const auto result =
      choose_branch_preserved_joint_vector(joint_models, current, requested);

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.chosen_targets.size(), 2U);
  ASSERT_EQ(result.deltas_from_current.size(), 2U);
  EXPECT_NEAR(result.chosen_targets[0], 0.12, 1e-6);
  EXPECT_NEAR(result.chosen_targets[1], 3.153185307179586, 1e-6);
  EXPECT_EQ(result.helper_used[0], "shortest_angular_distance_with_limits");
  EXPECT_EQ(result.helper_used[1],
            "shortest_angular_distance_with_large_limits");
}
} // namespace motion_core
