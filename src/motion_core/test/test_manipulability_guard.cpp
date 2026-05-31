#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "motion_core/manipulability_guard.hpp"

namespace motion_core {
namespace {
trajectory_msgs::msg::JointTrajectory
make_trajectory(const std::vector<std::string> &joint_names,
                const std::vector<std::vector<double>> &points) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joint_names;
  for (const auto &positions : points) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = positions;
    traj.points.push_back(pt);
  }
  return traj;
}

TEST(ManipulabilityGuardTest, ExplicitlyDisabledPassesEverything) {
  ManipulabilityGuard guard = ManipulabilityGuard::disabled();
  auto traj = make_trajectory({"joint_1_s"}, {{0.0}, {1.0}});
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_FALSE(guard.enabled());
}

TEST(ManipulabilityGuardTest, DisabledGuardReturnsZeroIndex) {
  ManipulabilityGuard guard = ManipulabilityGuard::disabled();
  const double w =
      guard.compute_yoshikawa_index({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(w, 0.0);
}

TEST(ManipulabilityGuardTest, NormalizesRawYoshikawaByReferenceLengthCubed) {
  const double raw_home_index = 0.018673921593123997;
  const double reference_length_m = 0.55;

  const double normalized =
      normalize_yoshikawa_index(raw_home_index, reference_length_m);
  const double expected =
      raw_home_index /
      (reference_length_m * reference_length_m * reference_length_m);

  EXPECT_NEAR(normalized, expected, 1e-12);
}

TEST(ManipulabilityGuardTest, NormalizationFailsClosedForInvalidInputs) {
  EXPECT_DOUBLE_EQ(normalize_yoshikawa_index(-0.1, 0.55), 0.0);
  EXPECT_DOUBLE_EQ(normalize_yoshikawa_index(0.02, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(
      normalize_yoshikawa_index(std::numeric_limits<double>::infinity(), 0.55),
      0.0);
  EXPECT_DOUBLE_EQ(
      normalize_yoshikawa_index(0.02, std::numeric_limits<double>::quiet_NaN()),
      0.0);
}

TEST(ManipulabilityGuardTest, EnabledGuardWithoutModelRejectsTrajectory) {
  ManipulabilityGuard guard(nullptr, "gp4_arm", 0.05, 1);
  auto traj = make_trajectory(
      {"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b",
       "joint_6_t"},
      {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.1, 0.0, 0.0, 0.0}});
  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_EQ(reason, "manipulability_guard enabled without robot model");
}

TEST(ManipulabilityGuardTest, SinglePointNoMotionTrajectoryPasses) {
  ManipulabilityGuard guard(nullptr, "gp4_arm", 0.05, 1);
  auto traj = make_trajectory({"joint_1_s", "joint_2_l", "joint_3_u",
                               "joint_4_r", "joint_5_b", "joint_6_t"},
                              {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(ManipulabilityGuardTest,
     RecoveryFromLowManipulabilityAllowsLocalDipAboveStart) {
  std::string reason;
  const std::vector<ManipulabilitySample> samples = {
      {0, 0.0187}, {10, 0.0206}, {25, 0.0190}, {40, 0.0520}};

  EXPECT_TRUE(check_manipulability_samples(0.05, samples, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(ManipulabilityGuardTest,
     RecoveryFromLowManipulabilityRejectsDipBelowStart) {
  std::string reason;
  const std::vector<ManipulabilitySample> samples = {
      {0, 0.0187}, {10, 0.0206}, {25, 0.0180}, {40, 0.0520}};

  EXPECT_FALSE(check_manipulability_samples(0.05, samples, reason));
  EXPECT_NE(reason.find("below recovery start"), std::string::npos);
}

TEST(ManipulabilityGuardTest,
     RecoveryFromLowManipulabilityRejectsFinalBelowFloor) {
  std::string reason;
  const std::vector<ManipulabilitySample> samples = {
      {0, 0.0187}, {10, 0.0206}, {25, 0.0190}};

  EXPECT_FALSE(check_manipulability_samples(0.05, samples, reason));
  EXPECT_NE(reason.find("recovery failed"), std::string::npos);
}

TEST(ManipulabilityGuardTest, NormalTrajectoryRejectsSampleBelowFloor) {
  std::string reason;
  const std::vector<ManipulabilitySample> samples = {{0, 0.0700}, {5, 0.0490}};

  EXPECT_FALSE(check_manipulability_samples(0.05, samples, reason));
  EXPECT_NE(reason.find("reject at point[5]"), std::string::npos);
}

} // namespace
} // namespace motion_core
