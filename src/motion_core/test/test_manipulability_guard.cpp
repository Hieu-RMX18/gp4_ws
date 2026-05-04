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

TEST(ManipulabilityGuardTest, DefaultConstructedPassesEverything) {
  ManipulabilityGuard guard;
  auto traj = make_trajectory({"joint_1_s"}, {{0.0}, {1.0}});
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_FALSE(guard.enabled());
}

TEST(ManipulabilityGuardTest, NullModelReturnsHighIndex) {
  ManipulabilityGuard guard;
  const double w =
      guard.compute_yoshikawa_index({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  EXPECT_GE(w, 1.0);
}

TEST(ManipulabilityGuardTest, NullModelPassesAnyTrajectory) {
  ManipulabilityGuard guard(nullptr, "gp4_arm", 0.05, 1);
  auto traj = make_trajectory({"joint_1_s", "joint_2_l", "joint_3_u",
                               "joint_4_r", "joint_5_b", "joint_6_t"},
                              {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
}

} // namespace
} // namespace motion_core
