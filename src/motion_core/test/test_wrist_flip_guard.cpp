#include <string>

#include <gtest/gtest.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "motion_core/wrist_flip_guard.hpp"

namespace motion_core {
namespace {
trajectory_msgs::msg::JointTrajectory
make_two_point_trajectory(double delta_joint_1) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint_1_s", "joint_2_l"};

  trajectory_msgs::msg::JointTrajectoryPoint point0;
  point0.positions = {0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint point1;
  point1.positions = {delta_joint_1, 0.1};

  traj.points.push_back(point0);
  traj.points.push_back(point1);
  return traj;
}

TEST(WristFlipGuardTest, PassesWhenConsecutiveDeltasAreWithinLimit) {
  const WristFlipGuard guard;
  const auto traj = make_two_point_trajectory(0.4);

  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(WristFlipGuardTest, RejectsWhenAnyJointDeltaExceedsThirtyDegrees) {
  const WristFlipGuard guard;
  const auto traj = make_two_point_trajectory(0.7);

  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_FALSE(reason.empty());
}
trajectory_msgs::msg::JointTrajectory
make_trajectory_6dof(const std::vector<std::vector<double>> &points) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint_1_s", "joint_2_l", "joint_3_u",
                      "joint_4_r", "joint_5_b", "joint_6_t"};
  for (const auto &positions : points) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = positions;
    traj.points.push_back(pt);
  }
  return traj;
}

TEST(CumulativeRotationGuardTest, PassesWhenBelowLimit) {
  WristFlipGuard guard({{"joint_5_b", 4.189}});
  auto traj = make_trajectory_6dof({
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.1, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.2, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.3, 0.0},
  });
  std::string reason;
  EXPECT_TRUE(guard.check_cumulative_rotation(traj, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(CumulativeRotationGuardTest, RejectsWhenCumulativeExceedsMax) {
  WristFlipGuard guard({{"joint_5_b", 0.5}});
  auto traj = make_trajectory_6dof({
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.3, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  });
  std::string reason;
  EXPECT_FALSE(guard.check_cumulative_rotation(traj, reason));
  EXPECT_NE(reason.find("cumulative_rotation_guard reject"), std::string::npos);
  EXPECT_NE(reason.find("joint_5_b"), std::string::npos);
}

TEST(CumulativeRotationGuardTest, DefaultConstructedPassesEverything) {
  WristFlipGuard guard;
  auto traj = make_trajectory_6dof({
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 99.0, 0.0},
  });
  std::string reason;
  EXPECT_TRUE(guard.check_cumulative_rotation(traj, reason));
}

TEST(CumulativeRotationGuardTest, UnconfiguredJointsAreIgnored) {
  WristFlipGuard guard({{"joint_4_r", 6.283}});
  auto traj = make_trajectory_6dof({
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 99.0, 0.0},
  });
  std::string reason;
  EXPECT_TRUE(guard.check_cumulative_rotation(traj, reason));
}

} // namespace
} // namespace motion_core
