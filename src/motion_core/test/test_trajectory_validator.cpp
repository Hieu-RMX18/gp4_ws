#include "motion_core/trajectory_validator.hpp"

#include <gtest/gtest.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace {
trajectory_msgs::msg::JointTrajectory make_valid_two_point_trajectory() {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint_1_s", "joint_2_l"};

  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = {0.0, 0.0};
  first.time_from_start.sec = 0;
  traj.points.push_back(first);

  trajectory_msgs::msg::JointTrajectoryPoint second;
  second.positions = {0.1, 0.2};
  second.time_from_start.sec = 1;
  traj.points.push_back(second);

  return traj;
}
} // namespace

TEST(TrajectoryValidator, RejectsSinglePointTrajectoryBeforeDispatch) {
  auto traj = make_valid_two_point_trajectory();
  traj.points.resize(1);
  traj.points.front().time_from_start.sec = 1;

  std::string reason;

  EXPECT_FALSE(motion_core::validate_trajectory_structure(traj, reason));
  EXPECT_EQ(reason, "trajectory must contain at least two points");
}

TEST(TrajectoryValidator, AcceptsTwoPointTrajectory) {
  const auto traj = make_valid_two_point_trajectory();

  std::string reason;

  EXPECT_TRUE(motion_core::validate_trajectory_structure(traj, reason));
}

TEST(TrajectoryValidator, AcceptsSinglePointNoopWhenCurrentJointsMatch) {
  auto traj = make_valid_two_point_trajectory();
  traj.points.resize(1);
  traj.points.front().time_from_start.sec = 1;

  std::string reason;

  EXPECT_TRUE(motion_core::is_single_point_noop_trajectory(
      traj, {0.0, 0.0}, 1e-6, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(TrajectoryValidator, RejectsSinglePointNoopWhenCurrentJointsDiffer) {
  auto traj = make_valid_two_point_trajectory();
  traj.points.resize(1);
  traj.points.front().time_from_start.sec = 1;

  std::string reason;

  EXPECT_FALSE(motion_core::is_single_point_noop_trajectory(
      traj, {0.5, 0.0}, 1e-6, reason));
  EXPECT_EQ(reason, "single-point trajectory does not match current joint state");
}
