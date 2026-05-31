// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <vector>

#include <gtest/gtest.h>

#include "motion_core/trajectory_assembler.hpp"

namespace motion_core {
namespace {

using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

JointTrajectory make_segment(const std::vector<std::string> &joint_names,
                             const std::vector<double> &durations_sec,
                             double position_value) {
  JointTrajectory traj;
  traj.joint_names = joint_names;
  double accumulated = 0.0;
  for (double dt : durations_sec) {
    accumulated += dt;
    JointTrajectoryPoint pt;
    pt.positions.assign(joint_names.size(), position_value);
    pt.time_from_start.sec = static_cast<int32_t>(accumulated);
    pt.time_from_start.nanosec =
        static_cast<uint32_t>((accumulated - static_cast<int32_t>(accumulated)) *
                              1'000'000'000);
    traj.points.push_back(std::move(pt));
  }
  return traj;
}

TEST(TrajectoryAssemblerTest, RejectsEmptySegments) {
  auto result = TrajectoryAssembler::merge({});
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("no segments"), std::string::npos);
}

TEST(TrajectoryAssemblerTest, RejectsJointNamesMismatch) {
  auto seg1 = make_segment({"j1", "j2"}, {1.0}, 0.0);
  auto seg2 = make_segment({"j1", "j3"}, {1.0}, 0.1);

  auto result = TrajectoryAssembler::merge({seg1, seg2});
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("joint_names mismatch"), std::string::npos);
}

TEST(TrajectoryAssemblerTest, RejectsEmptySegment) {
  auto seg1 = make_segment({"j1"}, {1.0}, 0.0);
  JointTrajectory empty_seg;
  empty_seg.joint_names = {"j1"};

  auto result = TrajectoryAssembler::merge({seg1, empty_seg});
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("is empty"), std::string::npos);
}

TEST(TrajectoryAssemblerTest, MergesTwoSegments) {
  // Identical positions -> first point of seg2 dropped, times accumulate
  auto seg1 = make_segment({"j1", "j2"}, {0.0, 1.0}, 0.0);
  auto seg2 = make_segment({"j1", "j2"}, {0.0, 1.0}, 0.0);

  auto result = TrajectoryAssembler::merge({seg1, seg2});
  EXPECT_TRUE(result.success);
  ASSERT_EQ(result.trajectory.points.size(), 3U);

  // Point 0: t=0
  EXPECT_EQ(result.trajectory.points[0].time_from_start.sec, 0);
  // Point 1: t=1 (seg1 end)
  EXPECT_EQ(result.trajectory.points[1].time_from_start.sec, 1);
  // Point 2: t=2 (seg2 end, accumulated 1 + 1)
  EXPECT_EQ(result.trajectory.points[2].time_from_start.sec, 2);
}

TEST(TrajectoryAssemblerTest, DropsDuplicateFirstPoint) {
  auto seg1 = make_segment({"j1"}, {0.0, 1.0}, 0.5);
  auto seg2 = make_segment({"j1"}, {0.0, 1.0}, 0.5);

  auto result = TrajectoryAssembler::merge({seg1, seg2});
  EXPECT_TRUE(result.success);
  // seg1 has 2 points [0.0, 1.0], seg2 has 2 points [0.0, 1.0]
  // first point of seg2 (0.5 at t=0 relative) is duplicate of last point of seg1
  // so total = 2 + 1 = 3
  ASSERT_EQ(result.trajectory.points.size(), 3U);
}

TEST(TrajectoryAssemblerTest, KeepsNonDuplicateFirstPoint) {
  auto seg1 = make_segment({"j1"}, {0.0, 1.0}, 0.5);
  auto seg2 = make_segment({"j1"}, {0.0, 1.0}, 0.6);

  auto result = TrajectoryAssembler::merge({seg1, seg2});
  EXPECT_TRUE(result.success);
  // positions differ, so all 4 points kept
  ASSERT_EQ(result.trajectory.points.size(), 4U);
}

TEST(TrajectoryAssemblerTest, RejectsBudgetExceeded) {
  std::vector<JointTrajectory> segments;
  segments.reserve(2);
  std::vector<double> durations;
  durations.reserve(100);
  for (int i = 0; i < 100; ++i) {
    durations.push_back(0.01);
  }
  segments.push_back(make_segment({"j1"}, durations, 0.0));
  segments.push_back(make_segment({"j1"}, durations, 0.1));

  auto result = TrajectoryAssembler::merge(segments);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("exceeds budget"), std::string::npos);
  EXPECT_TRUE(result.trajectory.points.empty());
}

TEST(TrajectoryAssemblerTest, PreservesVelocityAndAcceleration) {
  JointTrajectory seg;
  seg.joint_names = {"j1"};
  JointTrajectoryPoint pt;
  pt.positions = {0.0};
  pt.velocities = {1.0};
  pt.accelerations = {2.0};
  pt.time_from_start.sec = 0;
  seg.points.push_back(pt);

  auto result = TrajectoryAssembler::merge({seg});
  EXPECT_TRUE(result.success);
  ASSERT_EQ(result.trajectory.points.size(), 1U);
  EXPECT_EQ(result.trajectory.points[0].velocities.size(), 1U);
  EXPECT_EQ(result.trajectory.points[0].velocities[0], 1.0);
  EXPECT_EQ(result.trajectory.points[0].accelerations.size(), 1U);
  EXPECT_EQ(result.trajectory.points[0].accelerations[0], 2.0);
}

} // namespace
} // namespace motion_core
