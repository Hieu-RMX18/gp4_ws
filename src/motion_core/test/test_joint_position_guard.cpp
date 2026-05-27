#include <string>

#include <gtest/gtest.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "motion_core/joint_position_guard.hpp"

namespace motion_core {
namespace {
std::unordered_map<std::string, JointLimit> make_operational_limits() {
  return {
      {"joint_1_s", {-2.967, 2.967}}, {"joint_2_l", {-1.920, 2.269}},
      {"joint_3_u", {-1.134, 3.491}}, {"joint_4_r", {-2.443, 2.443}},
      {"joint_5_b", {-1.80, 1.80}}, {"joint_6_t", {-3.142, 3.142}},
  };
}

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

TEST(JointPositionGuardTest, PassesEmptyTrajectory) {
  JointPositionGuard guard(make_operational_limits());
  trajectory_msgs::msg::JointTrajectory traj;
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(JointPositionGuardTest, PassesTrajectoryWithinLimits) {
  JointPositionGuard guard(make_operational_limits());
  auto traj = make_trajectory({"joint_1_s", "joint_2_l", "joint_3_u",
                               "joint_4_r", "joint_5_b", "joint_6_t"},
                              {
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {1.0, 0.5, 1.0, 1.0, 1.0, 1.0},
                              });
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(JointPositionGuardTest, PassesJointNotInLimitsMap) {
  JointPositionGuard guard(make_operational_limits());
  auto traj = make_trajectory({"station_axis_1"}, {{99.0}});
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
}

TEST(JointPositionGuardTest, RejectsJoint4RAboveLimit) {
  JointPositionGuard guard(make_operational_limits());
  auto traj = make_trajectory({"joint_1_s", "joint_2_l", "joint_3_u",
                               "joint_4_r", "joint_5_b", "joint_6_t"},
                              {
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 2.460, 0.0, 0.0},
                              });
  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_NE(reason.find("joint_4_r"), std::string::npos);
  EXPECT_NE(reason.find("point[1]"), std::string::npos);
  EXPECT_NE(reason.find("2.4600"), std::string::npos);
}

TEST(JointPositionGuardTest, RejectsJoint5BAtPoint3) {
  JointPositionGuard guard(make_operational_limits());
  auto traj = make_trajectory({"joint_5_b"}, {{0.0}, {-0.5}, {-1.0}, {-1.815}});
  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_NE(reason.find("joint_5_b"), std::string::npos);
  EXPECT_NE(reason.find("point[3]"), std::string::npos);
}

TEST(JointPositionGuardTest, RejectsJoint6TFarOutside) {
  JointPositionGuard guard(make_operational_limits());
  auto traj = make_trajectory({"joint_6_t"}, {{5.0}});
  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_NE(reason.find("joint_6_t"), std::string::npos);
  EXPECT_NE(reason.find("5.0000"), std::string::npos);
}

TEST(JointPositionGuardTest, ReasonMessageMatchesFormat) {
  JointPositionGuard guard(make_operational_limits());
  auto traj = make_trajectory({"joint_5_b"}, {{-1.815}});
  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_NE(reason.find("joint_position_guard reject at point[0]"),
            std::string::npos);
  EXPECT_NE(reason.find("joint_5_b"), std::string::npos);
  EXPECT_NE(reason.find("outside"), std::string::npos);
}

TEST(JointPositionGuardTest, HasLimitAndGetLimit) {
  JointPositionGuard guard(make_operational_limits());
  EXPECT_TRUE(guard.has_limit("joint_5_b"));
  EXPECT_FALSE(guard.has_limit("nonexistent_joint"));
  const auto lim = guard.get_limit("joint_5_b");
  EXPECT_DOUBLE_EQ(lim.min, -1.80);
  EXPECT_DOUBLE_EQ(lim.max, 1.80);
}

TEST(JointPositionGuardTest, DefaultConstructedPassesEverything) {
  JointPositionGuard guard;
  auto traj = make_trajectory({"joint_5_b"}, {{-99.0}});
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
}

TEST(JointPositionGuardTest, ModeDefaultRejectsExtendedRange) {
  auto limits = make_operational_limits();
  JointPositionGuard guard(limits);
  auto traj = make_trajectory({"joint_6_t"}, {{5.0}});
  std::string reason;
  EXPECT_FALSE(
      guard.check_trajectory(traj, reason, JointPositionGuard::Mode::Default));
  EXPECT_NE(reason.find("mode=default"), std::string::npos);
}

TEST(JointPositionGuardTest, ModeExtendedAcceptsExtendedRange) {
  std::unordered_map<std::string, TieredLimit> tiered;
  tiered["joint_6_t"] = TieredLimit{{-3.142, 3.142}, JointLimit{-7.941, 7.941}};
  JointPositionGuard guard(std::move(tiered));
  auto traj = make_trajectory({"joint_6_t"}, {{5.0}});
  std::string reason;
  EXPECT_TRUE(
      guard.check_trajectory(traj, reason, JointPositionGuard::Mode::Extended));
}

TEST(JointPositionGuardTest, ModeExtendedRejectsBeyondHardwareCap) {
  std::unordered_map<std::string, TieredLimit> tiered;
  tiered["joint_6_t"] = TieredLimit{{-3.142, 3.142}, JointLimit{-7.941, 7.941}};
  JointPositionGuard guard(std::move(tiered));
  auto traj = make_trajectory({"joint_6_t"}, {{8.0}});
  std::string reason;
  EXPECT_FALSE(
      guard.check_trajectory(traj, reason, JointPositionGuard::Mode::Extended));
  EXPECT_NE(reason.find("8.0000"), std::string::npos);
  EXPECT_NE(reason.find("mode=extended"), std::string::npos);
}

TEST(JointPositionGuardTest, ModeExtendedFallsBackToDefaultForNonTiered) {
  auto limits = make_operational_limits();
  JointPositionGuard guard(limits);
  auto traj = make_trajectory({"joint_6_t"}, {{5.0}});
  std::string reason;
  EXPECT_FALSE(
      guard.check_trajectory(traj, reason, JointPositionGuard::Mode::Extended));
  EXPECT_NE(reason.find("mode=extended"), std::string::npos);
}

TEST(JointPositionGuardTest, ToleranceAllowsSlightOvershoot) {
  JointPositionGuard guard(make_operational_limits(), 0.005);
  auto traj = make_trajectory({"joint_5_b"}, {{-1.8037}});
  std::string reason;
  EXPECT_TRUE(guard.check_trajectory(traj, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(JointPositionGuardTest, ToleranceStillRejectsBeyondMargin) {
  JointPositionGuard guard(make_operational_limits(), 0.005);
  auto traj = make_trajectory({"joint_5_b"}, {{-1.810}});
  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_NE(reason.find("joint_5_b"), std::string::npos);
}

TEST(JointPositionGuardTest, ZeroToleranceRejectsExactOvershoot) {
  JointPositionGuard guard(make_operational_limits(), 0.0);
  auto traj = make_trajectory({"joint_5_b"}, {{-1.8001}});
  std::string reason;
  EXPECT_FALSE(guard.check_trajectory(traj, reason));
  EXPECT_NE(reason.find("joint_5_b"), std::string::npos);
}

TEST(JointPositionGuardTest, DefaultToleranceMatchesConstant) {
  JointPositionGuard guard(make_operational_limits());
  EXPECT_DOUBLE_EQ(guard.tolerance_rad(),
                   JointPositionGuard::kDefaultToleranceRad);
}

} // namespace
} // namespace motion_core
