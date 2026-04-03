// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "hw_adapter/trajectory_executor.hpp"

namespace
{
using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFjt = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

trajectory_msgs::msg::JointTrajectory make_trajectory(const std::size_t point_count = 2U)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {
    "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

  for (std::size_t index = 0; index < point_count; ++index)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, static_cast<double>(index) * 0.1};
    point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(index) * 0.1);
    traj.points.push_back(point);
  }

  return traj;
}

class ExecutorThread
{
public:
  explicit ExecutorThread(rclcpp::executors::SingleThreadedExecutor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorThread()
  {
    executor_.cancel();
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor & executor_;
  std::thread thread_;
};

class FollowJointTrajectoryServerHarness
{
public:
  enum class Behavior
  {
    kSucceed,
    kTimeout
  };

  FollowJointTrajectoryServerHarness(const std::string & node_name, const std::string & action_name, Behavior behavior)
  : node_(std::make_shared<rclcpp::Node>(node_name)),
    behavior_(behavior)
  {
    server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      node_,
      action_name,
      std::bind(&FollowJointTrajectoryServerHarness::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FollowJointTrajectoryServerHarness::handle_cancel, this, std::placeholders::_1),
      std::bind(&FollowJointTrajectoryServerHarness::handle_accepted, this, std::placeholders::_1));
  }

  ~FollowJointTrajectoryServerHarness()
  {
    for (auto & worker : workers_)
    {
      if (worker.joinable())
      {
        worker.join();
      }
    }
  }

  rclcpp::Node::SharedPtr node() const
  {
    return node_;
  }

  int goal_count() const
  {
    return goal_count_.load();
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal>)
  {
    ++goal_count_;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFjt>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFjt> goal_handle)
  {
    workers_.emplace_back([this, goal_handle]() {
      if (behavior_ == Behavior::kTimeout)
      {
        std::this_thread::sleep_for(250ms);
      }

      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "";
      goal_handle->succeed(result);
    });
  }

  rclcpp::Node::SharedPtr node_;
  Behavior behavior_;
  std::atomic<int> goal_count_{0};
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
  std::vector<std::thread> workers_;
};

class TrajectoryExecutorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }
};
}  // namespace

TEST_F(TrajectoryExecutorTest, successful_goal_send_with_mock_action_server)
{
  const std::string action_name = "/test_hw_adapter/follow_joint_trajectory_success";
  FollowJointTrajectoryServerHarness server(
    "trajectory_executor_success_server",
    action_name,
    FollowJointTrajectoryServerHarness::Behavior::kSucceed);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server.node());
  ExecutorThread spin_thread(executor);

  auto client_node = std::make_shared<rclcpp::Node>("trajectory_executor_success_client");
  hw_adapter::TrajectoryExecutor executor_client(
    *client_node,
    action_name,
    [](std::string & reason) {reason.clear(); return true;},
    [](std::string & reason) {reason.clear(); return true;},
    500ms);

  std::string reason;
  EXPECT_TRUE(executor_client.execute(make_trajectory(), reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_EQ(server.goal_count(), 1);
}

TEST_F(TrajectoryExecutorTest, timeout_case)
{
  const std::string action_name = "/test_hw_adapter/follow_joint_trajectory_timeout";
  FollowJointTrajectoryServerHarness server(
    "trajectory_executor_timeout_server",
    action_name,
    FollowJointTrajectoryServerHarness::Behavior::kTimeout);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server.node());
  ExecutorThread spin_thread(executor);

  auto client_node = std::make_shared<rclcpp::Node>("trajectory_executor_timeout_client");
  hw_adapter::TrajectoryExecutor executor_client(
    *client_node,
    action_name,
    [](std::string & reason) {reason.clear(); return true;},
    [](std::string & reason) {reason.clear(); return true;},
    50ms);

  std::string reason;
  EXPECT_FALSE(executor_client.execute_with_timeout(make_trajectory(), 0.05, reason));
  EXPECT_NE(reason.find("Timed out"), std::string::npos);
}

TEST_F(TrajectoryExecutorTest, not_ready_case)
{
  auto client_node = std::make_shared<rclcpp::Node>("trajectory_executor_not_ready_client");
  hw_adapter::TrajectoryExecutor executor_client(
    *client_node,
    "/test_hw_adapter/not_used",
    [](std::string & reason) {
      reason = "robot not ready: E-STOP active";
      return false;
    },
    [](std::string & reason) {reason.clear(); return true;},
    100ms);

  std::string reason;
  EXPECT_FALSE(executor_client.execute(make_trajectory(), reason));
  EXPECT_EQ(reason, "robot not ready: E-STOP active");
}

TEST_F(TrajectoryExecutorTest, rejects_more_than_200_points)
{
  auto client_node = std::make_shared<rclcpp::Node>("trajectory_executor_too_many_points_client");
  hw_adapter::TrajectoryExecutor executor_client(*client_node, "/test_hw_adapter/not_used");

  std::string reason;
  EXPECT_FALSE(executor_client.execute(make_trajectory(201U), reason));
  EXPECT_NE(reason.find("200"), std::string::npos);
  EXPECT_NE(reason.find("downsampling"), std::string::npos);
}

TEST_F(TrajectoryExecutorTest, rejects_empty_trajectory)
{
  auto client_node = std::make_shared<rclcpp::Node>("trajectory_executor_empty_client");
  hw_adapter::TrajectoryExecutor executor_client(*client_node, "/test_hw_adapter/not_used");

  trajectory_msgs::msg::JointTrajectory empty_traj;
  std::string reason;
  EXPECT_FALSE(executor_client.execute(empty_traj, reason));
  EXPECT_EQ(reason, "trajectory is empty");
}
