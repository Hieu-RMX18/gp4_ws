// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <gtest/gtest.h>
#include <industrial_msgs/msg/robot_status.hpp>
#include <motoros2_interfaces/msg/motion_ready_enum.hpp>
#include <motoros2_interfaces/srv/start_traj_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "hw_adapter/hw_adapter_node.hpp"

namespace
{
using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFjt = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

trajectory_msgs::msg::JointTrajectory make_trajectory()
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {
    "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

  for (std::size_t index = 0; index < 2U; ++index)
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

industrial_msgs::msg::RobotStatus make_ready_status()
{
  industrial_msgs::msg::RobotStatus status;
  status.mode.val = industrial_msgs::msg::RobotMode::AUTO;
  status.e_stopped.val = industrial_msgs::msg::TriState::FALSE;
  status.drives_powered.val = industrial_msgs::msg::TriState::TRUE;
  status.motion_possible.val = industrial_msgs::msg::TriState::TRUE;
  status.in_motion.val = industrial_msgs::msg::TriState::FALSE;
  status.in_error.val = industrial_msgs::msg::TriState::FALSE;
  return status;
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

bool wait_until(
  const std::function<bool()> & predicate,
  const std::chrono::milliseconds timeout,
  const std::chrono::milliseconds poll_period = 10ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (predicate())
    {
      return true;
    }
    std::this_thread::sleep_for(poll_period);
  }

  return predicate();
}

class FollowJointTrajectoryServerHarness
{
public:
  enum class Behavior
  {
    kSucceed,
    kAbort,
    kHoldForCancel
  };

  FollowJointTrajectoryServerHarness(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::string & action_name,
    Behavior behavior)
  : behavior_(behavior)
  {
    server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      node,
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

  int goal_count() const
  {
    return goal_count_.load();
  }

  int cancel_count() const
  {
    return cancel_count_.load();
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
    ++cancel_count_;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFjt> goal_handle)
  {
    workers_.emplace_back([this, goal_handle]() {
      if (behavior_ == Behavior::kHoldForCancel)
      {
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        while (std::chrono::steady_clock::now() < deadline)
        {
          if (goal_handle->is_canceling())
          {
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            result->error_string = "execution canceled by stop_motion";
            goal_handle->canceled(result);
            return;
          }
          std::this_thread::sleep_for(10ms);
        }
      }

      auto result = std::make_shared<FollowJointTrajectory::Result>();
      if (behavior_ == Behavior::kAbort)
      {
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "controller reported fatal execution fault";
        goal_handle->abort(result);
        return;
      }

      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string.clear();
      goal_handle->succeed(result);
    });
  }

  Behavior behavior_;
  std::atomic<int> goal_count_{0};
  std::atomic<int> cancel_count_{0};
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
  std::vector<std::thread> workers_;
};

class HwAdapterNodeTest : public ::testing::Test
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

  static rclcpp::NodeOptions make_node_options(
    const std::string & robot_status_topic,
    const std::string & action_name,
    const std::string & start_service,
    const std::string & stop_service = std::string())
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("robot_status_topic", robot_status_topic),
      rclcpp::Parameter("follow_joint_trajectory_action", action_name),
      rclcpp::Parameter("start_traj_mode_service", start_service),
      rclcpp::Parameter("reset_error_service", "/test_hw_adapter/reset_error_unused"),
      rclcpp::Parameter("stop_motion_service", stop_service)});
    return options;
  }
};
}  // namespace

TEST_F(HwAdapterNodeTest, orchestrator_blocks_when_not_ready)
{
  const std::string robot_status_topic = "/test_hw_adapter/orchestrator_not_ready_status";
  const std::string action_name = "/test_hw_adapter/orchestrator_not_ready_action";
  const std::string start_service = "/test_hw_adapter/orchestrator_not_ready_start";

  auto server_node = std::make_shared<rclcpp::Node>("hw_adapter_node_not_ready_server");
  std::atomic<int> start_requests{0};
  auto start_server = server_node->create_service<motoros2_interfaces::srv::StartTrajMode>(
    start_service,
    [&start_requests](
      const std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Request>,
      std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Response> response)
    {
      ++start_requests;
      response->result_code.value = motoros2_interfaces::msg::MotionReadyEnum::READY;
      response->message = "Ready";
    });
  (void)start_server;

  auto hw_node = std::make_shared<hw_adapter::HwAdapterNode>(
    make_node_options(robot_status_topic, action_name, start_service));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(hw_node);
  ExecutorThread spin_thread(executor);

  const auto report = hw_node->execute_trajectory(make_trajectory(), 300ms);

  EXPECT_FALSE(report.success);
  EXPECT_TRUE(report.blocked);
  EXPECT_FALSE(report.fatal_error);
  EXPECT_EQ(start_requests.load(), 0);
  EXPECT_NE(report.message.find("unknown"), std::string::npos);
}

TEST_F(HwAdapterNodeTest, orchestrator_allows_execution_when_ready)
{
  const std::string robot_status_topic = "/test_hw_adapter/orchestrator_ready_status";
  const std::string action_name = "/test_hw_adapter/orchestrator_ready_action";
  const std::string start_service = "/test_hw_adapter/orchestrator_ready_start";

  auto server_node = std::make_shared<rclcpp::Node>("hw_adapter_node_ready_server");
  auto publisher_node = std::make_shared<rclcpp::Node>("hw_adapter_node_ready_publisher");
  std::atomic<int> start_requests{0};

  auto start_server = server_node->create_service<motoros2_interfaces::srv::StartTrajMode>(
    start_service,
    [&start_requests](
      const std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Request>,
      std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Response> response)
    {
      ++start_requests;
      response->result_code.value = motoros2_interfaces::msg::MotionReadyEnum::READY;
      response->message = "Ready";
    });
  (void)start_server;

  FollowJointTrajectoryServerHarness action_server(
    server_node,
    action_name,
    FollowJointTrajectoryServerHarness::Behavior::kSucceed);

  auto status_publisher =
    publisher_node->create_publisher<industrial_msgs::msg::RobotStatus>(robot_status_topic, rclcpp::QoS(10));

  auto hw_node = std::make_shared<hw_adapter::HwAdapterNode>(
    make_node_options(robot_status_topic, action_name, start_service));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(publisher_node);
  executor.add_node(hw_node);
  ExecutorThread spin_thread(executor);

  ASSERT_TRUE(wait_until([&status_publisher]() {return status_publisher->get_subscription_count() > 0;} , 500ms));
  status_publisher->publish(make_ready_status());
  ASSERT_TRUE(wait_until([&hw_node]() {return hw_node->robot_status_monitor().is_ready();}, 500ms));

  const auto report = hw_node->execute_trajectory(make_trajectory(), 500ms);

  EXPECT_TRUE(report.success);
  EXPECT_FALSE(report.blocked);
  EXPECT_FALSE(report.fatal_error);
  EXPECT_FALSE(report.stop_motion_attempted);
  EXPECT_EQ(start_requests.load(), 1);
  EXPECT_EQ(action_server.goal_count(), 1);

  const auto snapshot = hw_node->orchestration_snapshot();
  EXPECT_TRUE(snapshot.robot_ready);
  EXPECT_TRUE(snapshot.session_ready);
  EXPECT_TRUE(snapshot.last_execution_success);
  EXPECT_NE(snapshot.status_message.find("completed successfully"), std::string::npos);
}

TEST_F(HwAdapterNodeTest, fatal_error_path_calls_stop_motion)
{
  const std::string robot_status_topic = "/test_hw_adapter/orchestrator_fatal_status";
  const std::string action_name = "/test_hw_adapter/orchestrator_fatal_action";
  const std::string start_service = "/test_hw_adapter/orchestrator_fatal_start";

  auto server_node = std::make_shared<rclcpp::Node>("hw_adapter_node_fatal_server");
  auto publisher_node = std::make_shared<rclcpp::Node>("hw_adapter_node_fatal_publisher");
  std::atomic<int> start_requests{0};

  auto start_server = server_node->create_service<motoros2_interfaces::srv::StartTrajMode>(
    start_service,
    [&start_requests](
      const std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Request>,
      std::shared_ptr<motoros2_interfaces::srv::StartTrajMode::Response> response)
    {
      ++start_requests;
      response->result_code.value = motoros2_interfaces::msg::MotionReadyEnum::READY;
      response->message = "Ready";
    });
  (void)start_server;

  FollowJointTrajectoryServerHarness action_server(
    server_node,
    action_name,
    FollowJointTrajectoryServerHarness::Behavior::kHoldForCancel);

  auto status_publisher =
    publisher_node->create_publisher<industrial_msgs::msg::RobotStatus>(robot_status_topic, rclcpp::QoS(10));

  auto hw_node = std::make_shared<hw_adapter::HwAdapterNode>(
    make_node_options(robot_status_topic, action_name, start_service));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(publisher_node);
  executor.add_node(hw_node);
  ExecutorThread spin_thread(executor);

  ASSERT_TRUE(wait_until([&status_publisher]() {return status_publisher->get_subscription_count() > 0;} , 500ms));
  status_publisher->publish(make_ready_status());
  ASSERT_TRUE(wait_until([&hw_node]() {return hw_node->robot_status_monitor().is_ready();}, 500ms));

  const auto report = hw_node->execute_trajectory(make_trajectory(), 50ms);
  ASSERT_TRUE(wait_until([&action_server]() {return action_server.cancel_count() > 0;}, 500ms));

  EXPECT_FALSE(report.success);
  EXPECT_FALSE(report.blocked);
  EXPECT_TRUE(report.fatal_error);
  EXPECT_TRUE(report.stop_motion_attempted);
  EXPECT_TRUE(report.stop_motion_succeeded);
  EXPECT_EQ(start_requests.load(), 1);
  EXPECT_EQ(action_server.goal_count(), 1);
  EXPECT_GT(action_server.cancel_count(), 0);
  EXPECT_NE(report.message.find("Timed out"), std::string::npos);

  const auto snapshot = hw_node->orchestration_snapshot();
  EXPECT_TRUE(snapshot.last_error_was_fatal);
  // V4 J4-Recovery: after fatal error, recovery FSM runs. In test environment
  // (no reset_error service), recovery will fail. Status message contains "fatal".
  EXPECT_NE(snapshot.status_message.find("fatal"), std::string::npos);
}
