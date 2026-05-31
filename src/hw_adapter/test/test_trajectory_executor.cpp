// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
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

namespace {
using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFjt = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

std::vector<std::string> canonical_joint_names() {
  return {"joint_1_s", "joint_2_l", "joint_3_u",
          "joint_4_r", "joint_5_b", "joint_6_t"};
}

trajectory_msgs::msg::JointTrajectory
make_trajectory(const std::size_t point_count = 2U) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = canonical_joint_names();
  traj.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  for (std::size_t index = 0; index < point_count; ++index) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.1,  -0.2,
                       0.3, -0.4, static_cast<double>(index) * 0.1};
    point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.time_from_start =
        rclcpp::Duration::from_seconds(static_cast<double>(index) * 0.1);
    traj.points.push_back(point);
  }

  return traj;
}

hw_adapter::ExecutionRuntimeSnapshot
make_runtime_snapshot(const std::vector<double> &positions,
                      const bool valid = true) {
  hw_adapter::ExecutionRuntimeSnapshot snapshot;
  snapshot.joint_state_valid = valid;
  snapshot.current_joint_positions = positions;
  snapshot.robot_ready = valid;
  snapshot.session_ready = valid;
  snapshot.failure_reason = valid ? "" : "runtime snapshot invalid";
  snapshot.session_status = valid ? "session ready" : "session not ready";
  snapshot.joint_state_age = std::chrono::milliseconds(10);
  snapshot.robot_status_age = std::chrono::milliseconds(10);
  return snapshot;
}

class ExecutorThread {
public:
  explicit ExecutorThread(rclcpp::executors::SingleThreadedExecutor &executor)
      : executor_(executor), thread_([this]() { executor_.spin(); }) {}

  ~ExecutorThread() {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor &executor_;
  std::thread thread_;
};

class FollowJointTrajectoryServerHarness {
public:
  enum class Behavior {
    kSucceed,
    kTimeout,
    kAbortInvalidJoints,
    kAbortGoalTolerance
  };

  FollowJointTrajectoryServerHarness(const std::string &node_name,
                                     const std::string &action_name,
                                     Behavior behavior)
      : node_(std::make_shared<rclcpp::Node>(node_name)), behavior_(behavior) {
    server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        node_, action_name,
        std::bind(&FollowJointTrajectoryServerHarness::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&FollowJointTrajectoryServerHarness::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&FollowJointTrajectoryServerHarness::handle_accepted, this,
                  std::placeholders::_1));
  }

  ~FollowJointTrajectoryServerHarness() {
    for (auto &worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  rclcpp::Node::SharedPtr node() const { return node_; }

  int goal_count() const { return goal_count_.load(); }

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const FollowJointTrajectory::Goal>) {
    ++goal_count_;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleFjt>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFjt> goal_handle) {
    workers_.emplace_back([this, goal_handle]() {
      if (behavior_ == Behavior::kTimeout) {
        std::this_thread::sleep_for(250ms);
      }

      auto result = std::make_shared<FollowJointTrajectory::Result>();
      if (behavior_ == Behavior::kAbortInvalidJoints) {
        result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
        result->error_string = "joint order mismatch at controller";
        goal_handle->abort(result);
        return;
      }
      if (behavior_ == Behavior::kAbortGoalTolerance) {
        result->error_code =
            FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "goal tolerance exceeded";
        goal_handle->abort(result);
        return;
      }

      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string.clear();
      goal_handle->succeed(result);
    });
  }

  rclcpp::Node::SharedPtr node_;
  Behavior behavior_;
  std::atomic<int> goal_count_{0};
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
  std::vector<std::thread> workers_;
};

class TrajectoryExecutorTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};
} // namespace

TEST_F(TrajectoryExecutorTest, successful_goal_send_with_runtime_checks) {
  const std::string action_name = "/test_hw_adapter/fjt_success";
  FollowJointTrajectoryServerHarness server(
      "trajectory_executor_success_server", action_name,
      FollowJointTrajectoryServerHarness::Behavior::kSucceed);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server.node());
  ExecutorThread spin_thread(executor);

  auto client_node =
      std::make_shared<rclcpp::Node>("trajectory_executor_success_client");
  const auto trajectory = make_trajectory();
  hw_adapter::TrajectoryExecutor executor_client(
      *client_node, action_name,
      [trajectory]() {
        return make_runtime_snapshot(trajectory.points.front().positions);
      },
      500ms, canonical_joint_names());

  hw_adapter::TrajectoryExecutionRequest request;
  request.trajectory = trajectory;
  request.result_timeout = 500ms;
  request.expected_start_positions = trajectory.points.front().positions;
  request.enforce_start_state_match = true;
  const auto result = executor_client.execute_blocking(request);

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.completed);
  EXPECT_EQ(result.failure_stage, "none");
  EXPECT_EQ(server.goal_count(), 1);
}

TEST_F(TrajectoryExecutorTest, timeout_case) {
  const std::string action_name = "/test_hw_adapter/fjt_timeout";
  FollowJointTrajectoryServerHarness server(
      "trajectory_executor_timeout_server", action_name,
      FollowJointTrajectoryServerHarness::Behavior::kTimeout);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server.node());
  ExecutorThread spin_thread(executor);

  auto client_node =
      std::make_shared<rclcpp::Node>("trajectory_executor_timeout_client");
  const auto trajectory = make_trajectory();
  hw_adapter::TrajectoryExecutor executor_client(
      *client_node, action_name,
      [trajectory]() {
        return make_runtime_snapshot(trajectory.points.front().positions);
      },
      50ms, canonical_joint_names());

  hw_adapter::TrajectoryExecutionRequest request;
  request.trajectory = trajectory;
  request.result_timeout = 50ms;
  const auto result = executor_client.execute_blocking(request);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_stage, "result_wait");
  EXPECT_NE(result.message.find("Timed out"), std::string::npos);
}

TEST_F(TrajectoryExecutorTest, rejects_more_than_200_points) {
  auto client_node =
      std::make_shared<rclcpp::Node>("trajectory_executor_too_many_points");
  hw_adapter::TrajectoryExecutor executor_client(
      *client_node, "/test_hw_adapter/not_used", []() {
        return make_runtime_snapshot(std::vector<double>(6U, 0.0), true);
      });

  std::string reason;
  EXPECT_FALSE(executor_client.validate_trajectory_request(
      make_trajectory(201U), reason));
  EXPECT_NE(reason.find("200"), std::string::npos);
}

TEST_F(TrajectoryExecutorTest, rejects_wrong_joint_order) {
  auto client_node =
      std::make_shared<rclcpp::Node>("trajectory_executor_wrong_joint_order");
  hw_adapter::TrajectoryExecutor executor_client(*client_node,
                                                 "/test_hw_adapter/not_used");

  auto trajectory = make_trajectory();
  std::reverse(trajectory.joint_names.begin(), trajectory.joint_names.end());

  std::string reason;
  EXPECT_FALSE(executor_client.validate_trajectory_request(trajectory, reason));
  EXPECT_NE(reason.find("canonical"), std::string::npos);
}

TEST_F(TrajectoryExecutorTest, rejects_vector_size_mismatch) {
  auto client_node = std::make_shared<rclcpp::Node>(
      "trajectory_executor_vector_size_mismatch");
  hw_adapter::TrajectoryExecutor executor_client(*client_node,
                                                 "/test_hw_adapter/not_used");

  auto trajectory = make_trajectory();
  trajectory.points[0].velocities.pop_back();

  std::string reason;
  EXPECT_FALSE(executor_client.validate_trajectory_request(trajectory, reason));
  EXPECT_NE(reason.find("velocities"), std::string::npos);
}

TEST_F(TrajectoryExecutorTest, rejects_nan_or_inf_values) {
  auto client_node = std::make_shared<rclcpp::Node>("trajectory_executor_nan");
  hw_adapter::TrajectoryExecutor executor_client(*client_node,
                                                 "/test_hw_adapter/not_used");

  auto trajectory = make_trajectory();
  trajectory.points[1].positions[2] = std::numeric_limits<double>::quiet_NaN();

  std::string reason;
  EXPECT_FALSE(executor_client.validate_trajectory_request(trajectory, reason));
  EXPECT_NE(reason.find("NaN"), std::string::npos);
}

TEST_F(TrajectoryExecutorTest, rejects_non_monotonic_timestamps) {
  auto client_node =
      std::make_shared<rclcpp::Node>("trajectory_executor_non_monotonic");
  hw_adapter::TrajectoryExecutor executor_client(*client_node,
                                                 "/test_hw_adapter/not_used");

  auto trajectory = make_trajectory();
  trajectory.points[1].time_from_start = trajectory.points[0].time_from_start;

  std::string reason;
  EXPECT_FALSE(executor_client.validate_trajectory_request(trajectory, reason));
  EXPECT_NE(reason.find("strictly monotonic"), std::string::npos);
}

TEST_F(TrajectoryExecutorTest, runtime_snapshot_invalid_blocks_before_send) {
  const std::string action_name = "/test_hw_adapter/fjt_runtime_invalid";
  FollowJointTrajectoryServerHarness server(
      "trajectory_executor_runtime_invalid_server", action_name,
      FollowJointTrajectoryServerHarness::Behavior::kSucceed);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server.node());
  ExecutorThread spin_thread(executor);

  auto client_node = std::make_shared<rclcpp::Node>(
      "trajectory_executor_runtime_invalid_client");
  hw_adapter::TrajectoryExecutor executor_client(
      *client_node, action_name,
      []() {
        hw_adapter::ExecutionRuntimeSnapshot snapshot;
        snapshot.joint_state_valid = false;
        snapshot.robot_ready = true;
        snapshot.session_ready = true;
        snapshot.failure_reason = "joint state stale";
        return snapshot;
      },
      200ms, canonical_joint_names());

  hw_adapter::TrajectoryExecutionRequest request;
  request.trajectory = make_trajectory();
  request.result_timeout = 200ms;
  const auto result = executor_client.execute_blocking(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_stage, "runtime_preflight");
  EXPECT_NE(result.message.find("joint state"), std::string::npos);
  EXPECT_EQ(server.goal_count(), 0);
}

TEST_F(TrajectoryExecutorTest, start_state_gate_rejects_preflight_mismatch) {
  const std::string action_name = "/test_hw_adapter/fjt_start_state_gate";
  FollowJointTrajectoryServerHarness server(
      "trajectory_executor_start_state_gate_server", action_name,
      FollowJointTrajectoryServerHarness::Behavior::kSucceed);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server.node());
  ExecutorThread spin_thread(executor);

  auto client_node = std::make_shared<rclcpp::Node>(
      "trajectory_executor_start_state_gate_client");
  const auto trajectory = make_trajectory();
  hw_adapter::TrajectoryExecutor executor_client(
      *client_node, action_name,
      []() { return make_runtime_snapshot(std::vector<double>(6U, 0.0)); },
      500ms, canonical_joint_names());

  hw_adapter::TrajectoryExecutionRequest request;
  request.trajectory = trajectory;
  request.result_timeout = 500ms;
  request.expected_start_positions = std::vector<double>(6U, 0.5);
  request.enforce_start_state_match = true;
  const auto result = executor_client.execute_blocking(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_stage, "start_state_gate_preflight");
  EXPECT_GT(result.max_start_state_abs_delta, 0.01);
  EXPECT_EQ(server.goal_count(), 0);
}

TEST_F(TrajectoryExecutorTest,
       commit_time_drift_recheck_blocks_before_controller_send) {
  const std::string action_name = "/test_hw_adapter/fjt_commit_drift";
  FollowJointTrajectoryServerHarness server(
      "trajectory_executor_commit_drift_server", action_name,
      FollowJointTrajectoryServerHarness::Behavior::kSucceed);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server.node());
  ExecutorThread spin_thread(executor);

  auto client_node =
      std::make_shared<rclcpp::Node>("trajectory_executor_commit_drift_client");
  const auto trajectory = make_trajectory();
  const std::vector<double> preflight_positions =
      trajectory.points.front().positions;
  std::vector<double> commit_positions = preflight_positions;
  commit_positions[0] += 0.018; // >0.01 drift threshold
  std::vector<double> expected_positions = preflight_positions;
  expected_positions[0] +=
      0.009; // preflight/commit each still within 0.01 of expected

  int snapshot_call_count = 0;
  hw_adapter::TrajectoryExecutor executor_client(
      *client_node, action_name,
      [preflight_positions, commit_positions, &snapshot_call_count]() mutable {
        ++snapshot_call_count;
        return snapshot_call_count == 1
                   ? make_runtime_snapshot(preflight_positions)
                   : make_runtime_snapshot(commit_positions);
      },
      500ms, canonical_joint_names());

  hw_adapter::TrajectoryExecutionRequest request;
  request.trajectory = trajectory;
  request.result_timeout = 500ms;
  request.expected_start_positions = expected_positions;
  request.enforce_start_state_match = true;
  const auto result = executor_client.execute_blocking(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_stage, "start_state_drift_commit");
  EXPECT_GT(result.commit_drift_max_abs_delta, 0.01);
  EXPECT_EQ(server.goal_count(), 0);
}

TEST_F(TrajectoryExecutorTest,
       decodes_controller_error_codes_and_preserves_error_string) {
  const std::string action_name = "/test_hw_adapter/fjt_decode_error_codes";
  FollowJointTrajectoryServerHarness server(
      "trajectory_executor_decode_error_codes_server", action_name,
      FollowJointTrajectoryServerHarness::Behavior::kAbortInvalidJoints);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server.node());
  ExecutorThread spin_thread(executor);

  auto client_node = std::make_shared<rclcpp::Node>(
      "trajectory_executor_decode_error_codes_client");
  const auto trajectory = make_trajectory();
  hw_adapter::TrajectoryExecutor executor_client(
      *client_node, action_name,
      [trajectory]() {
        return make_runtime_snapshot(trajectory.points.front().positions);
      },
      500ms, canonical_joint_names());

  hw_adapter::TrajectoryExecutionRequest request;
  request.trajectory = trajectory;
  request.result_timeout = 500ms;
  const auto result = executor_client.execute_blocking(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_stage, "controller_aborted");
  EXPECT_EQ(result.controller_error_name, "INVALID_JOINTS");
  EXPECT_EQ(result.controller_error_code,
            FollowJointTrajectory::Result::INVALID_JOINTS);
  EXPECT_EQ(result.controller_error_string,
            "joint order mismatch at controller");
}
