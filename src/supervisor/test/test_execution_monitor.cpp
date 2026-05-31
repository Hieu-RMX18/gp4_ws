// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/action/execute_motion.hpp"
#include "supervisor/execution_monitor.hpp"

namespace {
using ExecuteMotionFeedbackMessage =
    interfaces::action::ExecuteMotion::Impl::FeedbackMessage;
using ExecuteMotionSendGoalRequest =
    interfaces::action::ExecuteMotion_SendGoal_Request;
using ExecuteMotionSendGoalResponse =
    interfaces::action::ExecuteMotion_SendGoal_Response;

class RosContextGuard {
public:
  RosContextGuard() {
    setenv("ROS_LOG_DIR", "/tmp/ros_test_logs", 1);
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  ~RosContextGuard() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

unique_identifier_msgs::msg::UUID make_uuid(const uint8_t seed) {
  unique_identifier_msgs::msg::UUID goal_id;
  goal_id.uuid[0] = seed;
  goal_id.uuid[15] = static_cast<uint8_t>(seed + 1U);
  return goal_id;
}

std::string
goal_id_to_string(const unique_identifier_msgs::msg::UUID &goal_id) {
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (const auto byte : goal_id.uuid) {
    oss << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

action_msgs::msg::GoalStatusArray
make_status_array(const unique_identifier_msgs::msg::UUID &goal_id,
                  const int8_t status_code, const rclcpp::Time &stamp) {
  action_msgs::msg::GoalStatusArray status_array;
  action_msgs::msg::GoalStatus goal_status;
  goal_status.goal_info.goal_id = goal_id;
  goal_status.goal_info.stamp = stamp;
  goal_status.status = status_code;
  status_array.status_list.push_back(goal_status);
  return status_array;
}

action_msgs::msg::GoalStatusArray make_status_array(
    const std::vector<std::pair<unique_identifier_msgs::msg::UUID, int8_t>>
        &entries,
    const rclcpp::Time &stamp) {
  action_msgs::msg::GoalStatusArray status_array;
  for (const auto &[goal_id, status_code] : entries) {
    action_msgs::msg::GoalStatus goal_status;
    goal_status.goal_info.goal_id = goal_id;
    goal_status.goal_info.stamp = stamp;
    goal_status.status = status_code;
    status_array.status_list.push_back(goal_status);
  }
  return status_array;
}

bool wait_for_condition(rclcpp::executors::SingleThreadedExecutor &executor,
                        const std::function<bool()> &predicate,
                        const std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  executor.spin_some();
  return predicate();
}

bool wait_for_connections(
    rclcpp::executors::SingleThreadedExecutor &executor,
    const std::vector<rclcpp::PublisherBase::SharedPtr> &publishers,
    const rclcpp::SubscriptionBase::SharedPtr &alert_sub) {
  return wait_for_condition(
      executor,
      [&publishers, &alert_sub]() {
        if (!alert_sub || alert_sub->get_publisher_count() == 0U) {
          return false;
        }
        for (const auto &publisher : publishers) {
          if (!publisher || publisher->get_subscription_count() == 0U) {
            return false;
          }
        }
        return true;
      },
      std::chrono::seconds(5));
}
} // namespace

class ExecutionMonitorFixture : public ::testing::Test {
protected:
  void SetUp() override {
    std::filesystem::create_directories("/tmp/ros_test_logs");

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
        rclcpp::Parameter("execution_monitor_nominal_duration_sec", 0.02),
        rclcpp::Parameter("execution_monitor_check_period_ms", 10),
        rclcpp::Parameter("execution_monitor_min_velocity_scale", 0.05),
        rclcpp::Parameter("execution_monitor_timeout_multiplier", 2.0),
        rclcpp::Parameter("supervisor_alert_heartbeat_period_ms", 50),
    });

    node_ =
        std::make_shared<rclcpp::Node>("execution_monitor_test_node", options);
    monitor_ = std::make_unique<supervisor::ExecutionMonitor>(*node_);

    alert_sub_ =
        node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
            "/supervisor/alerts", rclcpp::QoS(10).reliable(),
            [this](
                const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(alert_mutex_);
              alerts_.push_back(*msg);
            });

    send_goal_request_pub_ =
        node_->create_publisher<ExecuteMotionSendGoalRequest>(
            "/execute_motion/_action/send_goal/_request",
            rclcpp::ServicesQoS());
    send_goal_response_pub_ =
        node_->create_publisher<ExecuteMotionSendGoalResponse>(
            "/execute_motion/_action/send_goal/_response",
            rclcpp::ServicesQoS());
    status_pub_ = node_->create_publisher<action_msgs::msg::GoalStatusArray>(
        "/execute_motion/_action/status", rclcpp::QoS(10).reliable());
    feedback_pub_ = node_->create_publisher<ExecuteMotionFeedbackMessage>(
        "/execute_motion/_action/feedback", rclcpp::QoS(10).reliable());

    executor_.add_node(node_);

    ASSERT_TRUE(wait_for_connections(executor_,
                                     {
                                         send_goal_request_pub_,
                                         send_goal_response_pub_,
                                         status_pub_,
                                         feedback_pub_,
                                     },
                                     alert_sub_));
  }

  void TearDown() override {
    executor_.remove_node(node_);
    monitor_.reset();
    node_.reset();
  }

  void publish_goal_request(const unique_identifier_msgs::msg::UUID &goal_id,
                            const double velocity_scale) {
    ExecuteMotionSendGoalRequest request;
    request.goal_id = goal_id;
    request.goal.velocity_scale = velocity_scale;
    request.goal.acceleration_scale = velocity_scale;
    request.goal.primitive_type = "PTP";
    request.goal.target_pose.orientation.w = 1.0;
    send_goal_request_pub_->publish(request);
  }

  void publish_goal_response(const bool accepted) {
    ExecuteMotionSendGoalResponse response;
    response.accepted = accepted;
    response.stamp = node_->now();
    send_goal_response_pub_->publish(response);
  }

  void publish_feedback(const unique_identifier_msgs::msg::UUID &goal_id,
                        const double progress,
                        const std::string &current_state) {
    ExecuteMotionFeedbackMessage feedback;
    feedback.goal_id = goal_id;
    feedback.feedback.progress = progress;
    feedback.feedback.current_state = current_state;
    feedback_pub_->publish(feedback);
  }

  void publish_status(const unique_identifier_msgs::msg::UUID &goal_id,
                      const int8_t status_code) {
    status_pub_->publish(make_status_array(goal_id, status_code, node_->now()));
  }

  void publish_statuses(
      const std::vector<std::pair<unique_identifier_msgs::msg::UUID, int8_t>>
          &entries) {
    status_pub_->publish(make_status_array(entries, node_->now()));
  }

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> alerts_copy() {
    std::lock_guard<std::mutex> lock(alert_mutex_);
    return alerts_;
  }

  bool wait_for_alert_message(const std::string &message, const uint8_t level) {
    return wait_for_condition(
        executor_,
        [this, &message, level]() {
          const auto alerts = alerts_copy();
          for (const auto &alert : alerts) {
            if (alert.level == level && alert.message == message) {
              return true;
            }
          }
          return false;
        },
        std::chrono::seconds(5));
  }

  RosContextGuard context_guard_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<supervisor::ExecutionMonitor> monitor_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::mutex alert_mutex_;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> alerts_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      alert_sub_;
  rclcpp::Publisher<ExecuteMotionSendGoalRequest>::SharedPtr
      send_goal_request_pub_;
  rclcpp::Publisher<ExecuteMotionSendGoalResponse>::SharedPtr
      send_goal_response_pub_;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub_;
  rclcpp::Publisher<ExecuteMotionFeedbackMessage>::SharedPtr feedback_pub_;
};

TEST_F(ExecutionMonitorFixture, NormalCompletionPathPublishesOkAlert) {
  const auto goal_id = make_uuid(0x01U);

  publish_goal_request(goal_id, 0.5);
  publish_goal_response(true);
  publish_status(goal_id, action_msgs::msg::GoalStatus::STATUS_EXECUTING);
  publish_feedback(goal_id, 0.5, "executing");
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  publish_status(goal_id, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  ASSERT_TRUE(
      wait_for_alert_message("execute_motion completed successfully",
                             diagnostic_msgs::msg::DiagnosticStatus::OK));

  const auto snapshot = monitor_->snapshot();
  EXPECT_EQ(snapshot.current_state, "IDLE");
  EXPECT_TRUE(snapshot.last_result_success);
  EXPECT_EQ(snapshot.consecutive_failure_count, 0U);
  EXPECT_EQ(snapshot.last_feedback_state, "executing");
  EXPECT_GT(snapshot.last_execution_time_sec, 0.0);
}

TEST_F(ExecutionMonitorFixture, IdleStatePublishesHeartbeatAlert) {
  ASSERT_TRUE(wait_for_alert_message(
      "idle", diagnostic_msgs::msg::DiagnosticStatus::OK));

  const auto snapshot = monitor_->snapshot();
  EXPECT_EQ(snapshot.current_state, "IDLE");
  EXPECT_EQ(snapshot.last_alert_message, "idle");
}

TEST_F(ExecutionMonitorFixture, TimeoutDetectionPublishesWarnAlert) {
  const auto goal_id = make_uuid(0x11U);

  publish_goal_request(goal_id, 1.0);
  publish_goal_response(true);
  publish_status(goal_id, action_msgs::msg::GoalStatus::STATUS_EXECUTING);

  ASSERT_TRUE(
      wait_for_alert_message("execute_motion exceeded allowed duration",
                             diagnostic_msgs::msg::DiagnosticStatus::WARN));

  const auto snapshot = monitor_->snapshot();
  EXPECT_EQ(snapshot.current_state, "EXECUTING");
  EXPECT_TRUE(snapshot.timeout_alert_active);
  EXPECT_EQ(snapshot.active_goal_id, goal_id_to_string(goal_id));

  publish_status(goal_id, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  ASSERT_TRUE(
      wait_for_alert_message("execute_motion completed successfully",
                             diagnostic_msgs::msg::DiagnosticStatus::OK));
}

TEST_F(ExecutionMonitorFixture, ConsecutiveFailuresPublishWarnAlert) {
  for (uint8_t index = 0U; index < 3U; ++index) {
    const auto goal_id = make_uuid(static_cast<uint8_t>(0x20U + index));
    publish_goal_request(goal_id, 0.5);
    publish_goal_response(true);
    publish_status(goal_id, action_msgs::msg::GoalStatus::STATUS_EXECUTING);
    publish_status(goal_id, action_msgs::msg::GoalStatus::STATUS_ABORTED);

    ASSERT_TRUE(wait_for_condition(
        executor_,
        [this, index]() {
          return monitor_->snapshot().consecutive_failure_count ==
                 static_cast<uint32_t>(index + 1U);
        },
        std::chrono::seconds(5)));
  }

  ASSERT_TRUE(wait_for_alert_message(
      "execute_motion consecutive failure threshold reached",
      diagnostic_msgs::msg::DiagnosticStatus::WARN));

  const auto snapshot = monitor_->snapshot();
  EXPECT_EQ(snapshot.current_state, "IDLE");
  EXPECT_EQ(snapshot.consecutive_failure_count, 3U);
  const auto alerts = alerts_copy();
  bool saw_threshold_warning = false;
  for (const auto &alert : alerts) {
    if (alert.level == diagnostic_msgs::msg::DiagnosticStatus::WARN &&
        alert.message ==
            "execute_motion consecutive failure threshold reached") {
      saw_threshold_warning = true;
      break;
    }
  }
  EXPECT_TRUE(saw_threshold_warning);
}

TEST_F(ExecutionMonitorFixture, FailedExecutionHeartbeatReturnsToOkIdle) {
  const auto goal_id = make_uuid(0x28U);

  publish_goal_request(goal_id, 0.5);
  publish_goal_response(true);
  publish_status(goal_id, action_msgs::msg::GoalStatus::STATUS_EXECUTING);
  publish_status(goal_id, action_msgs::msg::GoalStatus::STATUS_ABORTED);

  ASSERT_TRUE(wait_for_alert_message(
      "execute_motion failed", diagnostic_msgs::msg::DiagnosticStatus::ERROR));

  const auto alerts_after_failure = alerts_copy().size();
  ASSERT_TRUE(wait_for_condition(
      executor_,
      [this, alerts_after_failure]() {
        const auto alerts = alerts_copy();
        for (std::size_t index = alerts_after_failure; index < alerts.size();
             ++index) {
          if (alerts[index].level ==
                  diagnostic_msgs::msg::DiagnosticStatus::OK &&
              alerts[index].message == "idle") {
            return true;
          }
        }
        return false;
      },
      std::chrono::seconds(5)));

  const auto snapshot = monitor_->snapshot();
  EXPECT_EQ(snapshot.current_state, "IDLE");
  EXPECT_FALSE(snapshot.last_result_success);
  EXPECT_EQ(snapshot.consecutive_failure_count, 1U);
}

TEST_F(ExecutionMonitorFixture, IgnoresTerminalHistoryWhenCountingActiveGoals) {
  const auto stale_goal_id = make_uuid(0x30U);
  const auto current_goal_id = make_uuid(0x31U);

  publish_goal_request(current_goal_id, 0.5);
  publish_goal_response(true);
  publish_statuses({
      {stale_goal_id, action_msgs::msg::GoalStatus::STATUS_ABORTED},
      {current_goal_id, action_msgs::msg::GoalStatus::STATUS_EXECUTING},
  });

  ASSERT_TRUE(wait_for_condition(
      executor_,
      [this, &current_goal_id]() {
        const auto snapshot = monitor_->snapshot();
        return snapshot.current_state == "EXECUTING" &&
               snapshot.active_goal_id == goal_id_to_string(current_goal_id) &&
               snapshot.active_goal_count == 1U;
      },
      std::chrono::seconds(5)));

  const auto snapshot = monitor_->snapshot();
  EXPECT_EQ(snapshot.active_goal_count, 1U);
  EXPECT_EQ(snapshot.active_goal_id, goal_id_to_string(current_goal_id));
  EXPECT_EQ(snapshot.current_state, "EXECUTING");
}
