// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <action_msgs/msg/goal_info.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "interfaces/action/execute_motion.hpp"
#include "interfaces/msg/robot_readiness.hpp"
#include "interfaces/srv/validate_command.hpp"
#include "supervisor/audit_logger.hpp"

namespace
{
using ExecuteMotionFeedbackMessage = interfaces::action::ExecuteMotion::Impl::FeedbackMessage;
using ValidateCommandRequest = interfaces::srv::ValidateCommand_Request;
using ValidateCommandResponse = interfaces::srv::ValidateCommand_Response;

class RosContextGuard
{
public:
  RosContextGuard()
  {
    std::filesystem::create_directories("/tmp/ros_test_logs");
    setenv("ROS_LOG_DIR", "/tmp/ros_test_logs", 1);
    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  ~RosContextGuard()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }
};

bool wait_for_subscribers(
  const std::vector<rclcpp::PublisherBase::SharedPtr> & publishers,
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    bool all_connected = true;
    for (const auto & publisher : publishers)
    {
      if (publisher->get_subscription_count() == 0U)
      {
        all_connected = false;
        break;
      }
    }

    if (all_connected)
    {
      return true;
    }

    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return false;
}
}  // namespace

TEST(AuditLoggerTest, RecordsBagAndJsonlWithoutBlocking)
{
  RosContextGuard context_guard;

  const auto temp_root = std::filesystem::path("/tmp") / "supervisor_audit_logger_test";
  std::filesystem::remove_all(temp_root);
  std::filesystem::create_directories(temp_root);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
    rclcpp::Parameter("audit_log_path", temp_root.string()),
  });

  auto node = std::make_shared<rclcpp::Node>("audit_logger_test_node", options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::string bag_uri;
  std::string jsonl_path;
  uint64_t max_callback_latency_ns = 0U;

  {
    supervisor::AuditLogger audit_logger(*node);

    auto status_pub = node->create_publisher<action_msgs::msg::GoalStatusArray>(
      "/execute_motion/_action/status",
      rclcpp::QoS(10).reliable());
    auto feedback_pub = node->create_publisher<ExecuteMotionFeedbackMessage>(
      "/execute_motion/_action/feedback",
      rclcpp::QoS(10).reliable());
    auto validate_request_pub = node->create_publisher<ValidateCommandRequest>(
      "/validate_command/_request",
      rclcpp::ServicesQoS());
    auto validate_response_pub = node->create_publisher<ValidateCommandResponse>(
      "/validate_command/_response",
      rclcpp::ServicesQoS());
    auto ready_pub = node->create_publisher<interfaces::msg::RobotReadiness>(
      "/hw_adapter/ready",
      rclcpp::QoS(1).reliable().transient_local());

    const std::vector<rclcpp::PublisherBase::SharedPtr> publishers = {
      status_pub, feedback_pub, validate_request_pub, validate_response_pub, ready_pub};

    ASSERT_TRUE(wait_for_subscribers(publishers, executor, std::chrono::seconds(5)));

    action_msgs::msg::GoalStatusArray status_message;
    action_msgs::msg::GoalStatus goal_status;
    goal_status.status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    goal_status.goal_info.stamp = node->now();
    goal_status.goal_info.goal_id.uuid[0] = 0xAB;
    status_message.status_list.push_back(goal_status);

    ExecuteMotionFeedbackMessage feedback_message;
    feedback_message.goal_id.uuid[0] = 0xCD;
    feedback_message.feedback.progress = 0.5;
    feedback_message.feedback.current_state = "executing";

    ValidateCommandRequest validate_request_message;
    validate_request_message.command_json = R"({"primitive":"PTP","target":"home"})";
    validate_request_message.primitive_type = "PTP";
    validate_request_message.target_pose.orientation.w = 1.0;
    validate_request_message.velocity_scale = 0.2;

    ValidateCommandResponse validate_response_message;
    validate_response_message.valid = true;
    validate_response_message.reason = "accepted";
    validate_response_message.sanitized_json = R"({"primitive":"PTP","target":"home"})";

    interfaces::msg::RobotReadiness readiness_message;
    readiness_message.ready = true;
    readiness_message.status_message = "ready";

    status_pub->publish(status_message);
    feedback_pub->publish(feedback_message);
    validate_request_pub->publish(validate_request_message);
    validate_response_pub->publish(validate_response_message);
    ready_pub->publish(readiness_message);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline &&
      audit_logger.written_message_count() < 5U)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    ASSERT_TRUE(audit_logger.wait_for_written_messages(5U, std::chrono::seconds(5)));
    EXPECT_GE(audit_logger.received_message_count(), 5U);

    max_callback_latency_ns = audit_logger.max_callback_latency_ns();
    EXPECT_LT(max_callback_latency_ns, 1000000U);

    bag_uri = audit_logger.bag_uri();
    jsonl_path = audit_logger.jsonl_path();
  }

  executor.remove_node(node);

  ASSERT_TRUE(std::filesystem::exists(jsonl_path));
  ASSERT_TRUE(std::filesystem::exists(bag_uri));

  std::ifstream jsonl_stream(jsonl_path);
  ASSERT_TRUE(jsonl_stream.is_open());

  Json::CharReaderBuilder builder;
  std::size_t jsonl_line_count = 0U;
  std::string line;
  while (std::getline(jsonl_stream, line))
  {
    if (line.empty())
    {
      continue;
    }

    Json::Value parsed;
    std::string errors;
    std::istringstream stream(line);
    ASSERT_TRUE(Json::parseFromStream(builder, stream, &parsed, &errors)) << errors;
    ++jsonl_line_count;
  }

  EXPECT_GE(jsonl_line_count, 5U);

  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);

  std::size_t bag_message_count = 0U;
  while (reader.has_next())
  {
    reader.read_next();
    ++bag_message_count;
  }

  EXPECT_GE(bag_message_count, 5U);
  EXPECT_LT(max_callback_latency_ns, 1000000U);
}
