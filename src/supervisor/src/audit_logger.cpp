// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "supervisor/audit_logger.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include <action_msgs/msg/goal_status_array.hpp>
#include <jsoncpp/json/json.h>
#include <rclcpp/serialization.hpp>
#include <rmw/rmw.h>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include "interfaces/action/execute_motion.hpp"
#include "interfaces/msg/robot_readiness.hpp"
#include "interfaces/srv/validate_command.hpp"

namespace {
using ExecuteMotionFeedbackMessage =
    interfaces::action::ExecuteMotion::Impl::FeedbackMessage;
using ValidateCommandRequest = interfaces::srv::ValidateCommand_Request;
using ValidateCommandResponse = interfaces::srv::ValidateCommand_Response;

template <typename ParameterT>
ParameterT declare_or_get_parameter(rclcpp::Node &node, const std::string &name,
                                    const ParameterT &default_value) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<ParameterT>();
  }

  return node.declare_parameter<ParameterT>(name, default_value);
}

std::string default_audit_log_path() { return "/tmp/gp4_audit"; }

std::string timestamp_string() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  const auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - seconds)
          .count();
  const auto raw_time = std::chrono::system_clock::to_time_t(now);

  std::tm time_info{};
  localtime_r(&raw_time, &time_info);

  std::ostringstream oss;
  oss << std::put_time(&time_info, "%Y%m%d_%H%M%S") << "_" << std::setw(9)
      << std::setfill('0') << nanoseconds;
  return oss.str();
}

std::string uuid_to_hex_string(const unique_identifier_msgs::msg::UUID &uuid) {
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (const auto byte : uuid.uuid) {
    oss << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

Json::Value pose_to_json(const geometry_msgs::msg::Pose &pose) {
  Json::Value output(Json::objectValue);

  output["position"]["x"] = pose.position.x;
  output["position"]["y"] = pose.position.y;
  output["position"]["z"] = pose.position.z;

  output["orientation"]["x"] = pose.orientation.x;
  output["orientation"]["y"] = pose.orientation.y;
  output["orientation"]["z"] = pose.orientation.z;
  output["orientation"]["w"] = pose.orientation.w;

  return output;
}

Json::Value parse_json_or_store_raw(const std::string &input) {
  if (input.empty()) {
    return Json::Value(Json::nullValue);
  }

  Json::CharReaderBuilder builder;
  std::string errors;
  Json::Value parsed;
  std::istringstream stream(input);
  if (Json::parseFromStream(builder, stream, &parsed, &errors)) {
    return parsed;
  }

  Json::Value fallback(Json::objectValue);
  fallback["raw"] = input;
  fallback["parse_error"] = errors;
  return fallback;
}

Json::Value
goal_status_array_to_json(const action_msgs::msg::GoalStatusArray &msg) {
  Json::Value output(Json::objectValue);
  Json::Value status_list(Json::arrayValue);

  for (const auto &status : msg.status_list) {
    Json::Value entry(Json::objectValue);
    entry["status"] = status.status;
    entry["goal_id"] = uuid_to_hex_string(status.goal_info.goal_id);
    entry["stamp"]["sec"] = status.goal_info.stamp.sec;
    entry["stamp"]["nanosec"] = status.goal_info.stamp.nanosec;
    status_list.append(entry);
  }

  output["status_list"] = status_list;
  return output;
}

Json::Value
execute_motion_feedback_to_json(const ExecuteMotionFeedbackMessage &msg) {
  Json::Value output(Json::objectValue);
  output["goal_id"] = uuid_to_hex_string(msg.goal_id);
  output["feedback"]["progress"] = msg.feedback.progress;
  output["feedback"]["current_state"] = msg.feedback.current_state;
  return output;
}

Json::Value
validate_command_request_to_json(const ValidateCommandRequest &msg) {
  Json::Value output(Json::objectValue);
  output["command_json"] = parse_json_or_store_raw(msg.command_json);
  output["primitive_type"] = msg.primitive_type;
  output["target_pose"] = pose_to_json(msg.target_pose);
  output["velocity_scale"] = msg.velocity_scale;
  return output;
}

Json::Value
validate_command_response_to_json(const ValidateCommandResponse &msg) {
  Json::Value output(Json::objectValue);
  output["valid"] = msg.valid;
  output["reason"] = msg.reason;
  output["sanitized_json"] = parse_json_or_store_raw(msg.sanitized_json);
  return output;
}

Json::Value
robot_readiness_to_json(const interfaces::msg::RobotReadiness &msg) {
  Json::Value output(Json::objectValue);
  output["ready"] = msg.ready;
  output["status_message"] = msg.status_message;
  return output;
}

template <typename MessageT>
MessageT
deserialize_message(const rclcpp::SerializedMessage &serialized_message) {
  MessageT message;
  rclcpp::Serialization<MessageT> serialization;
  auto copy = serialized_message;
  serialization.deserialize_message(&copy, &message);
  return message;
}
} // namespace

namespace supervisor {
AuditLogger::AuditLogger(rclcpp::Node &node)
    : node_(node), logger_(node.get_logger()) {
  serialization_format_ = rmw_get_serialization_format() == nullptr
                              ? "cdr"
                              : std::string(rmw_get_serialization_format());
  audit_path_ = declare_or_get_parameter<std::string>(node_, "audit_log_path",
                                                      default_audit_log_path());
  max_queue_depth_ = static_cast<std::size_t>(declare_or_get_parameter<int64_t>(
      node_, "audit_logger_max_queue_depth", 2048));

  initialize_output_paths();
  initialize_writer();
  initialize_subscriptions();
  start_worker();

  RCLCPP_INFO(logger_, "audit_logger recording to bag='%s' and jsonl='%s'",
              bag_uri_.string().c_str(), jsonl_path_.string().c_str());
}

AuditLogger::~AuditLogger() {
  stop_worker();

  if (writer_) {
    writer_->close();
  }

  if (jsonl_stream_.is_open()) {
    jsonl_stream_.flush();
    jsonl_stream_.close();
  }
}

std::string AuditLogger::bag_uri() const {
  std::lock_guard<std::mutex> lock(path_mutex_);
  return bag_uri_.string();
}

std::string AuditLogger::jsonl_path() const {
  std::lock_guard<std::mutex> lock(path_mutex_);
  return jsonl_path_.string();
}

uint64_t AuditLogger::max_callback_latency_ns() const {
  return max_callback_latency_ns_.load();
}

std::size_t AuditLogger::received_message_count() const {
  return received_message_count_.load();
}

std::size_t AuditLogger::written_message_count() const {
  return written_message_count_.load();
}

bool AuditLogger::wait_for_written_messages(
    const std::size_t min_count, const std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  return written_cv_.wait_for(lock, timeout, [this, min_count]() {
    return written_message_count_.load() >= min_count;
  });
}

void AuditLogger::initialize_output_paths() {
  const auto run_stamp = timestamp_string();
  std::filesystem::create_directories(audit_path_);

  std::lock_guard<std::mutex> lock(path_mutex_);
  bag_uri_ = std::filesystem::path(audit_path_) / run_stamp;
  jsonl_path_ = std::filesystem::path(audit_path_) / (run_stamp + ".jsonl");

  jsonl_stream_.open(jsonl_path_, std::ios::out | std::ios::trunc);
  if (!jsonl_stream_.is_open()) {
    throw std::runtime_error("failed to open audit JSONL file: " +
                             jsonl_path_.string());
  }
}

void AuditLogger::initialize_writer() {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_uri_.string();
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = serialization_format_;
  converter_options.output_serialization_format = serialization_format_;

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(storage_options, converter_options);

  const auto execute_motion_action_name = declare_or_get_parameter<std::string>(
      node_, "execute_motion_action_name", "/execute_motion");
  const auto validate_request_topic = declare_or_get_parameter<std::string>(
      node_, "validate_command_request_topic", "/validate_command/_request");
  const auto validate_response_topic = declare_or_get_parameter<std::string>(
      node_, "validate_command_response_topic", "/validate_command/_response");
  const auto hw_ready_topic = declare_or_get_parameter<std::string>(
      node_, "hw_adapter_ready_topic", "/hw_adapter/ready");

  topic_specs_ = {
      TopicSpec{
          StreamId::kExecuteStatus,
          execute_motion_action_name + "/_action/status",
          rosidl_generator_traits::name<action_msgs::msg::GoalStatusArray>(),
          rclcpp::QoS(10).reliable()},
      TopicSpec{StreamId::kExecuteFeedback,
                execute_motion_action_name + "/_action/feedback",
                rosidl_generator_traits::name<ExecuteMotionFeedbackMessage>(),
                rclcpp::QoS(10).reliable()},
      TopicSpec{StreamId::kValidateRequest, validate_request_topic,
                rosidl_generator_traits::name<ValidateCommandRequest>(),
                rclcpp::ServicesQoS()},
      TopicSpec{StreamId::kValidateResponse, validate_response_topic,
                rosidl_generator_traits::name<ValidateCommandResponse>(),
                rclcpp::ServicesQoS()},
      TopicSpec{
          StreamId::kHwReady, hw_ready_topic,
          rosidl_generator_traits::name<interfaces::msg::RobotReadiness>(),
          rclcpp::QoS(1).reliable().transient_local()},
  };

  for (const auto &spec : topic_specs_) {
    writer_->create_topic(rosbag2_storage::TopicMetadata{
        spec.topic_name, spec.type_name, serialization_format_, ""});
  }
}

void AuditLogger::initialize_subscriptions() {
  subscriptions_.reserve(topic_specs_.size());

  for (const auto &spec : topic_specs_) {
    subscriptions_.push_back(node_.create_generic_subscription(
        spec.topic_name, spec.type_name, spec.qos,
        [this,
         spec](std::shared_ptr<rclcpp::SerializedMessage> serialized_message) {
          enqueue_message(spec, std::move(serialized_message));
        }));
  }
}

void AuditLogger::start_worker() {
  stop_requested_.store(false);
  worker_thread_ = std::thread(&AuditLogger::worker_loop, this);
}

void AuditLogger::stop_worker() {
  stop_requested_.store(true);
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void AuditLogger::enqueue_message(
    const TopicSpec &spec,
    std::shared_ptr<rclcpp::SerializedMessage> serialized_message) {
  const auto callback_start = std::chrono::steady_clock::now();
  if (!serialized_message) {
    return;
  }

  const auto received_time = node_.get_clock()->now();

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (event_queue_.size() >= max_queue_depth_) {
      event_queue_.pop_front();
      RCLCPP_WARN_THROTTLE(logger_, *node_.get_clock(), 5000,
                           "audit_logger queue full; dropping oldest event to "
                           "remain non-blocking");
    }

    event_queue_.push_back(QueuedEvent{spec.stream_id, spec.topic_name,
                                       spec.type_name, received_time,
                                       std::move(serialized_message)});
    ++received_message_count_;
  }

  queue_cv_.notify_one();

  const auto callback_duration = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - callback_start)
          .count());
  auto current_max = max_callback_latency_ns_.load();
  while (callback_duration > current_max &&
         !max_callback_latency_ns_.compare_exchange_weak(current_max,
                                                         callback_duration)) {
  }
}

void AuditLogger::worker_loop() {
  Json::StreamWriterBuilder writer_builder;
  writer_builder["indentation"] = "";

  while (true) {
    QueuedEvent event;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return stop_requested_.load() || !event_queue_.empty();
      });

      if (event_queue_.empty()) {
        if (stop_requested_.load()) {
          return;
        }
        continue;
      }

      event = std::move(event_queue_.front());
      event_queue_.pop_front();
    }

    try {
      write_event(std::move(event));
      ++written_message_count_;
      written_cv_.notify_all();
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(logger_, "audit_logger write failed: %s", exception.what());
    }
  }
}

void AuditLogger::write_event(QueuedEvent event) {
  if (!event.serialized_message) {
    return;
  }

  Json::Value record(Json::objectValue);
  record["timestamp_ns"] = Json::Int64(event.received_time.nanoseconds());
  record["topic"] = event.topic_name;
  record["type"] = event.type_name;

  switch (event.stream_id) {
  case StreamId::kExecuteStatus:
    record["payload"] = goal_status_array_to_json(
        deserialize_message<action_msgs::msg::GoalStatusArray>(
            *event.serialized_message));
    break;

  case StreamId::kExecuteFeedback:
    record["payload"] = execute_motion_feedback_to_json(
        deserialize_message<ExecuteMotionFeedbackMessage>(
            *event.serialized_message));
    break;

  case StreamId::kValidateRequest:
    record["payload"] = validate_command_request_to_json(
        deserialize_message<ValidateCommandRequest>(*event.serialized_message));
    break;

  case StreamId::kValidateResponse:
    record["payload"] = validate_command_response_to_json(
        deserialize_message<ValidateCommandResponse>(
            *event.serialized_message));
    break;

  case StreamId::kHwReady:
    record["payload"] = robot_readiness_to_json(
        deserialize_message<interfaces::msg::RobotReadiness>(
            *event.serialized_message));
    break;
  }

  Json::StreamWriterBuilder writer_builder;
  writer_builder["indentation"] = "";
  jsonl_stream_ << Json::writeString(writer_builder, record) << '\n';
  jsonl_stream_.flush();

  writer_->write(event.serialized_message, event.topic_name, event.type_name,
                 event.received_time);
}
} // namespace supervisor
