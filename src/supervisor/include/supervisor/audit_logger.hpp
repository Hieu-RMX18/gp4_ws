// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>

namespace supervisor {
class AuditLogger {
public:
  explicit AuditLogger(rclcpp::Node &node);
  ~AuditLogger();

  AuditLogger(const AuditLogger &) = delete;
  AuditLogger &operator=(const AuditLogger &) = delete;

  std::string bag_uri() const;
  std::string jsonl_path() const;
  uint64_t max_callback_latency_ns() const;
  std::size_t received_message_count() const;
  std::size_t written_message_count() const;
  bool wait_for_written_messages(std::size_t min_count,
                                 std::chrono::milliseconds timeout);

private:
  enum class StreamId : uint8_t {
    kExecuteStatus,
    kExecuteFeedback,
    kValidateRequest,
    kValidateResponse,
    kHwReady,
  };

  struct TopicSpec {
    StreamId stream_id;
    std::string topic_name;
    std::string type_name;
    rclcpp::QoS qos;
  };

  struct QueuedEvent {
    StreamId stream_id;
    std::string topic_name;
    std::string type_name;
    rclcpp::Time received_time;
    std::shared_ptr<rclcpp::SerializedMessage> serialized_message;
  };

  void initialize_output_paths();
  void initialize_writer();
  void initialize_subscriptions();
  void start_worker();
  void stop_worker();
  void enqueue_message(
      const TopicSpec &spec,
      std::shared_ptr<rclcpp::SerializedMessage> serialized_message);
  void worker_loop();
  void write_event(QueuedEvent event);

  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  std::string serialization_format_;
  std::filesystem::path audit_path_;
  std::filesystem::path bag_uri_;
  std::filesystem::path jsonl_path_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::ofstream jsonl_stream_;
  std::vector<TopicSpec> topic_specs_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::deque<QueuedEvent> event_queue_;
  mutable std::mutex queue_mutex_;
  mutable std::mutex path_mutex_;
  std::condition_variable queue_cv_;
  std::condition_variable written_cv_;
  std::thread worker_thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<uint64_t> max_callback_latency_ns_{0U};
  std::atomic<std::size_t> received_message_count_{0U};
  std::atomic<std::size_t> written_message_count_{0U};
  std::size_t max_queue_depth_{2048U};
};
} // namespace supervisor
