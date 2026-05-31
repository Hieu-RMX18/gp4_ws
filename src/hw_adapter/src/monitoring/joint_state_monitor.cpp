// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "hw_adapter/joint_state_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace hw_adapter {
JointStateMonitor::JointStateMonitor(
    rclcpp::Node &node, std::vector<std::string> expected_joint_names,
    std::string topic_name, std::chrono::milliseconds max_age)
    : logger_(node.get_logger()), clock_(node.get_clock()),
      expected_joint_names_(std::move(expected_joint_names)),
      max_age_(max_age) {
  if (max_age_.count() <= 0) {
    max_age_ = std::chrono::milliseconds(200);
  }

  joint_state_sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
      std::move(topic_name), rclcpp::SensorDataQoS(),
      std::bind(&JointStateMonitor::joint_state_callback, this,
                std::placeholders::_1));
}

JointStateSnapshot JointStateMonitor::latest_snapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  JointStateSnapshot snapshot = snapshot_;
  if (!has_message_) {
    snapshot.has_message = false;
    snapshot.fresh = false;
    snapshot.valid = false;
    snapshot.age = std::chrono::milliseconds::max();
    snapshot.status_message = "unknown: no joint state received";
    return snapshot;
  }

  snapshot.has_message = true;
  const auto age_ns = (clock_->now() - receive_time_).nanoseconds();
  const auto non_negative_age_ns = age_ns < 0 ? 0 : age_ns;
  snapshot.age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::nanoseconds(non_negative_age_ns));
  snapshot.fresh = snapshot.age <= max_age_;
  if (!snapshot.fresh) {
    std::ostringstream stream;
    stream << snapshot.status_message
           << " (stale joint state: age=" << snapshot.age.count()
           << " ms exceeds max_age=" << max_age_.count() << " ms)";
    snapshot.status_message = stream.str();
    snapshot.valid = false;
  }

  return snapshot;
}

void JointStateMonitor::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg) {
    RCLCPP_WARN(logger_, "Received null JointState message.");
    return;
  }

  JointStateSnapshot next_snapshot;
  next_snapshot.has_message = true;
  next_snapshot.header_stamp = msg->header.stamp;

  const auto &expected_joint_names =
      expected_joint_names_.empty() ? msg->name : expected_joint_names_;
  next_snapshot.ordered_positions.reserve(expected_joint_names.size());

  if (msg->name.empty()) {
    next_snapshot.status_message =
        "invalid joint state: message contains no joint names";
  } else if (msg->position.size() < msg->name.size()) {
    std::ostringstream stream;
    stream << "invalid joint state: position count " << msg->position.size()
           << " is smaller than joint name count " << msg->name.size();
    next_snapshot.status_message = stream.str();
  } else {
    bool mapping_ok = true;
    for (const auto &expected_name : expected_joint_names) {
      const auto it =
          std::find(msg->name.begin(), msg->name.end(), expected_name);
      if (it == msg->name.end()) {
        next_snapshot.status_message =
            "invalid joint state: missing required joint '" + expected_name +
            "'";
        mapping_ok = false;
        break;
      }

      const auto index =
          static_cast<std::size_t>(std::distance(msg->name.begin(), it));
      const double position = msg->position.at(index);
      if (!std::isfinite(position)) {
        next_snapshot.status_message =
            "invalid joint state: non-finite position for joint '" +
            expected_name + "'";
        mapping_ok = false;
        break;
      }
      next_snapshot.ordered_positions.push_back(position);
    }

    if (mapping_ok) {
      next_snapshot.valid = true;
      next_snapshot.fresh = true;
      next_snapshot.status_message = "joint state ready";
    }
  }

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_ = std::move(next_snapshot);
    has_message_ = true;
    receive_time_ = clock_->now();
  }
}

} // namespace hw_adapter
