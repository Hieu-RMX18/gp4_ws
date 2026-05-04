// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#if __has_include(<motoros2_interfaces/srv/reset_error.hpp>) && \
  __has_include(<motoros2_interfaces/srv/start_traj_mode.hpp>)
#include <motoros2_interfaces/srv/reset_error.hpp>
#include <motoros2_interfaces/srv/start_traj_mode.hpp>
#define HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES 1
#else
#define HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES 0
#endif

#if __has_include(<industrial_msgs/srv/stop_motion.hpp>)
#include <industrial_msgs/srv/stop_motion.hpp>
#define HW_ADAPTER_HAS_STOP_MOTION_SERVICE 1
#else
#define HW_ADAPTER_HAS_STOP_MOTION_SERVICE 0
#endif

namespace hw_adapter {
struct SessionServiceNames {
  std::string start_traj_mode = "/yaskawa/start_traj_mode";
  std::string reset_error = "/yaskawa/reset_error";
  std::string stop_motion;
  std::string follow_joint_trajectory_action =
      "/yaskawa/follow_joint_trajectory";
};

struct SessionManagerSnapshot {
  bool motoros2_interfaces_available = false;
  bool start_traj_mode_configured = false;
  bool reset_error_configured = false;
  bool stop_motion_configured = false;
  bool stop_motion_uses_action_cancel = false;
  bool session_ready = false;
  std::string status_message = "MotoROS2 session services not initialized";
};

class Motoros2SessionManager {
public:
  explicit Motoros2SessionManager(
      rclcpp::Node &node, SessionServiceNames service_names = {},
      std::chrono::milliseconds operation_timeout = std::chrono::seconds(2));

  SessionManagerSnapshot snapshot() const;
  bool wait_for_required_services(std::chrono::milliseconds timeout,
                                  std::string &reason) const;
  bool start_traj_mode(std::string &reason);
  bool ensure_trajectory_mode(std::string &reason);
  bool reset_error(std::string &reason);
  bool stop_motion(std::string &reason);
  bool is_session_ready() const;

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using FollowJointTrajectoryClient =
      rclcpp_action::Client<FollowJointTrajectory>;
#if HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES
  using StartTrajMode = motoros2_interfaces::srv::StartTrajMode;
  using ResetError = motoros2_interfaces::srv::ResetError;
#endif
#if HW_ADAPTER_HAS_STOP_MOTION_SERVICE
  using StopMotion = industrial_msgs::srv::StopMotion;
#endif

  void update_status_message(const std::string &message);
  void set_session_ready(bool ready, const std::string &message);

  rclcpp::Logger logger_;
  std::shared_ptr<rclcpp::Node> client_node_;
  SessionServiceNames service_names_;
  std::chrono::milliseconds operation_timeout_;

  mutable std::mutex call_mutex_;
  mutable std::mutex snapshot_mutex_;
  SessionManagerSnapshot snapshot_;

#if HW_ADAPTER_HAS_MOTOROS2_SESSION_SERVICES
  mutable rclcpp::Client<StartTrajMode>::SharedPtr start_traj_mode_client_;
  mutable rclcpp::Client<ResetError>::SharedPtr reset_error_client_;
#endif
#if HW_ADAPTER_HAS_STOP_MOTION_SERVICE
  mutable rclcpp::Client<StopMotion>::SharedPtr stop_motion_client_;
#endif
  mutable FollowJointTrajectoryClient::SharedPtr
      follow_joint_trajectory_client_;
};
} // namespace hw_adapter
