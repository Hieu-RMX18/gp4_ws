#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "interfaces/action/execute_motion.hpp"

namespace motion_core
{
class ExecuteMotionActionSupport
{
public:
  using ExecuteMotion = interfaces::action::ExecuteMotion;
  using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;
  using ExecuteFn = std::function<void(const std::shared_ptr<GoalHandleExecuteMotion> &)>;

  ExecuteMotionActionSupport(
    rclcpp::Logger logger,
    double max_velocity_scale,
    double max_acceleration_scale);

  void cleanup_finished_workers();
  void wait_for_workers();

  rclcpp_action::GoalResponse handle_goal(
    const std::shared_ptr<const ExecuteMotion::Goal> & goal) const;

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle) const;

  void handle_accepted(
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    const ExecuteFn & execute_fn,
    const std::atomic<bool> & shutdown_requested);

  static std::string normalize_primitive(std::string primitive);
  static bool is_supported_primitive(const std::string & primitive);
  static bool is_non_motion_primitive(const std::string & primitive);
  static std::string approval_rejected_message();
  static std::string goal_uuid_to_string(const rclcpp_action::GoalUUID & goal_id);

private:
  rclcpp::Logger logger_;
  double max_velocity_scale_;
  double max_acceleration_scale_;
  mutable std::mutex worker_mutex_;
  std::vector<std::future<void>> worker_futures_;
};
}  // namespace motion_core
