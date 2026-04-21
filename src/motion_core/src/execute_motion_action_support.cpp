#include "motion_core/execute_motion_action_support.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <exception>
#include <future>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace motion_core
{
ExecuteMotionActionSupport::ExecuteMotionActionSupport(
  rclcpp::Logger logger,
  const double max_velocity_scale,
  const double max_acceleration_scale)
: logger_(std::move(logger)),
  max_velocity_scale_(max_velocity_scale),
  max_acceleration_scale_(max_acceleration_scale)
{
}

void ExecuteMotionActionSupport::cleanup_finished_workers()
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  auto it = worker_futures_.begin();
  while (it != worker_futures_.end())
  {
    if (it->valid() && it->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      try
      {
        it->get();
      }
      catch (const std::exception & ex)
      {
        RCLCPP_ERROR(logger_, "execute_motion worker ended with exception: %s", ex.what());
      }
      catch (...)
      {
        RCLCPP_ERROR(logger_, "execute_motion worker ended with unknown exception.");
      }
      it = worker_futures_.erase(it);
      continue;
    }
    ++it;
  }
}

void ExecuteMotionActionSupport::wait_for_workers()
{
  std::vector<std::future<void>> workers;
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    workers.swap(worker_futures_);
  }
  for (auto & worker : workers)
  {
    if (!worker.valid())
    {
      continue;
    }
    try
    {
      worker.get();
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(logger_, "execute_motion worker join failed: %s", ex.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(logger_, "execute_motion worker join failed with unknown exception.");
    }
  }
}

rclcpp_action::GoalResponse ExecuteMotionActionSupport::handle_goal(
  const std::shared_ptr<const ExecuteMotion::Goal> & goal) const
{
  if (!goal)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  const std::string primitive = normalize_primitive(goal->primitive_type);
  if (is_non_motion_primitive(primitive))
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  if (goal->velocity_scale < 0.0 || goal->acceleration_scale < 0.0)
  {
    RCLCPP_WARN(logger_, "Rejecting goal: negative scaling is not allowed.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->velocity_scale > max_velocity_scale_)
  {
    RCLCPP_WARN(
      logger_,
      "Rejecting goal: velocity_scale %.3f exceeds %.3f.",
      goal->velocity_scale,
      max_velocity_scale_);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->acceleration_scale > max_acceleration_scale_)
  {
    RCLCPP_WARN(
      logger_,
      "Rejecting goal: acceleration_scale %.3f exceeds %.3f.",
      goal->acceleration_scale,
      max_acceleration_scale_);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecuteMotionActionSupport::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle) const
{
  RCLCPP_WARN(
    logger_,
    "Cancel requested for execute_motion goal_id=%s",
    goal_uuid_to_string(goal_handle->get_goal_id()).c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecuteMotionActionSupport::handle_accepted(
  const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
  const ExecuteFn & execute_fn,
  const std::atomic<bool> & shutdown_requested)
{
  cleanup_finished_workers();
  std::future<void> worker = std::async(
    std::launch::async,
    [goal_handle, execute_fn, &shutdown_requested]()
    {
      if (shutdown_requested.load())
      {
        return;
      }
      execute_fn(goal_handle);
    });
  std::lock_guard<std::mutex> lock(worker_mutex_);
  worker_futures_.emplace_back(std::move(worker));
}

std::string ExecuteMotionActionSupport::normalize_primitive(std::string primitive)
{
  primitive.erase(
    std::remove_if(primitive.begin(), primitive.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }),
    primitive.end());

  std::transform(
    primitive.begin(), primitive.end(), primitive.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

  return primitive;
}

bool ExecuteMotionActionSupport::is_supported_primitive(const std::string & primitive)
{
  return primitive == "HOME" || primitive == "PTP" || primitive == "LIN" ||
         primitive == "CIRC" || primitive == "MOVE_REL" || primitive == "CARTESIAN_PATH" ||
         primitive == "SET_SPEED" || primitive == "WAIT" || primitive == "STOP" ||
         primitive == "MOVE_JOINT" || primitive == "MOVE_JOINTS" ||
         primitive == "IO_SET" || primitive == "ALARM_RESET";
}

bool ExecuteMotionActionSupport::is_non_motion_primitive(const std::string & primitive)
{
  return primitive == "ALARM_RESET" || primitive == "STOP" ||
         primitive == "WAIT" || primitive == "IO_SET" ||
         primitive == "SET_SPEED";
}

std::string ExecuteMotionActionSupport::goal_uuid_to_string(const rclcpp_action::GoalUUID & goal_id)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : goal_id)
  {
    stream << std::setw(2) << static_cast<int>(byte);
  }
  return stream.str();
}
}  // namespace motion_core
