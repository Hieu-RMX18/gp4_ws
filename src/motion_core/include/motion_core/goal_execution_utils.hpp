#pragma once

#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp_action/rclcpp_action.hpp>

#include "interfaces/action/execute_motion.hpp"
#include "motion_core/execution_orchestrator.hpp"
#include "motion_core/planning_scene_manager.hpp"

namespace motion_core {
using ExecuteMotion = interfaces::action::ExecuteMotion;
using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;

inline std::string make_interrupt_reason(const bool shutdown_requested,
                                         const bool goal_canceling,
                                         const bool stop_requested,
                                         const std::string &stage) {
  if (shutdown_requested) {
    return "node shutdown requested during " + stage;
  }

  if (goal_canceling) {
    return "goal canceled during " + stage;
  }

  if (stop_requested) {
    return "STOP requested during " + stage;
  }

  return {};
}

inline bool ensure_scene_ready(const bool require_planning_scene,
                               const PlanningSceneManager &scene_manager,
                               const std::string &scene_objects_path,
                               const SceneLoadResult scene_load_result,
                               std::string &reason) {
  reason.clear();
  if (!require_planning_scene) {
    return true;
  }

  if (scene_manager.is_scene_loaded()) {
    return true;
  }

  std::ostringstream stream;
  stream << "planning scene is required but not loaded";
  if (!scene_objects_path.empty()) {
    stream << " (path='" << scene_objects_path
           << "', status=" << scene_load_result_name(scene_load_result) << ")";
  } else {
    stream << " (scene_objects_path is empty)";
  }
  reason = stream.str();
  return false;
}

inline void
set_result_timing(const std::chrono::steady_clock::time_point &started_at,
                  ExecuteMotion::Result &result) {
  const auto ended_at = std::chrono::steady_clock::now();
  result.execution_time_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(ended_at -
                                                                started_at)
          .count();
}

inline void
publish_feedback(const std::shared_ptr<GoalHandleExecuteMotion> &goal_handle,
                 const double progress, const std::string &state) {
  auto feedback = std::make_shared<ExecuteMotion::Feedback>();
  feedback->progress = progress;
  feedback->current_state = state;
  goal_handle->publish_feedback(feedback);
}

inline void
abort_with_message(const std::shared_ptr<GoalHandleExecuteMotion> &goal_handle,
                   const std::chrono::steady_clock::time_point &started_at,
                   const std::string &message) {
  auto result = std::make_shared<ExecuteMotion::Result>();
  result->success = false;
  result->message = message;
  set_result_timing(started_at, *result);
  goal_handle->abort(result);
}

inline void
cancel_with_message(const std::shared_ptr<GoalHandleExecuteMotion> &goal_handle,
                    const std::chrono::steady_clock::time_point &started_at,
                    const std::string &message) {
  auto result = std::make_shared<ExecuteMotion::Result>();
  result->success = false;
  result->message = message;
  set_result_timing(started_at, *result);
  goal_handle->canceled(result);
}
} // namespace motion_core
