#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include "interfaces/action/execute_motion.hpp"
#include "motion_core/dispatch_trajectory_executor.hpp"
#include "motion_core/execution_orchestrator.hpp"
#include "motion_core/primitive_router_dispatch.hpp"

namespace motion_core {
class MotionPrimitiveExecutor {
public:
  using ExecuteMotion = interfaces::action::ExecuteMotion;

  enum class Status {
    kSucceeded,
    kAborted,
    kCanceled,
  };

  struct Result {
    Status status = Status::kAborted;
    std::string message;
  };

  struct Dependencies {
    PrimitiveRouterDispatch &primitive_router_dispatch;
    DispatchTrajectoryExecutor &dispatch_executor;
    ExecutionOrchestrator &execution_orchestrator;
    std::string planning_group;
    std::function<bool(std::string &)> ensure_move_group;
    std::function<bool(std::string &)> ensure_scene_ready;
    std::function<
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface>()>
        move_group_provider;
    std::function<bool(moveit::core::RobotState &, std::string &,
                       builtin_interfaces::msg::Time *)>
        build_current_robot_state;
    std::function<PrimitiveRouterDispatch::PlanningStageResult(
        moveit::planning_interface::MoveGroupInterface::Plan &,
        const std::string &)>
        plan_with_interruption;
    std::function<std::string(const std::string &)> interrupt_reason;
    std::function<void(double, const std::string &)> publish_feedback;
    std::function<void(ExecutionPhase, const std::string &)> update_phase;
  };

  explicit MotionPrimitiveExecutor(Dependencies dependencies);

  Result execute(const std::shared_ptr<const ExecuteMotion::Goal> &goal,
                 const std::string &goal_id, const std::string &primitive,
                 std::uint64_t goal_sequence) const;

private:
  Dependencies dependencies_;
};
} // namespace motion_core
