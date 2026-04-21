#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <interfaces/action/execute_motion.hpp>
#include <interfaces/action/dispatch_trajectory.hpp>
#include <interfaces/srv/get_current_pose.hpp>
#include <interfaces/srv/alarm_reset.hpp>
#include <interfaces/srv/io_set.hpp>

#include "motion_core/execute_motion_action_support.hpp"
#include "motion_core/goal_execution_utils.hpp"
#include "motion_core/ik_selector.hpp"
#include "motion_core/dispatch_trajectory_executor.hpp"
#include "motion_core/execution_orchestrator.hpp"
#include "motion_core/motion_primitive_executor.hpp"
#include "motion_core/non_motion_primitive_executor.hpp"
#include "motion_core/orientation_filter.hpp"
#include "motion_core/planner_router.hpp"
#include "motion_core/planning_scene_manager.hpp"
#include "motion_core/primitive_router_dispatch.hpp"
#include "motion_core/query_handler.hpp"
#include "motion_core/quality_gate.hpp"
#include "motion_core/seed_manager.hpp"
#include "motion_core/trajectory_post_processor.hpp"

namespace motion_core
{
class MotionCoreNode final : public rclcpp::Node
{
public:
  using ExecuteMotion = interfaces::action::ExecuteMotion;
  using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;
  using DispatchTrajectory = interfaces::action::DispatchTrajectory;
  using GetCurrentPose = interfaces::srv::GetCurrentPose;
  using AlarmReset = interfaces::srv::AlarmReset;
  using IoSet = interfaces::srv::IoSet;

  MotionCoreNode()
  : rclcpp::Node("motion_core_node"),
    seed_manager_(*this),
    quality_gate_(kTrajectoryHardLimitPoints, QualityGate::kMinimumCartesianFraction)
  {
    // V4 Contract: motion_core does NOT execute directly on hardware.
    // It sends validated trajectories to hw_adapter via DispatchTrajectory action.
    dispatch_action_name_ = declare_parameter<std::string>(
      "dispatch_action_name",
      "/hw_adapter/dispatch_trajectory");
    dispatch_timeout_sec_ = declare_parameter<double>(
      "dispatch_timeout_sec",
      60.0);
    dispatch_client_ = rclcpp_action::create_client<DispatchTrajectory>(
      this,
      dispatch_action_name_);
    action_support_ = std::make_unique<ExecuteMotionActionSupport>(
      get_logger(),
      kMaxVelocityScale,
      kMaxAccelerationScale);
    dispatch_executor_ = std::make_unique<DispatchTrajectoryExecutor>(
      get_logger(),
      dispatch_client_,
      dispatch_action_name_,
      dispatch_timeout_sec_,
      kTrajectorySafeBudgetPoints,
      quality_gate_,
      trajectory_post_processor_);

    query_handler_ = std::make_unique<QueryHandler>(
      get_logger(),
      [this](std::string & reason) { return ensure_move_group(reason); },
      [this](geometry_msgs::msg::PoseStamped & current_stamped, std::string & reason, double timeout_sec) {
        return read_current_tcp_pose(current_stamped, reason, timeout_sec);
      });
    primitive_router_dispatch_ = std::make_unique<PrimitiveRouterDispatch>(
      get_logger(),
      [this]() { return move_group_; },
      planner_router_,
      orientation_filter_,
      seed_manager_,
      ik_selector_,
      trajectory_post_processor_,
      [this](geometry_msgs::msg::PoseStamped & current_stamped, std::string & reason, double timeout_sec) {
        return read_current_tcp_pose(current_stamped, reason, timeout_sec);
      });
    // GET_POSE: dedicated query service — no motion, no planning, no execution.
    get_pose_service_ = create_service<GetCurrentPose>(
      "/get_current_pose",
      std::bind(
        &QueryHandler::handle_get_current_pose,
        query_handler_.get(),
        std::placeholders::_1,
        std::placeholders::_2));

    // Step 3.6: ALARM_RESET service client to hw_adapter
    // VERIFY_FROM_WORKSPACE before deployment: actual hw_adapter alarm service name
    alarm_reset_service_name_ = declare_parameter<std::string>(
      "alarm_reset_service_name",
      "/hw_adapter/alarm_reset");
    alarm_reset_client_ = create_client<AlarmReset>(alarm_reset_service_name_);

    // Step 3.7: IO_SET service client to hw_adapter
    // VERIFY_FROM_WORKSPACE before deployment: actual hw_adapter io service name
    io_set_service_name_ = declare_parameter<std::string>(
      "io_set_service_name",
      "/hw_adapter/io_set");
    io_set_client_ = create_client<IoSet>(io_set_service_name_);
    non_motion_executor_ = std::make_unique<NonMotionPrimitiveExecutor>(
      get_logger(),
      execution_orchestrator_,
      dispatch_client_,
      alarm_reset_client_,
      alarm_reset_service_name_,
      io_set_client_,
      io_set_service_name_,
      [this]()
      {
        if (move_group_)
        {
          move_group_->stop();
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "motion_core_node started (plan-only mode, execution via %s, query via /get_current_pose)",
      dispatch_action_name_.c_str());
  }

  ~MotionCoreNode() override
  {
    shutdown_requested_.store(true);
    if (action_support_)
    {
      action_support_->wait_for_workers();
    }
  }

  void initialize()
  {
    std::string reason;
    if (!ensure_move_group(reason)) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize MoveGroup in initialize(): %s", reason.c_str());
    }

    // Single-owner planning-scene policy:
    //   - required mode blocks planning until scene load succeeds
    //   - optional mode allows degraded planning in an empty scene
    require_planning_scene_ = declare_parameter<bool>("require_planning_scene", true);
    scene_objects_path_ = declare_parameter<std::string>("scene_objects_path", "");

    if (!scene_objects_path_.empty())
    {
      scene_load_result_ = scene_manager_.load_and_apply(scene_objects_path_);
      if (scene_load_result_ != SceneLoadResult::OK)
      {
        if (require_planning_scene_)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Planning scene load failed (%s) from '%s' and require_planning_scene=true; "
            "motion planning is fail-closed until a valid scene is loaded.",
            scene_load_result_name(scene_load_result_),
            scene_objects_path_.c_str());
        }
        else
        {
          RCLCPP_WARN(
            get_logger(),
            "Planning scene load failed (%s) from '%s' but require_planning_scene=false; "
            "planning will continue in degraded mode.",
            scene_load_result_name(scene_load_result_),
            scene_objects_path_.c_str());
        }
      }
    }
    else
    {
      scene_load_result_ = SceneLoadResult::SCENE_NOT_READY;
      if (require_planning_scene_)
      {
        RCLCPP_ERROR(
          get_logger(),
          "No scene_objects_path configured while require_planning_scene=true; "
          "planning is blocked (fail-closed).");
      }
      else
      {
        RCLCPP_WARN(
          get_logger(),
          "No scene_objects_path configured — planning scene empty (degraded mode allowed).");
      }
    }

    action_server_ = rclcpp_action::create_server<ExecuteMotion>(
      this,
      "execute_motion",
      std::bind(&MotionCoreNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionCoreNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MotionCoreNode::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "execute_motion action server ready");
  }

private:
  static constexpr double kMaxVelocityScale = 0.06;
  static constexpr double kMaxAccelerationScale = 0.06;
  static constexpr double kPlanningTimeSec = 5.0;
  static constexpr const char * kPlanningGroup = "gp4_arm";
  // Production point-budget policy.
  static constexpr std::size_t kTrajectorySafeBudgetPoints = 180;
  static constexpr std::size_t kTrajectoryHardLimitPoints = 200;

  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;
  // V4: NO FollowJointTrajectory client. Execution goes through hw_adapter only.
  rclcpp_action::Client<DispatchTrajectory>::SharedPtr dispatch_client_;
  // GET_POSE: state query service — completely separate from motion path.
  rclcpp::Service<GetCurrentPose>::SharedPtr get_pose_service_;
  std::unique_ptr<ExecuteMotionActionSupport> action_support_;
  std::unique_ptr<QueryHandler> query_handler_;
  std::unique_ptr<PrimitiveRouterDispatch> primitive_router_dispatch_;
  std::unique_ptr<NonMotionPrimitiveExecutor> non_motion_executor_;
  std::unique_ptr<DispatchTrajectoryExecutor> dispatch_executor_;
  // Step 3.6/3.7: service clients for ALARM_RESET and IO_SET (via hw_adapter)
  rclcpp::Client<AlarmReset>::SharedPtr alarm_reset_client_;
  rclcpp::Client<IoSet>::SharedPtr io_set_client_;

  PlannerRouter planner_router_;
  OrientationFilter orientation_filter_;
  SeedManager seed_manager_;
  IkSelector ik_selector_;
  TrajectoryPostProcessor trajectory_post_processor_;
  QualityGate quality_gate_;
  ExecutionOrchestrator execution_orchestrator_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  PlanningSceneManager scene_manager_{get_logger()};
  bool require_planning_scene_{true};
  SceneLoadResult scene_load_result_{SceneLoadResult::SCENE_NOT_READY};
  std::string scene_objects_path_;
  std::string dispatch_action_name_;
  double dispatch_timeout_sec_{60.0};
  std::string alarm_reset_service_name_;
  std::string io_set_service_name_;
  std::atomic<bool> shutdown_requested_{false};

  enum class StageStatus
  {
    kSuccess,
    kFailure,
    kCanceled,
  };

  struct StageResult
  {
    StageStatus status = StageStatus::kFailure;
    std::string reason;
    std::string note;
  };

  bool ensure_move_group(std::string & reason)
  {
    reason.clear();

    if (move_group_)
    {
      return true;
    }

    try
    {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        kPlanningGroup,
        std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(2.0));
      move_group_->setPlanningTime(kPlanningTimeSec);
      ik_selector_.set_move_group(move_group_);
      ik_selector_.set_planning_group(kPlanningGroup);
      RCLCPP_INFO(
        get_logger(),
        "MoveGroup initialized; current joint state will be resolved per-request from "
        "/yaskawa/joint_states after the executor starts spinning.");

      return true;
    }
    catch (const std::exception & ex)
    {
      reason = std::string("failed to initialize MoveGroupInterface: ") + ex.what();
      return false;
    }
  }

  bool build_current_robot_state(
    moveit::core::RobotState & current_state,
    std::string & reason,
    builtin_interfaces::msg::Time * source_joint_state_stamp = nullptr) const
  {
    reason.clear();

    if (!move_group_)
    {
      reason = "MoveGroup unavailable";
      return false;
    }

    std::vector<double> current_joint_positions;
    builtin_interfaces::msg::Time joint_state_stamp;
    if (!seed_manager_.get_current_joint_positions(current_joint_positions, joint_state_stamp))
    {
      reason = "latest /yaskawa/joint_states unavailable";
      return false;
    }
    if (source_joint_state_stamp)
    {
      *source_joint_state_stamp = joint_state_stamp;
    }

    current_state = moveit::core::RobotState(move_group_->getRobotModel());
    const auto * joint_model_group = current_state.getJointModelGroup(kPlanningGroup);
    if (!joint_model_group)
    {
      reason = "planning group '" + std::string(kPlanningGroup) + "' not found";
      return false;
    }

    const std::size_t expected_dof = joint_model_group->getVariableCount();
    if (current_joint_positions.size() != expected_dof)
    {
      std::ostringstream stream;
      stream << "latest /yaskawa/joint_states size mismatch for group '" << kPlanningGroup
             << "': expected " << expected_dof << ", got " << current_joint_positions.size();
      reason = stream.str();
      return false;
    }

    current_state.setJointGroupPositions(joint_model_group, current_joint_positions);
    current_state.update();
    return true;
  }

  bool read_current_tcp_pose(
    geometry_msgs::msg::PoseStamped & current_stamped,
    std::string & reason,
    const double /*timeout_sec*/ = 1.0)
  {
    reason.clear();

    if (!move_group_)
    {
      reason = "MoveGroup unavailable";
      return false;
    }

    moveit::core::RobotState current_state(move_group_->getRobotModel());
    if (!build_current_robot_state(current_state, reason))
    {
      return false;
    }

    const auto * joint_model_group = current_state.getJointModelGroup(kPlanningGroup);
    if (!joint_model_group)
    {
      reason = "planning group '" + std::string(kPlanningGroup) + "' not found";
      return false;
    }

    std::string tcp_link = move_group_->getEndEffectorLink();
    if (tcp_link.empty())
    {
      const auto & link_names = joint_model_group->getLinkModelNames();
      if (link_names.empty())
      {
        reason = "planning group '" + std::string(kPlanningGroup) + "' has no link models";
        return false;
      }
      tcp_link = link_names.back();
    }

    current_state.update();
    const Eigen::Isometry3d & tcp_transform = current_state.getGlobalLinkTransform(tcp_link);
    const Eigen::Vector3d translation = tcp_transform.translation();
    const Eigen::Quaterniond tcp_quaternion(tcp_transform.linear());

    if (!std::isfinite(translation.x()) || !std::isfinite(translation.y()) ||
      !std::isfinite(translation.z()))
    {
      reason = "current TCP pose has non-finite position for link '" + tcp_link + "'";
      return false;
    }

    const double qnorm_sq = tcp_quaternion.squaredNorm();
    if (!std::isfinite(qnorm_sq) || qnorm_sq <= 1e-12)
    {
      reason = "current TCP pose has invalid orientation for link '" + tcp_link + "'";
      return false;
    }

    const Eigen::Quaterniond normalized_q = tcp_quaternion.normalized();
    std::string pose_frame = move_group_->getPlanningFrame();
    if (pose_frame.empty())
    {
      pose_frame = move_group_->getRobotModel()->getModelFrame();
    }
    if (pose_frame.empty())
    {
      reason = "current TCP pose frame is unavailable";
      return false;
    }

    current_stamped.header.stamp = now();
    current_stamped.header.frame_id = pose_frame;
    current_stamped.pose.position.x = translation.x();
    current_stamped.pose.position.y = translation.y();
    current_stamped.pose.position.z = translation.z();
    current_stamped.pose.orientation.x = normalized_q.x();
    current_stamped.pose.orientation.y = normalized_q.y();
    current_stamped.pose.orientation.z = normalized_q.z();
    current_stamped.pose.orientation.w = normalized_q.w();
    return true;
  }

  StageResult plan_with_interruption(
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    const std::uint64_t sequence,
    const std::string & stage)
  {
    constexpr auto kPollPeriod = std::chrono::milliseconds(50);
    StageResult result;
    std::atomic<bool> planning_finished{false};
    moveit::core::MoveItErrorCode plan_code = moveit::core::MoveItErrorCode::FAILURE;
    std::exception_ptr planning_exception;

    std::thread planning_thread([this, &plan, &plan_code, &planning_finished, &planning_exception]() {
      try
      {
        plan_code = move_group_->plan(plan);
      }
      catch (...)
      {
        planning_exception = std::current_exception();
      }
      planning_finished.store(true);
    });

    std::string interrupted_reason;
    while (!planning_finished.load())
    {
      interrupted_reason = make_interrupt_reason(
        shutdown_requested_.load(),
        goal_handle->is_canceling(),
        execution_orchestrator_.stop_requested(sequence),
        stage);
      if (!interrupted_reason.empty())
      {
        move_group_->stop();
      }
      std::this_thread::sleep_for(kPollPeriod);
    }

    if (planning_thread.joinable())
    {
      planning_thread.join();
    }

    if (planning_exception)
    {
      try
      {
        std::rethrow_exception(planning_exception);
      }
      catch (const std::exception & ex)
      {
        result.reason = std::string("planning threw exception: ") + ex.what();
        return result;
      }
    }

    if (!interrupted_reason.empty())
    {
      result.status = StageStatus::kCanceled;
      result.reason = interrupted_reason;
      return result;
    }

    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      std::ostringstream oss;
      oss << "planning failed (MoveIt error code: " << plan_code.val
          << ", planner: " << move_group_->getPlannerId()
          << ", group: " << move_group_->getName() << ")";
      result.reason = oss.str();
      RCLCPP_ERROR(get_logger(), "plan() failed: %s", result.reason.c_str());
      return result;
    }

    result.status = StageStatus::kSuccess;
    return result;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteMotion::Goal> goal)
  {
    if (!action_support_)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return action_support_->handle_goal(goal);
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
  {
    if (!action_support_)
    {
      return rclcpp_action::CancelResponse::REJECT;
    }
    return action_support_->handle_cancel(goal_handle);
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
  {
    if (!action_support_)
    {
      return;
    }
    action_support_->handle_accepted(
      goal_handle,
      [this](const std::shared_ptr<GoalHandleExecuteMotion> & accepted_goal_handle)
      {
        execute(accepted_goal_handle);
      },
      shutdown_requested_);
  }

  void execute(const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle)
  {
    const auto started_at = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();
    const std::string goal_id =
      ExecuteMotionActionSupport::goal_uuid_to_string(goal_handle->get_goal_id());
    const std::string primitive =
      ExecuteMotionActionSupport::normalize_primitive(goal->primitive_type);

    RCLCPP_INFO(
      get_logger(),
      "execute_motion goal_id=%s primitive=%s accepted",
      goal_id.c_str(),
      primitive.c_str());

    publish_feedback(goal_handle, 0.05, "goal_accepted");

    if (goal->require_approval)
    {
      abort_with_message(
          goal_handle, started_at,
          "[DEFERRED] require_approval=true: human-in-the-loop approval gate "
          "is architecturally reserved but not implemented in Phase 4. "
          "Set require_approval=false or auto_clear_unimplemented_approval=true "
          "in the gateway launch config to use the current execution path.");
      return;
    }

    if (!ExecuteMotionActionSupport::is_supported_primitive(primitive))
    {
      abort_with_message(
        goal_handle,
        started_at,
        "unsupported primitive_type for Phase 4: " + goal->primitive_type);
      return;
    }

    // ── Step 3.3: STOP — top-priority, cancel everything immediately ──
    if (primitive == "STOP")
    {
      if (non_motion_executor_ &&
        non_motion_executor_->handle_stop_primitive(
          primitive,
          goal_id,
          goal_handle,
          started_at))
      {
        return;
      }
      abort_with_message(goal_handle, started_at, "STOP handler unavailable");
      return;
    }

    const auto start_result = execution_orchestrator_.begin_goal(primitive);
    if (!start_result.acquired)
    {
      const std::string reason = "orchestration reject: " + start_result.reason;
      RCLCPP_WARN(
        get_logger(),
        "execute_motion goal_id=%s primitive=%s %s",
        goal_id.c_str(),
        primitive.c_str(),
        reason.c_str());
      abort_with_message(goal_handle, started_at, reason);
      return;
    }
    const std::uint64_t goal_sequence = start_result.sequence;
    execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "goal accepted");
    struct FinishGuard
    {
      ExecutionOrchestrator & orchestrator;
      std::uint64_t sequence;
      ~FinishGuard()
      {
        orchestrator.finish_goal(sequence, "goal finished");
      }
    } finish_guard{execution_orchestrator_, goal_sequence};
    (void)finish_guard;

    RCLCPP_INFO(
      get_logger(),
      "execute_motion goal_seq=%lu goal_id=%s primitive=%s owner=motion_core_node",
      static_cast<unsigned long>(goal_sequence),
      goal_id.c_str(),
      primitive.c_str());

    const std::string initial_interrupt = make_interrupt_reason(
      shutdown_requested_.load(),
      goal_handle->is_canceling(),
      execution_orchestrator_.stop_requested(goal_sequence),
      "goal_start");
    if (!initial_interrupt.empty())
    {
      cancel_with_message(goal_handle, started_at, initial_interrupt);
      return;
    }

    if (ExecuteMotionActionSupport::is_non_motion_primitive(primitive))
    {
      if (non_motion_executor_ &&
        non_motion_executor_->handle_non_motion_primitive(
          primitive,
          goal_sequence,
          goal,
          goal_handle,
          started_at,
          [this, goal_handle](const double progress, const std::string & state)
          {
            publish_feedback(goal_handle, progress, state);
          },
          [this, goal_handle, started_at](const std::string & message)
          {
            abort_with_message(goal_handle, started_at, message);
          },
          [this, goal_handle, started_at](const std::string & message)
          {
            cancel_with_message(goal_handle, started_at, message);
          }))
      {
        return;
      }
      abort_with_message(goal_handle, started_at, "non-motion primitive handler unavailable");
      return;
    }

    if (!dispatch_executor_ || !primitive_router_dispatch_)
    {
      abort_with_message(goal_handle, started_at, "motion primitive executor dependencies unavailable");
      return;
    }

    MotionPrimitiveExecutor motion_executor({
      *primitive_router_dispatch_,
      *dispatch_executor_,
      execution_orchestrator_,
      kPlanningGroup,
      [this](std::string & reason) -> bool
      {
        return ensure_move_group(reason);
      },
      [this](std::string & reason) -> bool
      {
        return motion_core::ensure_scene_ready(
          require_planning_scene_,
          scene_manager_,
          scene_objects_path_,
          scene_load_result_,
          reason);
      },
      [this]() -> std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      {
        return move_group_;
      },
      [this](
        moveit::core::RobotState & current_state,
        std::string & reason,
        builtin_interfaces::msg::Time * source_joint_state_stamp) -> bool
      {
        return build_current_robot_state(current_state, reason, source_joint_state_stamp);
      },
      [this, goal_handle, goal_sequence](
        moveit::planning_interface::MoveGroupInterface::Plan & plan,
        const std::string & stage) -> PrimitiveRouterDispatch::PlanningStageResult
      {
        const auto stage_result = plan_with_interruption(plan, goal_handle, goal_sequence, stage);
        PrimitiveRouterDispatch::PlanningStageResult converted;
        switch (stage_result.status)
        {
          case StageStatus::kSuccess:
            converted.status = PrimitiveRouterDispatch::PlanningStatus::kSuccess;
            break;
          case StageStatus::kCanceled:
            converted.status = PrimitiveRouterDispatch::PlanningStatus::kCanceled;
            break;
          case StageStatus::kFailure:
          default:
            converted.status = PrimitiveRouterDispatch::PlanningStatus::kFailure;
            break;
        }
        converted.reason = stage_result.reason;
        return converted;
      },
      [this, goal_handle, goal_sequence](const std::string & stage) -> std::string
      {
        return make_interrupt_reason(
          shutdown_requested_.load(),
          goal_handle->is_canceling(),
          execution_orchestrator_.stop_requested(goal_sequence),
          stage);
      },
      [this, goal_handle](const double progress, const std::string & state)
      {
        publish_feedback(goal_handle, progress, state);
      },
      [this, goal_sequence](const ExecutionPhase phase, const std::string & detail)
      {
        execution_orchestrator_.update_phase(goal_sequence, phase, detail);
      },
    });

    const auto motion_result =
      motion_executor.execute(goal, goal_id, primitive, goal_sequence);
    switch (motion_result.status)
    {
      case MotionPrimitiveExecutor::Status::kCanceled:
        cancel_with_message(goal_handle, started_at, motion_result.message);
        return;
      case MotionPrimitiveExecutor::Status::kAborted:
        abort_with_message(goal_handle, started_at, motion_result.message);
        return;
      case MotionPrimitiveExecutor::Status::kSucceeded:
      default:
        break;
    }

    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = true;
    result->message = motion_result.message;
    set_result_timing(started_at, *result);
    goal_handle->succeed(result);
  }

};
}  // namespace motion_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motion_core::MotionCoreNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
