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

#include "motion_core/ik_selector.hpp"
#include "motion_core/dispatch_trajectory_executor.hpp"
#include "motion_core/execution_orchestrator.hpp"
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

    RCLCPP_INFO(
      get_logger(),
      "motion_core_node started (plan-only mode, execution via %s, query via /get_current_pose)",
      dispatch_action_name_.c_str());
  }

  ~MotionCoreNode() override
  {
    shutdown_requested_.store(true);
    wait_for_workers();
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
  // Standard Cartesian eef_step — tighter fidelity for CARTESIAN_PATH.
  static constexpr double kCartesianEefStep = 0.005;
  // Relaxed eef_step for LIN/CIRC fallback paths to reduce point density.
  static constexpr double kCartesianEefStepRelaxed = 0.010;
  // V4 G0: jump_threshold must be >= 1.5 in Cartesian planning config.
  static constexpr double kCartesianJumpThreshold = 1.5;
  static constexpr const char * kPlanningGroup = "gp4_arm";
  // Production point-budget policy.
  static constexpr std::size_t kTrajectorySafeBudgetPoints = 180;
  static constexpr std::size_t kTrajectoryHardLimitPoints = 200;

  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;
  // V4: NO FollowJointTrajectory client. Execution goes through hw_adapter only.
  rclcpp_action::Client<DispatchTrajectory>::SharedPtr dispatch_client_;
  // GET_POSE: state query service — completely separate from motion path.
  rclcpp::Service<GetCurrentPose>::SharedPtr get_pose_service_;
  std::unique_ptr<QueryHandler> query_handler_;
  std::unique_ptr<PrimitiveRouterDispatch> primitive_router_dispatch_;
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
  std::mutex worker_mutex_;
  std::vector<std::future<void>> worker_futures_;

  void cleanup_finished_workers()
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
          RCLCPP_ERROR(get_logger(), "execute_motion worker ended with exception: %s", ex.what());
        }
        catch (...)
        {
          RCLCPP_ERROR(get_logger(), "execute_motion worker ended with unknown exception.");
        }
        it = worker_futures_.erase(it);
        continue;
      }
      ++it;
    }
  }

  void wait_for_workers()
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
        RCLCPP_ERROR(get_logger(), "execute_motion worker join failed: %s", ex.what());
      }
      catch (...)
      {
        RCLCPP_ERROR(get_logger(), "execute_motion worker join failed with unknown exception.");
      }
    }
  }

  static std::string normalize_primitive(std::string primitive)
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

  static bool is_supported_primitive(const std::string & primitive)
  {
    // Supported primitives wired through motion_core -> hw_adapter.
    return primitive == "HOME" || primitive == "PTP" || primitive == "LIN" ||
           primitive == "CIRC" ||
           primitive == "MOVE_REL" || primitive == "CARTESIAN_PATH" ||
           primitive == "SET_SPEED" || primitive == "WAIT" || primitive == "STOP" ||
           primitive == "MOVE_JOINT" || primitive == "MOVE_JOINTS" ||
           primitive == "IO_SET" || primitive == "ALARM_RESET";
  }

  /// Step 3.2: Non-motion primitives that do not require velocity/acceleration checks.
  /// These are utility/query/control commands — not motion planning commands.
  static bool is_non_motion_primitive(const std::string & primitive)
  {
    return primitive == "ALARM_RESET" || primitive == "STOP" ||
           primitive == "WAIT" || primitive == "IO_SET" ||
           primitive == "SET_SPEED";
  }

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

  static std::string goal_uuid_to_string(const rclcpp_action::GoalUUID & goal_id)
  {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte : goal_id)
    {
      stream << std::setw(2) << static_cast<int>(byte);
    }
    return stream.str();
  }

  std::string interrupt_reason(
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    const std::uint64_t sequence,
    const std::string & stage) const
  {
    if (shutdown_requested_.load())
    {
      return "node shutdown requested during " + stage;
    }

    if (goal_handle->is_canceling())
    {
      return "goal canceled during " + stage;
    }

    if (execution_orchestrator_.stop_requested(sequence))
    {
      return "STOP requested during " + stage;
    }

    return {};
  }

  bool ensure_scene_ready(std::string & reason) const
  {
    reason.clear();
    if (!require_planning_scene_)
    {
      return true;
    }

    if (scene_manager_.is_scene_loaded())
    {
      return true;
    }

    std::ostringstream stream;
    stream << "planning scene is required but not loaded";
    if (!scene_objects_path_.empty())
    {
      stream << " (path='" << scene_objects_path_ << "', status="
             << scene_load_result_name(scene_load_result_) << ")";
    }
    else
    {
      stream << " (scene_objects_path is empty)";
    }
    reason = stream.str();
    return false;
  }

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

  static void set_result_timing(
    const std::chrono::steady_clock::time_point & started_at,
    ExecuteMotion::Result & result)
  {
    const auto ended_at = std::chrono::steady_clock::now();
    result.execution_time_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(ended_at - started_at).count();
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

  void publish_feedback(
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    double progress,
    const std::string & state) const
  {
    auto feedback = std::make_shared<ExecuteMotion::Feedback>();
    feedback->progress = progress;
    feedback->current_state = state;
    goal_handle->publish_feedback(feedback);
  }

  void abort_with_message(
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    const std::chrono::steady_clock::time_point & started_at,
    const std::string & message) const
  {
    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = false;
    result->message = message;
    set_result_timing(started_at, *result);
    goal_handle->abort(result);
  }

  void cancel_with_message(
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    const std::chrono::steady_clock::time_point & started_at,
    const std::string & message) const
  {
    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = false;
    result->message = message;
    set_result_timing(started_at, *result);
    goal_handle->canceled(result);
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
      interrupted_reason = interrupt_reason(goal_handle, sequence, stage);
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
    if (!goal)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Step 3.2: Non-motion primitives bypass velocity/acceleration checks.
    // They don't plan or execute trajectories, so scaling values are irrelevant.
    const std::string primitive = normalize_primitive(goal->primitive_type);
    if (is_non_motion_primitive(primitive))
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    if (goal->velocity_scale < 0.0 || goal->acceleration_scale < 0.0)
    {
      RCLCPP_WARN(get_logger(), "Rejecting goal: negative scaling is not allowed.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->velocity_scale > kMaxVelocityScale)
    {
      RCLCPP_WARN(
        get_logger(), "Rejecting goal: velocity_scale %.3f exceeds %.3f.", goal->velocity_scale,
        kMaxVelocityScale);
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->acceleration_scale > kMaxAccelerationScale)
    {
      RCLCPP_WARN(
        get_logger(), "Rejecting goal: acceleration_scale %.3f exceeds %.3f.",
        goal->acceleration_scale, kMaxAccelerationScale);
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
  {
    RCLCPP_WARN(
      get_logger(),
      "Cancel requested for execute_motion goal_id=%s",
      goal_uuid_to_string(goal_handle->get_goal_id()).c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
  {
    cleanup_finished_workers();
    std::future<void> worker = std::async(
      std::launch::async,
      [this, goal_handle]()
      {
        if (shutdown_requested_.load())
        {
          return;
        }
        execute(goal_handle);
      });
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_futures_.emplace_back(std::move(worker));
  }

  void execute(const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle)
  {
    const auto started_at = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();
    const std::string goal_id = goal_uuid_to_string(goal_handle->get_goal_id());
    const std::string primitive = normalize_primitive(goal->primitive_type);

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

    if (!is_supported_primitive(primitive))
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
      std::string stop_reason;
      const bool had_active_goal = execution_orchestrator_.request_stop(stop_reason);
      RCLCPP_WARN(
        get_logger(),
        "STOP primitive received for goal_id=%s — %s",
        goal_id.c_str(),
        had_active_goal ? stop_reason.c_str() : "no active execute_motion goal to stop");
      // Cancel any in-flight dispatch goals
      if (dispatch_client_)
      {
        dispatch_client_->async_cancel_all_goals();
      }
      // Stop MoveGroup planning/execution if available
      if (move_group_)
      {
        move_group_->stop();
      }
      auto result = std::make_shared<ExecuteMotion::Result>();
      result->success = true;
      result->message = had_active_goal ?
        ("STOP: motion halt requested, dispatch cancel issued (" + stop_reason + ")") :
        "STOP: no active execute_motion goal was running";
      set_result_timing(started_at, *result);
      goal_handle->succeed(result);
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

    const std::string initial_interrupt = interrupt_reason(goal_handle, goal_sequence, "goal_start");
    if (!initial_interrupt.empty())
    {
      cancel_with_message(goal_handle, started_at, initial_interrupt);
      return;
    }

    // ── Step 3.4: SET_SPEED — stateless acknowledge-only ──
    // Does NOT persist velocity across future goals. Each motion command must
    // include its own velocity_scale. This is a convenience primitive for the
    // LLM gateway to acknowledge speed-related user intent without side effects.
    if (primitive == "SET_SPEED")
    {
      execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "set_speed");
      const double requested_scale = goal->velocity_scale;
      auto result = std::make_shared<ExecuteMotion::Result>();
      result->success = true;
      std::ostringstream msg;
      msg << "SET_SPEED acknowledged: goal_seq=" << goal_sequence
          << ", velocity_scale=" << requested_scale
          << ". NOTE: this is stateless — subsequent motion commands must "
             "include their own velocity_scale field to take effect.";
      result->message = msg.str();
      set_result_timing(started_at, *result);
      RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
      goal_handle->succeed(result);
      return;
    }

    // ── Step 3.5: WAIT — cancellation-aware timed pause ──
    // execute() runs in an owned async worker launched by handle_accepted(),
    // so blocking sleep is safe here and lifecycle remains bounded.
    if (primitive == "WAIT")
    {
      execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "wait");
      const double wait_sec = std::max(goal->wait_duration_sec, 0.0);
      RCLCPP_INFO(
        get_logger(),
        "WAIT goal_seq=%lu: pausing for %.3f seconds.",
        static_cast<unsigned long>(goal_sequence),
        wait_sec);
      publish_feedback(goal_handle, 0.1, "wait_started");

      const auto wait_start = std::chrono::steady_clock::now();
      const auto wait_duration = std::chrono::duration<double>(wait_sec);
      constexpr auto poll_interval = std::chrono::milliseconds(50);

      while (true)
      {
        const auto elapsed = std::chrono::steady_clock::now() - wait_start;
        if (elapsed >= wait_duration)
        {
          break;
        }
        if (goal_handle->is_canceling())
        {
          cancel_with_message(goal_handle, started_at, "WAIT: cancelled during wait");
          return;
        }
        if (execution_orchestrator_.stop_requested(goal_sequence))
        {
          cancel_with_message(goal_handle, started_at, "WAIT: STOP requested during wait");
          return;
        }
        const double progress = std::min(
          0.1 + 0.8 * (std::chrono::duration<double>(elapsed).count() / wait_sec),
          0.9);
        publish_feedback(goal_handle, progress, "waiting");
        std::this_thread::sleep_for(poll_interval);
      }

      auto result = std::make_shared<ExecuteMotion::Result>();
      result->success = true;
      result->message = "WAIT: completed " + std::to_string(wait_sec) + " seconds";
      set_result_timing(started_at, *result);
      goal_handle->succeed(result);
      return;
    }

    // ── Step 3.6: ALARM_RESET — delegate to hw_adapter via service ──
    if (primitive == "ALARM_RESET")
    {
      execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "alarm_reset");
      RCLCPP_INFO(
        get_logger(),
        "ALARM_RESET goal_seq=%lu: sending reset request to %s",
        static_cast<unsigned long>(goal_sequence),
        alarm_reset_service_name_.c_str());

      if (!alarm_reset_client_->wait_for_service(std::chrono::seconds(5)))
      {
        abort_with_message(goal_handle, started_at,
          "ALARM_RESET: service unavailable at " + alarm_reset_service_name_);
        return;
      }

      auto request = std::make_shared<AlarmReset::Request>();
      auto future = alarm_reset_client_->async_send_request(request);

      if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
      {
        abort_with_message(goal_handle, started_at,
          "ALARM_RESET: timed out waiting for response from " + alarm_reset_service_name_);
        return;
      }

      auto response = future.get();
      auto result = std::make_shared<ExecuteMotion::Result>();
      result->success = response->success;
      result->message = response->success
        ? "ALARM_RESET: " + response->message
        : "ALARM_RESET failed: " + response->message;
      set_result_timing(started_at, *result);

      if (response->success)
      {
        goal_handle->succeed(result);
      }
      else
      {
        goal_handle->abort(result);
      }
      return;
    }

    // ── Step 3.7: IO_SET — delegate to hw_adapter via service ──
    if (primitive == "IO_SET")
    {
      execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kAccepted, "io_set");
      RCLCPP_INFO(
        get_logger(),
        "IO_SET goal_seq=%lu: address=%u, value=%d -> %s",
        static_cast<unsigned long>(goal_sequence),
        goal->io_address,
        goal->io_value,
        io_set_service_name_.c_str());

      if (!io_set_client_->wait_for_service(std::chrono::seconds(5)))
      {
        abort_with_message(goal_handle, started_at,
          "IO_SET: service unavailable at " + io_set_service_name_);
        return;
      }

      auto request = std::make_shared<IoSet::Request>();
      request->address = goal->io_address;
      request->value = goal->io_value;
      auto future = io_set_client_->async_send_request(request);

      if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
      {
        abort_with_message(goal_handle, started_at,
          "IO_SET: timed out waiting for response from " + io_set_service_name_);
        return;
      }

      auto response = future.get();
      auto result = std::make_shared<ExecuteMotion::Result>();
      result->success = response->success;
      result->message = response->success
        ? "IO_SET: " + response->message
        : "IO_SET failed: " + response->message;
      set_result_timing(started_at, *result);

      if (response->success)
      {
        goal_handle->succeed(result);
      }
      else
      {
        goal_handle->abort(result);
      }
      return;
    }

    // ── Motion primitives below require MoveGroup ──
    execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kPlanning, "ensure_move_group");
    std::string move_group_reason;
    if (!ensure_move_group(move_group_reason))
    {
      abort_with_message(goal_handle, started_at, move_group_reason);
      return;
    }

    std::string scene_reason;
    if (!ensure_scene_ready(scene_reason))
    {
      abort_with_message(goal_handle, started_at, scene_reason);
      return;
    }

    // Step 3.9: MOVE_JOINTS is a semantic alias for PTP (full joint-space target).
    // Use a local effective primitive string — no const_cast, no goal mutation.
    const std::string effective_primitive =
      (primitive == "MOVE_JOINTS") ? "PTP" : primitive;

    const double velocity_scale =
      (goal->velocity_scale > 0.0) ? goal->velocity_scale : TrajectoryPostProcessor::kDefaultVelocityScaling;
    const double acceleration_scale =
      (goal->acceleration_scale > 0.0) ? goal->acceleration_scale : TrajectoryPostProcessor::kDefaultAccelerationScaling;
    moveit::core::RobotState current_robot_state(move_group_->getRobotModel());
    builtin_interfaces::msg::Time source_joint_state_stamp;
    std::string current_state_reason;
    if (!build_current_robot_state(
          current_robot_state,
          current_state_reason,
          &source_joint_state_stamp))
    {
      abort_with_message(goal_handle, started_at,
        "failed to read current joint state: " + current_state_reason);
      return;
    }

    const auto * joint_model_group =
      current_robot_state.getJointModelGroup(kPlanningGroup);
    if (!joint_model_group)
    {
      abort_with_message(goal_handle, started_at,
        "planning group '" + std::string(kPlanningGroup) + "' not found");
      return;
    }

    std::vector<double> current_joint_positions;
    current_robot_state.copyJointGroupPositions(joint_model_group, current_joint_positions);
    const auto & active_joint_models = joint_model_group->getActiveJointModels();
    const auto & active_joint_names = joint_model_group->getActiveJointModelNames();
    execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kPlanning, "planning_request_prepared");
    publish_feedback(goal_handle, 0.2, "planning_request_prepared");

    PrimitiveRouterDispatch::PlanningRequest planning_request(current_robot_state);
    planning_request.goal = goal;
    planning_request.primitive = primitive;
    planning_request.effective_primitive = effective_primitive;
    planning_request.goal_sequence = goal_sequence;
    planning_request.velocity_scale = velocity_scale;
    planning_request.acceleration_scale = acceleration_scale;
    planning_request.current_joint_positions = current_joint_positions;
    planning_request.active_joint_models = active_joint_models;
    planning_request.active_joint_names = active_joint_names;
    planning_request.plan_with_interruption =
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
      };
    planning_request.interrupt_reason =
      [this, goal_handle, goal_sequence](const std::string & stage) -> std::string
      {
        return interrupt_reason(goal_handle, goal_sequence, stage);
      };

    const auto planning_result = primitive_router_dispatch_->plan_for_primitive(planning_request);
    if (planning_result.status == PrimitiveRouterDispatch::PlanningStatus::kCanceled)
    {
      const std::string cancel_reason = planning_result.is_move_joint ?
        ("MOVE_JOINT planning canceled: " + planning_result.reason) :
        ("planning canceled: " + planning_result.reason);
      cancel_with_message(goal_handle, started_at, cancel_reason);
      return;
    }
    if (planning_result.status != PrimitiveRouterDispatch::PlanningStatus::kSuccess)
    {
      abort_with_message(goal_handle, started_at, planning_result.reason);
      return;
    }

    publish_feedback(goal_handle, 0.55, "post_processing");
    if (!dispatch_executor_)
    {
      abort_with_message(goal_handle, started_at, "dispatch trajectory executor is unavailable");
      return;
    }
    trajectory_msgs::msg::JointTrajectory output_traj = planning_result.trajectory;
    DispatchTrajectoryExecutor::DispatchMetadata dispatch_metadata;
    dispatch_metadata.command_id = goal_id;
    dispatch_metadata.primitive = planning_result.dispatch_primitive;
    dispatch_metadata.planner_id = planning_result.planner_id;
    dispatch_metadata.source_joint_state_stamp = source_joint_state_stamp;
    dispatch_metadata.enforce_start_state_match = true;

    std::size_t dispatched_point_count = 0U;
    std::size_t dispatched_segment_count = 0U;
    const auto dispatch_result = dispatch_executor_->apply_budget_quality_and_dispatch(
      output_traj,
      planning_result.dispatch_primitive,
      dispatch_metadata,
      planning_result.cartesian_fraction,
      [this, goal_handle, goal_sequence](const std::string & stage) -> std::string
      {
        return interrupt_reason(goal_handle, goal_sequence, stage);
      },
      [this, goal_handle](const double progress, const std::string & state)
      {
        publish_feedback(goal_handle, progress, state);
      },
      [this, goal_sequence](const ExecutionPhase phase, const std::string & detail)
      {
        execution_orchestrator_.update_phase(goal_sequence, phase, detail);
      },
      dispatched_point_count,
      dispatched_segment_count);
    if (dispatch_result.status == DispatchTrajectoryExecutor::Status::kCanceled)
    {
      const std::string cancel_reason = planning_result.is_move_joint ?
        ("MOVE_JOINT dispatch canceled: " + dispatch_result.reason) :
        ("trajectory dispatch canceled: " + dispatch_result.reason);
      cancel_with_message(goal_handle, started_at, cancel_reason);
      return;
    }
    if (dispatch_result.status != DispatchTrajectoryExecutor::Status::kSuccess)
    {
      const std::string abort_reason = planning_result.is_move_joint ?
        ("MOVE_JOINT dispatch failed: " + dispatch_result.reason) :
        ("trajectory dispatch failed: " + dispatch_result.reason);
      abort_with_message(goal_handle, started_at, abort_reason);
      return;
    }

    publish_feedback(goal_handle, 0.95, "trajectory_execution_complete");

    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = true;

    std::ostringstream message;
    if (planning_result.is_move_joint)
    {
      message << "MOVE_JOINT success: joint[" << planning_result.move_joint_index << "]="
              << planning_result.move_joint_target_angle << " rad, points=" << dispatched_point_count
              << ", segments=" << dispatched_segment_count;
    }
    else
    {
      message << "execution success; primitive=" << primitive
              << ", planner_id=" << planning_result.planner_id
              << ", points=" << dispatched_point_count
              << ", segments=" << dispatched_segment_count;
      if (planning_result.cartesian_fraction >= 0.0)
      {
        message << ", cartesian_fraction=" << planning_result.cartesian_fraction;
      }
    }
    if (!planning_result.time_parameterization_note.empty())
    {
      message << ", " << planning_result.time_parameterization_note;
    }
    if (!planning_result.ruckig_reason.empty())
    {
      message << ", ruckig_status=" << planning_result.ruckig_reason;
    }
    if (!dispatch_result.note.empty())
    {
      message << ", " << dispatch_result.note;
    }

    result->message = message.str();
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
