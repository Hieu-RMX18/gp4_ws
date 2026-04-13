#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <interfaces/action/execute_motion.hpp>
#include <interfaces/action/dispatch_trajectory.hpp>
#include <interfaces/srv/get_current_pose.hpp>
#include <interfaces/srv/alarm_reset.hpp>
#include <interfaces/srv/io_set.hpp>

#include "motion_core/ik_selector.hpp"
#include "motion_core/angle_branch_utils.hpp"
#include "motion_core/execution_orchestrator.hpp"
#include "motion_core/move_rel_validator.hpp"
#include "motion_core/orientation_filter.hpp"
#include "motion_core/planner_router.hpp"
#include "motion_core/planning_scene_manager.hpp"
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

    // GET_POSE: dedicated query service — no motion, no planning, no execution.
    get_pose_service_ = create_service<GetCurrentPose>(
      "/get_current_pose",
      std::bind(&MotionCoreNode::handle_get_current_pose, this,
        std::placeholders::_1, std::placeholders::_2));

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
    // B2: HOME/PTP/LIN/MOVE_REL are fully wired end-to-end.
    // CIRC is deferred — see PRIMITIVE_SHORTLIST.md.
    // Step 3.1: New primitives added for this sprint.
    // CARTESIAN_PATH: multi-waypoint smooth path for draw_shape macros.
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

  struct PlannerSelection
  {
    std::string pipeline_id;
    std::string planner_id;
  };

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

  static PlannerSelection resolve_planner_selection(const std::string & planner_id)
  {
    std::string normalized = planner_id;
    normalized.erase(
      std::remove_if(normalized.begin(), normalized.end(), [](unsigned char c) {
        return std::isspace(c) != 0 || c == '_' || c == '-';
      }),
      normalized.end());

    std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    if (normalized == "PILZLIN" || normalized == "LIN")
    {
      return {"pilz_industrial_motion_planner", "LIN"};
    }

    if (normalized == "PILZPTP" || normalized == "PTP")
    {
      return {"pilz_industrial_motion_planner", "PTP"};
    }

    if (normalized == "PILZCIRC" || normalized == "CIRC")
    {
      return {"pilz_industrial_motion_planner", "CIRC"};
    }

    if (normalized == "OMPLRRTCONNECT" || normalized == "RRTCONNECT")
    {
      return {"ompl", "RRTConnect"};
    }

    return {"", planner_id};
  }

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

  static double max_abs_value(const std::vector<double> & values)
  {
    double max_value = 0.0;
    for (const double value : values)
    {
      max_value = std::max(max_value, std::abs(value));
    }
    return max_value;
  }

  static std::string format_joint_vector(const std::vector<double> & joints)
  {
    std::ostringstream stream;
    stream << "[";
    for (std::size_t index = 0; index < joints.size(); ++index)
    {
      if (index > 0U)
      {
        stream << ", ";
      }
      stream << std::fixed << std::setprecision(4) << joints[index];
    }
    stream << "]";
    return stream.str();
  }

  static bool is_geometry_sensitive_primitive(const std::string & primitive)
  {
    return primitive == "LIN" || primitive == "CIRC" ||
           primitive == "MOVE_REL" || primitive == "CARTESIAN_PATH";
  }

  void log_joint_branch_selection(
    const std::string & primitive,
    const std::uint64_t sequence,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & current,
    const std::vector<double> & requested,
    const BranchPreservedJointVectorResult & branch_result) const
  {
    RCLCPP_INFO(
      get_logger(),
      "%s goal_seq=%lu branch-preserved target selection: current=%s requested=%s chosen=%s max_abs_delta=%.4f",
      primitive.c_str(),
      static_cast<unsigned long>(sequence),
      format_joint_vector(current).c_str(),
      format_joint_vector(requested).c_str(),
      format_joint_vector(branch_result.chosen_targets).c_str(),
      max_abs_value(branch_result.deltas_from_current));

    for (std::size_t index = 0; index < branch_result.chosen_targets.size(); ++index)
    {
      const std::string joint_name =
        index < joint_names.size() ? joint_names[index] : ("joint_" + std::to_string(index));
      RCLCPP_DEBUG(
        get_logger(),
        "%s goal_seq=%lu joint=%s current=%.6f requested=%.6f chosen=%.6f delta=%.6f helper=%s",
        primitive.c_str(),
        static_cast<unsigned long>(sequence),
        joint_name.c_str(),
        current[index],
        requested[index],
        branch_result.chosen_targets[index],
        branch_result.deltas_from_current[index],
        branch_result.helper_used[index].c_str());
    }
  }

  std::string interrupt_reason(
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    const std::uint64_t sequence,
    const std::string & stage) const
  {
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

  static bool is_pose_goal_required(const std::string & primitive, bool has_joint_target)
  {
    if (primitive == "LIN")
    {
      return true;
    }

    if (primitive == "PTP")
    {
      return !has_joint_target;
    }

    return false;
  }

  static double quaternion_norm_sq(const geometry_msgs::msg::Quaternion & q)
  {
    return (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
  }

  bool build_current_robot_state(
    moveit::core::RobotState & current_state,
    std::string & reason) const
  {
    reason.clear();

    if (!move_group_)
    {
      reason = "MoveGroup unavailable";
      return false;
    }

    std::vector<double> current_joint_positions;
    if (!seed_manager_.get_current_joint_positions(current_joint_positions))
    {
      reason = "latest /yaskawa/joint_states unavailable";
      return false;
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

  /// Dispatch a validated trajectory to hw_adapter via DispatchTrajectory action.
  /// This is the V4-compliant execution path. motion_core never talks to FJT directly.
  StageResult dispatch_to_hw_adapter(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    const std::uint64_t sequence)
  {
    constexpr auto kPollPeriod = std::chrono::milliseconds(50);
    StageResult result;

    if (!dispatch_client_)
    {
      result.reason = "DispatchTrajectory client not initialized";
      return result;
    }

    if (!dispatch_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      result.reason = "DispatchTrajectory action server unavailable at " + dispatch_action_name_;
      return result;
    }

    const std::string pre_dispatch_interrupt =
      interrupt_reason(goal_handle, sequence, "dispatch_setup");
    if (!pre_dispatch_interrupt.empty())
    {
      result.status = StageStatus::kCanceled;
      result.reason = pre_dispatch_interrupt;
      return result;
    }

    DispatchTrajectory::Goal goal;
    goal.trajectory = trajectory;
    goal.timeout_sec = dispatch_timeout_sec_;

    RCLCPP_INFO(
      get_logger(),
      "execute_motion goal_seq=%lu dispatch_start target=%s points=%zu timeout=%.2fs",
      static_cast<unsigned long>(sequence),
      dispatch_action_name_.c_str(),
      trajectory.points.size(),
      dispatch_timeout_sec_);

    auto send_goal_future = dispatch_client_->async_send_goal(goal);
    const auto send_goal_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (send_goal_future.wait_for(kPollPeriod) != std::future_status::ready)
    {
      const std::string interrupt =
        interrupt_reason(goal_handle, sequence, "dispatch_goal_send");
      if (!interrupt.empty())
      {
        result.status = StageStatus::kCanceled;
        result.reason = interrupt;
        return result;
      }

      if (std::chrono::steady_clock::now() >= send_goal_deadline)
      {
        result.reason = "timed out sending trajectory to " + dispatch_action_name_;
        return result;
      }
    }

    auto dispatch_goal_handle = send_goal_future.get();
    if (!dispatch_goal_handle)
    {
      result.reason =
        "hw_adapter rejected trajectory dispatch goal (dispatch already in progress or unavailable)";
      return result;
    }

    execution_orchestrator_.update_phase(
      sequence,
      ExecutionPhase::kExecuting,
      "dispatch accepted by hw_adapter");

    auto result_future = dispatch_client_->async_get_result(dispatch_goal_handle);
    const auto result_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(dispatch_timeout_sec_, 10.0)));
    while (result_future.wait_for(kPollPeriod) != std::future_status::ready)
    {
      const std::string interrupt =
        interrupt_reason(goal_handle, sequence, "dispatch_wait");
      if (!interrupt.empty())
      {
        dispatch_client_->async_cancel_goal(dispatch_goal_handle);
        result.status = StageStatus::kCanceled;
        result.reason = interrupt;
        return result;
      }

      if (std::chrono::steady_clock::now() >= result_deadline)
      {
        dispatch_client_->async_cancel_goal(dispatch_goal_handle);
        result.reason = "timed out waiting for dispatch result from " + dispatch_action_name_;
        return result;
      }
    }

    const auto wrapped_result = result_future.get();
    if (!wrapped_result.result)
    {
      result.reason = "hw_adapter returned no dispatch result";
      return result;
    }

    if (!wrapped_result.result->success)
    {
      result.reason = wrapped_result.result->message.empty() ?
        "hw_adapter execution failed" : wrapped_result.result->message;
      const std::string interrupt =
        interrupt_reason(goal_handle, sequence, "dispatch_result");
      if (!interrupt.empty())
      {
        result.status = StageStatus::kCanceled;
        result.reason = interrupt + " (" + result.reason + ")";
      }
      return result;
    }

    std::ostringstream note;
    note << "dispatched_via=" << dispatch_action_name_
         << ", hw_execution_time=" << wrapped_result.result->execution_time_sec << "s";
    result.status = StageStatus::kSuccess;
    result.note = note.str();
    RCLCPP_INFO(
      get_logger(),
      "execute_motion goal_seq=%lu dispatch_end target=%s success=true detail=%s",
      static_cast<unsigned long>(sequence),
      dispatch_action_name_.c_str(),
      result.note.c_str());
    return result;
  }

  bool split_trajectory_for_dispatch(
    const trajectory_msgs::msg::JointTrajectory & input_trajectory,
    const std::size_t max_points_per_segment,
    std::vector<trajectory_msgs::msg::JointTrajectory> & output_segments,
    std::string & reason) const
  {
    reason.clear();
    output_segments.clear();

    if (input_trajectory.points.empty())
    {
      reason = "cannot split an empty trajectory";
      return false;
    }

    if (max_points_per_segment < 2U)
    {
      reason = "split policy requires max_points_per_segment >= 2";
      return false;
    }

    if (input_trajectory.points.size() <= max_points_per_segment)
    {
      output_segments.push_back(input_trajectory);
      return true;
    }

    const std::size_t point_count = input_trajectory.points.size();
    std::size_t start_index = 0U;

    while (start_index < point_count)
    {
      const std::size_t end_index =
        std::min(start_index + max_points_per_segment - 1U, point_count - 1U);

      trajectory_msgs::msg::JointTrajectory segment;
      segment.header = input_trajectory.header;
      segment.joint_names = input_trajectory.joint_names;
      segment.points.reserve((end_index - start_index) + 1U);
      for (std::size_t point_index = start_index; point_index <= end_index; ++point_index)
      {
        segment.points.push_back(input_trajectory.points[point_index]);
      }

      if (segment.points.size() < 2U)
      {
        reason = "split policy produced a segment with fewer than 2 points";
        return false;
      }

      const rclcpp::Duration segment_start_time(segment.points.front().time_from_start);
      for (auto & point : segment.points)
      {
        const rclcpp::Duration point_time(point.time_from_start);
        if (point_time < segment_start_time)
        {
          reason = "split policy produced non-monotonic segment timestamps";
          return false;
        }
        point.time_from_start = point_time - segment_start_time;
      }

      output_segments.push_back(std::move(segment));

      if (end_index >= point_count - 1U)
      {
        break;
      }

      // Segment boundary overlap keeps endpoint continuity while respecting
      // "split and dispatch sequentially" with a full stop between segments.
      start_index = end_index;
    }

    return true;
  }

  StageResult apply_budget_quality_and_dispatch(
    trajectory_msgs::msg::JointTrajectory trajectory,
    const std::string & primitive,
    const double cartesian_fraction,
    const std::shared_ptr<GoalHandleExecuteMotion> & goal_handle,
    const std::uint64_t sequence,
    std::size_t & reported_point_count,
    std::size_t & reported_segment_count)
  {
    StageResult result;
    std::ostringstream note_stream;
    const std::size_t original_point_count = trajectory.points.size();
    reported_point_count = original_point_count;
    reported_segment_count = 0U;

    std::vector<trajectory_msgs::msg::JointTrajectory> segments;

    if (original_point_count > kTrajectorySafeBudgetPoints)
    {
      if (is_geometry_sensitive_primitive(primitive))
      {
        std::string split_reason;
        if (!split_trajectory_for_dispatch(
              trajectory,
              kTrajectorySafeBudgetPoints,
              segments,
              split_reason))
        {
          result.reason = "split-and-dispatch mitigation failed: " + split_reason;
          return result;
        }

        RCLCPP_WARN(
          get_logger(),
          "goal_seq=%lu primitive=%s trajectory points=%zu exceed safe budget=%zu; "
          "using split-and-dispatch mitigation (segments=%zu, full-stop between segments).",
          static_cast<unsigned long>(sequence),
          primitive.c_str(),
          original_point_count,
          kTrajectorySafeBudgetPoints,
          segments.size());

        note_stream << "budget_mitigation=split_sequential"
                    << ", original_points=" << original_point_count
                    << ", safe_budget=" << kTrajectorySafeBudgetPoints
                    << ", segments=" << segments.size();
      }
      else
      {
        std::string downsample_reason;
        if (!trajectory_post_processor_.downsample_to_max_points(
              trajectory,
              kTrajectorySafeBudgetPoints,
              downsample_reason))
        {
          result.reason = "downsample mitigation failed: " + downsample_reason;
          return result;
        }

        RCLCPP_WARN(
          get_logger(),
          "goal_seq=%lu primitive=%s trajectory points=%zu exceed safe budget=%zu; "
          "using downsample mitigation (mitigated_points=%zu).",
          static_cast<unsigned long>(sequence),
          primitive.c_str(),
          original_point_count,
          kTrajectorySafeBudgetPoints,
          trajectory.points.size());

        segments.push_back(trajectory);
        reported_point_count = trajectory.points.size();
        note_stream << "budget_mitigation=downsample"
                    << ", original_points=" << original_point_count
                    << ", mitigated_points=" << trajectory.points.size()
                    << ", safe_budget=" << kTrajectorySafeBudgetPoints;
      }
    }
    else
    {
      segments.push_back(trajectory);
      note_stream << "budget_mitigation=none"
                  << ", points=" << original_point_count;
    }

    reported_segment_count = segments.size();

    for (std::size_t index = 0; index < segments.size(); ++index)
    {
      const double fraction_for_segment =
        (index == 0U) ? cartesian_fraction : QualityGate::kFractionNotApplicable;
      std::string quality_reason;
      if (!quality_gate_.validate_plan(segments[index], fraction_for_segment, primitive, quality_reason))
      {
        std::ostringstream quality_stream;
        quality_stream << "quality gate failed for dispatch segment " << (index + 1U)
                       << "/" << segments.size() << ": " << quality_reason;
        result.reason = quality_stream.str();
        return result;
      }
    }

    execution_orchestrator_.update_phase(
      sequence,
      ExecutionPhase::kDispatchWait,
      "trajectory dispatch requested");

    for (std::size_t index = 0; index < segments.size(); ++index)
    {
      const std::string pre_dispatch_interrupt =
        interrupt_reason(
        goal_handle,
        sequence,
        "pre_dispatch_segment_" + std::to_string(index + 1U));
      if (!pre_dispatch_interrupt.empty())
      {
        result.status = StageStatus::kCanceled;
        result.reason = pre_dispatch_interrupt;
        return result;
      }

      std::ostringstream dispatch_detail;
      dispatch_detail << "dispatching segment " << (index + 1U) << "/" << segments.size();
      execution_orchestrator_.update_phase(
        sequence,
        ExecutionPhase::kDispatchWait,
        dispatch_detail.str());

      const double progress = std::min(
        0.75 + (0.18 * static_cast<double>(index + 1U) / static_cast<double>(segments.size())),
        0.94);
      publish_feedback(goal_handle, progress, "trajectory_dispatch_requested");

      const auto dispatch_result = dispatch_to_hw_adapter(segments[index], goal_handle, sequence);
      if (dispatch_result.status == StageStatus::kCanceled)
      {
        result.status = StageStatus::kCanceled;
        std::ostringstream canceled_stream;
        canceled_stream << "dispatch segment " << (index + 1U) << "/" << segments.size()
                        << " canceled: " << dispatch_result.reason;
        result.reason = canceled_stream.str();
        return result;
      }
      if (dispatch_result.status != StageStatus::kSuccess)
      {
        std::ostringstream failure_stream;
        failure_stream << "dispatch segment " << (index + 1U) << "/" << segments.size()
                       << " failed: " << dispatch_result.reason;
        result.reason = failure_stream.str();
        return result;
      }

      if (!dispatch_result.note.empty())
      {
        note_stream << ", segment_" << (index + 1U) << "_detail={" << dispatch_result.note << "}";
      }
    }

    result.status = StageStatus::kSuccess;
    result.note = note_stream.str();
    return result;
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
      result.reason = "planning failed";
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
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
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
    // execute() runs in a detached worker thread (confirmed in handle_accepted),
    // so blocking sleep is safe here.
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

    // B2: CIRC is now rejected by is_supported_primitive() above.
    // When CIRC is implemented, add its planning logic here and update the whitelist.

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

    const std::string planning_interrupt =
      interrupt_reason(goal_handle, goal_sequence, "planning_setup");
    if (!planning_interrupt.empty())
    {
      cancel_with_message(goal_handle, started_at, planning_interrupt);
      return;
    }

    const double velocity_scale =
      (goal->velocity_scale > 0.0) ? goal->velocity_scale : TrajectoryPostProcessor::kDefaultVelocityScaling;
    const double acceleration_scale =
      (goal->acceleration_scale > 0.0) ? goal->acceleration_scale : TrajectoryPostProcessor::kDefaultAccelerationScaling;
    moveit::core::RobotState current_robot_state(move_group_->getRobotModel());
    std::string current_state_reason;
    if (!build_current_robot_state(current_robot_state, current_state_reason))
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

    // ── Step 3.8: MOVE_JOINT — single-axis motion via PTP planning ──
    if (primitive == "MOVE_JOINT")
    {
      const int joint_idx = goal->joint_index;
      const double target_angle = goal->joint_angle;

      if (joint_idx < 0 || static_cast<std::size_t>(joint_idx) >= current_joint_positions.size())
      {
        abort_with_message(goal_handle, started_at,
          "MOVE_JOINT: joint_index " + std::to_string(joint_idx) +
          " out of range [0, " + std::to_string(current_joint_positions.size() - 1) + "]");
        return;
      }

      std::vector<double> requested_joint_positions = current_joint_positions;
      requested_joint_positions[static_cast<std::size_t>(joint_idx)] = target_angle;
      const auto branch_result = choose_branch_preserved_joint_vector(
        active_joint_models,
        current_joint_positions,
        requested_joint_positions);
      if (!branch_result.success)
      {
        abort_with_message(goal_handle, started_at,
          "MOVE_JOINT branch preservation failed: " + branch_result.reason);
        return;
      }

      log_joint_branch_selection(
        "MOVE_JOINT",
        goal_sequence,
        active_joint_names,
        current_joint_positions,
        requested_joint_positions,
        branch_result);

      // Delegate to PTP joint-space planning below
      // Set joint target and fall through to PTP planning path
      move_group_->setPlanningTime(kPlanningTimeSec);
      // Use PTP planner for single-joint motion
      const std::string ptp_planner = planner_router_.route_planner("PTP", false);
      const PlannerSelection ptp_selection = resolve_planner_selection(
        ptp_planner.empty() ? "PILZ_PTP" : ptp_planner);
      RCLCPP_INFO(
        get_logger(),
        "MOVE_JOINT goal_seq=%lu planner_selected pipeline=%s planner=%s",
        static_cast<unsigned long>(goal_sequence),
        ptp_selection.pipeline_id.empty() ? "<default>" : ptp_selection.pipeline_id.c_str(),
        ptp_selection.planner_id.c_str());
      if (!ptp_selection.pipeline_id.empty())
      {
        move_group_->setPlanningPipelineId(ptp_selection.pipeline_id);
      }
      move_group_->setPlannerId(ptp_selection.planner_id);
      move_group_->setMaxVelocityScalingFactor(velocity_scale);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
      move_group_->setStartState(current_robot_state);
      move_group_->clearPoseTargets();

      if (!move_group_->setJointValueTarget(branch_result.chosen_targets))
      {
        abort_with_message(goal_handle, started_at,
          "MOVE_JOINT: failed to set joint target for PTP planning");
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_result = plan_with_interruption(
        plan,
        goal_handle,
        goal_sequence,
        "MOVE_JOINT planning");
      if (plan_result.status == StageStatus::kCanceled)
      {
        cancel_with_message(goal_handle, started_at, plan_result.reason);
        return;
      }
      if (plan_result.status != StageStatus::kSuccess)
      {
        abort_with_message(goal_handle, started_at,
          "MOVE_JOINT: " + plan_result.reason + " for joint[" +
          std::to_string(joint_idx) + "] -> " + std::to_string(target_angle));
        return;
      }

      // Post-process and dispatch (shared with PTP/HOME path below)
      publish_feedback(goal_handle, 0.55, "post_processing");

      moveit::core::RobotState reference_state(move_group_->getRobotModel());
      if (!moveit::core::robotStateMsgToRobotState(plan.start_state_, reference_state, true))
      {
        abort_with_message(goal_handle, started_at,
          "MOVE_JOINT: failed to convert plan start_state");
        return;
      }

      robot_trajectory::RobotTrajectory robot_traj(move_group_->getRobotModel(), kPlanningGroup);
      robot_traj.setRobotTrajectoryMsg(reference_state, plan.trajectory_);

      std::string time_parameterization_note;
      std::string ruckig_reason;
      const bool ruckig_ok =
        trajectory_post_processor_.apply_ruckig_smoothing(robot_traj, velocity_scale, acceleration_scale, ruckig_reason);
      const bool ruckig_applied = ruckig_ok &&
        ruckig_reason.find("unavailable") == std::string::npos &&
        ruckig_reason.find("skipped") == std::string::npos;

      if (ruckig_applied)
      {
        time_parameterization_note = "time_parameterization=ruckig";
      }
      else
      {
        if (robot_traj.getWayPointCount() >= 2U)
        {
          std::string totg_reason;
          if (!trajectory_post_processor_.apply_totg(
                robot_traj, velocity_scale, acceleration_scale, totg_reason))
          {
            std::string failure_detail =
              ruckig_reason.empty() ? "Ruckig unavailable" : ("Ruckig status: " + ruckig_reason);
            abort_with_message(
              goal_handle,
              started_at,
              "MOVE_JOINT time parameterization failed; " + failure_detail +
              "; TOTG fallback failed: " + totg_reason);
            return;
          }
          time_parameterization_note = "time_parameterization=totg_fallback";
        }
        else
        {
          time_parameterization_note = "time_parameterization=none_single_waypoint";
        }
      }

      moveit_msgs::msg::RobotTrajectory postprocessed_msg;
      robot_traj.getRobotTrajectoryMsg(postprocessed_msg);
      trajectory_msgs::msg::JointTrajectory output_traj = postprocessed_msg.joint_trajectory;

      std::size_t dispatched_point_count = 0U;
      std::size_t dispatched_segment_count = 0U;
      const auto dispatch_result = apply_budget_quality_and_dispatch(
        output_traj,
        primitive,
        QualityGate::kFractionNotApplicable,
        goal_handle,
        goal_sequence,
        dispatched_point_count,
        dispatched_segment_count);
      if (dispatch_result.status == StageStatus::kCanceled)
      {
        cancel_with_message(goal_handle, started_at,
          "MOVE_JOINT dispatch canceled: " + dispatch_result.reason);
        return;
      }
      if (dispatch_result.status != StageStatus::kSuccess)
      {
        abort_with_message(goal_handle, started_at,
          "MOVE_JOINT dispatch failed: " + dispatch_result.reason);
        return;
      }

      auto result = std::make_shared<ExecuteMotion::Result>();
      result->success = true;
      std::ostringstream msg;
      msg << "MOVE_JOINT success: joint[" << joint_idx << "]="
          << target_angle << " rad, points=" << dispatched_point_count
          << ", segments=" << dispatched_segment_count;
      if (!time_parameterization_note.empty()) { msg << ", " << time_parameterization_note; }
      if (!dispatch_result.note.empty()) { msg << ", " << dispatch_result.note; }
      result->message = msg.str();
      set_result_timing(started_at, *result);
      goal_handle->succeed(result);
      return;
    }

    std::string planner_id = goal->planner_id;
    if (planner_id.empty())
    {
      planner_id = planner_router_.route_planner(
        (effective_primitive == "HOME") ? "PTP" :
        (effective_primitive == "MOVE_REL") ? "LIN" :
        (effective_primitive == "CIRC") ? "CIRC" : effective_primitive, false);
    }

    if (planner_id.empty())
    {
      abort_with_message(goal_handle, started_at, "unable to resolve planner_id for primitive " + primitive);
      return;
    }

    const PlannerSelection planner_selection = resolve_planner_selection(planner_id);
    RCLCPP_INFO(
      get_logger(),
      "execute_motion goal_seq=%lu planner_selected primitive=%s pipeline=%s planner=%s",
      static_cast<unsigned long>(goal_sequence),
      effective_primitive.c_str(),
      planner_selection.pipeline_id.empty() ? "<default>" : planner_selection.pipeline_id.c_str(),
      planner_selection.planner_id.c_str());

    move_group_->setPlanningTime(kPlanningTimeSec);
    if (!planner_selection.pipeline_id.empty())
    {
      move_group_->setPlanningPipelineId(planner_selection.pipeline_id);
    }
    move_group_->setPlannerId(planner_selection.planner_id);
    move_group_->setMaxVelocityScalingFactor(velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
    move_group_->setStartState(current_robot_state);
    move_group_->clearPoseTargets();

    execution_orchestrator_.update_phase(goal_sequence, ExecutionPhase::kPlanning, "planning_request_prepared");
    publish_feedback(goal_handle, 0.2, "planning_request_prepared");

    geometry_msgs::msg::Pose normalized_pose;
    const bool has_joint_target = !goal->joint_target.empty();
    bool pose_required = is_pose_goal_required(effective_primitive, has_joint_target);
    bool move_rel_resolved = false;

    // ── MOVE_REL: resolve relative delta into absolute Cartesian target ──
    // Validation functions are in move_rel_validator.hpp (single source of truth
    // for delta limits and workspace bounds — no duplication with safety_rules.yaml
    // constants; see that header for the cross-reference documentation).
    if (effective_primitive == "MOVE_REL")
    {
      std::string rel_reason;

      if (!validate_move_rel_frame(goal->reference_frame, rel_reason))
      {
        abort_with_message(goal_handle, started_at, rel_reason);
        return;
      }

      const double dx = goal->delta_x;
      const double dy = goal->delta_y;
      const double dz = goal->delta_z;

      if (!validate_move_rel_deltas(dx, dy, dz, rel_reason))
      {
        abort_with_message(goal_handle, started_at, rel_reason);
        return;
      }

      geometry_msgs::msg::PoseStamped current_stamped;
      if (!read_current_tcp_pose(current_stamped, rel_reason, 5.0))
      {
        abort_with_message(goal_handle, started_at,
          "MOVE_REL: failed to read current TCP pose: " + rel_reason);
        return;
      }

      const auto & current = current_stamped.pose;
      if (quaternion_norm_sq(current.orientation) <= 1e-12)
      {
        abort_with_message(goal_handle, started_at,
          "MOVE_REL: current pose has invalid orientation; cannot proceed safely");
        return;
      }

      normalized_pose = compute_move_rel_target(current, dx, dy, dz);

      if (!validate_move_rel_target_bounds(normalized_pose, rel_reason))
      {
        abort_with_message(goal_handle, started_at, rel_reason);
        return;
      }

      RCLCPP_INFO(get_logger(),
        "MOVE_REL resolved: delta=(%.4f, %.4f, %.4f), "
        "current=(%.4f, %.4f, %.4f), target=(%.4f, %.4f, %.4f)",
        dx, dy, dz,
        current.position.x, current.position.y, current.position.z,
        normalized_pose.position.x, normalized_pose.position.y, normalized_pose.position.z);

      move_rel_resolved = true;
      pose_required = true;  // Enable shared IK/orientation path below
    }

    if (pose_required)
    {
      if (!move_rel_resolved)
      {
        // Original path: extract pose from goal for LIN/PTP
        normalized_pose = goal->target_pose;
        if (quaternion_norm_sq(normalized_pose.orientation) <= 1e-12)
        {
          geometry_msgs::msg::PoseStamped current_stamped;
          std::string current_pose_reason;
          if (!read_current_tcp_pose(current_stamped, current_pose_reason, 5.0))
          {
            abort_with_message(
              goal_handle,
              started_at,
              "orientation unresolved: command omitted orientation and current pose is unavailable: " +
              current_pose_reason);
            return;
          }
          const auto & current_pose = current_stamped.pose;
          normalized_pose.orientation = current_pose.orientation;
          RCLCPP_WARN(
            get_logger(),
            "Goal orientation omitted; using current end-effector orientation for deterministic IK.");
        }
      }

      std::string orientation_reason;
      if (!orientation_filter_.normalize_and_validate(normalized_pose, orientation_reason))
      {
        abort_with_message(goal_handle, started_at, "orientation rejected: " + orientation_reason);
        return;
      }

      std::vector<double> seed_state;
      if (!seed_manager_.get_seed_state(seed_state))
      {
        abort_with_message(
          goal_handle,
          started_at,
          "IK seed unavailable: waiting /yaskawa/joint_states or named-target fallback hook not integrated");
        return;
      }

      std::vector<double> ik_solution;
      std::string ik_reason;
      if (!ik_selector_.solve_ik(normalized_pose, seed_state, ik_solution, ik_reason))
      {
        abort_with_message(goal_handle, started_at, "IK solve failed: " + ik_reason);
        return;
      }

      if (effective_primitive == "PTP")
      {
        RCLCPP_INFO(
          get_logger(),
          "execute_motion goal_seq=%lu IK-derived PTP target current_seed=%s ik_solution=%s",
          static_cast<unsigned long>(goal_sequence),
          format_joint_vector(seed_state).c_str(),
          format_joint_vector(ik_solution).c_str());
        if (!move_group_->setJointValueTarget(ik_solution))
        {
          abort_with_message(goal_handle, started_at, "failed to set IK-derived joint target for PTP");
          return;
        }
      }
    }

    const std::string before_plan_interrupt =
      interrupt_reason(goal_handle, goal_sequence, "pre_plan");
    if (!before_plan_interrupt.empty())
    {
      cancel_with_message(goal_handle, started_at, before_plan_interrupt);
      return;
    }

    moveit_msgs::msg::RobotTrajectory planned_trajectory_msg;
    moveit_msgs::msg::RobotState plan_start_state_msg;
    bool has_plan_start_state = false;
    double cartesian_fraction = QualityGate::kFractionNotApplicable;

    // ── CIRC: circular arc via Pilz — waypoints[0]=auxiliary, target_pose=final ──
    if (effective_primitive == "CIRC")
    {
      if (goal->waypoints.empty())
      {
        abort_with_message(goal_handle, started_at,
          "CIRC requires at least 1 auxiliary waypoint");
        return;
      }

      // Pilz CIRC requires exactly 2 pose targets: [auxiliary_pose, target_pose]
      geometry_msgs::msg::Pose aux_pose = goal->waypoints[0];
      geometry_msgs::msg::Pose final_pose = goal->target_pose;

      if (quaternion_norm_sq(final_pose.orientation) <= 1e-12)
      {
        // Reuse current orientation if final pose has no orientation
        geometry_msgs::msg::PoseStamped current_stamped;
        std::string current_reason;
        if (!read_current_tcp_pose(current_stamped, current_reason, 5.0))
        {
          abort_with_message(goal_handle, started_at,
            "CIRC: cannot resolve orientation for final pose: " + current_reason);
          return;
        }
        final_pose.orientation = current_stamped.pose.orientation;
      }

      if (quaternion_norm_sq(aux_pose.orientation) <= 1e-12)
      {
        aux_pose.orientation = final_pose.orientation;
      }

      // Validate both orientations are (approximately) unit quaternions and
      // renormalize within a 1% slack. A non-unit quaternion here indicates
      // either a serialization bug upstream or a malformed pose — MoveIt/Pilz
      // would silently misinterpret the rotation, so fail closed.
      auto ensure_unit_quaternion =
        [&](geometry_msgs::msg::Pose & pose, const char * which) -> bool
        {
          const double n2 = quaternion_norm_sq(pose.orientation);
          if (n2 < 0.98 || n2 > 1.02)
          {
            abort_with_message(
              goal_handle,
              started_at,
              std::string("CIRC: ") + which +
                " orientation is not a unit quaternion (norm^2=" +
                std::to_string(n2) + ")");
            return false;
          }
          const double n = std::sqrt(n2);
          pose.orientation.x /= n;
          pose.orientation.y /= n;
          pose.orientation.z /= n;
          pose.orientation.w /= n;
          return true;
        };
      if (!ensure_unit_quaternion(final_pose, "target_pose"))
      {
        return;
      }
      if (!ensure_unit_quaternion(aux_pose, "auxiliary waypoint"))
      {
        return;
      }

      // Fail-closed planner routing: CIRC must be planned by Pilz CIRC.
      // A computeCartesianPath fallback here would produce a line, not an
      // arc — silently wrong. Abort instead if Pilz CIRC was not selected
      // (e.g. plugin load failure, config drift).
      if (planner_selection.pipeline_id != "pilz_industrial_motion_planner" ||
          planner_selection.planner_id != "CIRC")
      {
        abort_with_message(
          goal_handle,
          started_at,
          std::string("CIRC: planner routing failed, expected pilz CIRC got pipeline='") +
            planner_selection.pipeline_id + "' planner='" +
            planner_selection.planner_id + "'");
        return;
      }

      RCLCPP_INFO(get_logger(),
        "CIRC: planning arc via Pilz CIRC planner");

      std::vector<geometry_msgs::msg::Pose> circ_poses;
      circ_poses.push_back(aux_pose);
      circ_poses.push_back(final_pose);
      move_group_->setPoseTargets(circ_poses);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_result = plan_with_interruption(
        plan,
        goal_handle,
        goal_sequence,
        "CIRC planning");
      if (plan_result.status == StageStatus::kCanceled)
      {
        cancel_with_message(goal_handle, started_at, plan_result.reason);
        return;
      }
      if (plan_result.status != StageStatus::kSuccess)
      {
        abort_with_message(goal_handle, started_at,
          "CIRC: Pilz CIRC planning failed: " + plan_result.reason);
        return;
      }

      planned_trajectory_msg = plan.trajectory_;
      plan_start_state_msg = plan.start_state_;
      has_plan_start_state = true;
      cartesian_fraction = 1.0;  // Pilz CIRC is exact; fraction implicit 1.0

      RCLCPP_INFO(get_logger(),
        "CIRC: planned fraction=%.3f, points=%zu",
        cartesian_fraction,
        planned_trajectory_msg.joint_trajectory.points.size());
    }
    else if (effective_primitive == "CARTESIAN_PATH")
    {
      if (goal->waypoints.empty())
      {
        abort_with_message(goal_handle, started_at,
          "CARTESIAN_PATH requires non-empty waypoints array");
        return;
      }

      RCLCPP_INFO(get_logger(),
        "CARTESIAN_PATH: planning smooth path through %zu waypoints",
        goal->waypoints.size());

      // Use all waypoints for a single computeCartesianPath call
      std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
      cartesian_waypoints.reserve(goal->waypoints.size());
      for (const auto & wp : goal->waypoints)
      {
        cartesian_waypoints.push_back(wp);
      }

      cartesian_fraction = move_group_->computeCartesianPath(
        cartesian_waypoints,
        kCartesianEefStep,
        kCartesianJumpThreshold,
        planned_trajectory_msg,
        true);

      if (cartesian_fraction < 0.0)
      {
        abort_with_message(goal_handle, started_at,
          "CARTESIAN_PATH: computeCartesianPath failed");
        return;
      }

      RCLCPP_INFO(get_logger(),
        "CARTESIAN_PATH: planned with fraction=%.3f, points=%zu",
        cartesian_fraction,
        planned_trajectory_msg.joint_trajectory.points.size());
    }
    else if (effective_primitive == "HOME")
    {
      if (!move_group_->setNamedTarget("home"))
      {
        abort_with_message(
          goal_handle,
          started_at,
          "HOME target not available in current SRDF; HOME is not yet fully implemented in Phase 4");
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_result = plan_with_interruption(
        plan,
        goal_handle,
        goal_sequence,
        "HOME planning");
      if (plan_result.status == StageStatus::kCanceled)
      {
        cancel_with_message(goal_handle, started_at, plan_result.reason);
        return;
      }
      if (plan_result.status != StageStatus::kSuccess)
      {
        abort_with_message(goal_handle, started_at, "planning failed for HOME primitive: " + plan_result.reason);
        return;
      }

      planned_trajectory_msg = plan.trajectory_;
      plan_start_state_msg = plan.start_state_;
      has_plan_start_state = true;
    }
    else if (effective_primitive == "PTP")
    {
      if (has_joint_target)
      {
        const auto branch_result = choose_branch_preserved_joint_vector(
          active_joint_models,
          current_joint_positions,
          goal->joint_target);
        if (!branch_result.success)
        {
          abort_with_message(goal_handle, started_at,
            "invalid branch-preserved joint_target for PTP goal: " + branch_result.reason);
          return;
        }

        log_joint_branch_selection(
          "PTP",
          goal_sequence,
          active_joint_names,
          current_joint_positions,
          goal->joint_target,
          branch_result);

        if (!move_group_->setJointValueTarget(branch_result.chosen_targets))
        {
          abort_with_message(goal_handle, started_at, "invalid joint_target for PTP goal");
          return;
        }
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_result = plan_with_interruption(
        plan,
        goal_handle,
        goal_sequence,
        "PTP planning");
      if (plan_result.status == StageStatus::kCanceled)
      {
        cancel_with_message(goal_handle, started_at, plan_result.reason);
        return;
      }
      if (plan_result.status != StageStatus::kSuccess)
      {
        abort_with_message(goal_handle, started_at, "planning failed for PTP primitive: " + plan_result.reason);
        return;
      }

      planned_trajectory_msg = plan.trajectory_;
      plan_start_state_msg = plan.start_state_;
      has_plan_start_state = true;
    }
    else if (effective_primitive == "LIN" || effective_primitive == "MOVE_REL")
    {
      // V4 E1: PILZ_LIN is the primary LIN strategy.
      // MOVE_REL delegates to LIN after resolving the absolute target above.
      // computeCartesianPath is kept as fallback only.
      // When planner_selection routes to Pilz LIN, we use MoveGroupInterface::plan()
      // with setPoseTarget, which triggers Pilz LIN directly.
      if (planner_selection.pipeline_id == "pilz_industrial_motion_planner" &&
          planner_selection.planner_id == "LIN")
      {
        // Use Pilz LIN natively via MoveGroupInterface
        move_group_->setPoseTarget(normalized_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = plan_with_interruption(
          plan,
          goal_handle,
          goal_sequence,
          effective_primitive + " planning");
        if (plan_result.status == StageStatus::kCanceled)
        {
          cancel_with_message(goal_handle, started_at, plan_result.reason);
          return;
        }
        if (plan_result.status != StageStatus::kSuccess)
        {
          // Fallback to computeCartesianPath if Pilz LIN fails
          RCLCPP_WARN(get_logger(),
            "Pilz LIN planning failed for goal_seq=%lu (%s), attempting computeCartesianPath fallback.",
            static_cast<unsigned long>(goal_sequence),
            plan_result.reason.c_str());

          std::vector<geometry_msgs::msg::Pose> waypoints;
          waypoints.push_back(normalized_pose);

          cartesian_fraction = move_group_->computeCartesianPath(
            waypoints,
            kCartesianEefStepRelaxed,
            kCartesianJumpThreshold,
            planned_trajectory_msg,
            true);

          if (cartesian_fraction < 0.0)
          {
            abort_with_message(goal_handle, started_at,
              "both Pilz LIN and computeCartesianPath failed for LIN primitive");
            return;
          }
        }
        else
        {
          planned_trajectory_msg = plan.trajectory_;
          plan_start_state_msg = plan.start_state_;
          has_plan_start_state = true;
          // Pilz provides native timing; fraction is implicit 1.0
          cartesian_fraction = 1.0;
        }
      }
      else
      {
        // Fallback: computeCartesianPath
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(normalized_pose);

        cartesian_fraction = move_group_->computeCartesianPath(
          waypoints,
          kCartesianEefStepRelaxed,
          kCartesianJumpThreshold,
          planned_trajectory_msg,
          true);

        if (cartesian_fraction < 0.0)
        {
          abort_with_message(goal_handle, started_at, "computeCartesianPath returned error for LIN primitive");
          return;
        }
      }
    }

    if (planned_trajectory_msg.joint_trajectory.points.empty())
    {
      abort_with_message(goal_handle, started_at, "planner returned empty joint trajectory");
      return;
    }

    const std::string post_plan_interrupt =
      interrupt_reason(goal_handle, goal_sequence, "post_plan");
    if (!post_plan_interrupt.empty())
    {
      cancel_with_message(goal_handle, started_at, post_plan_interrupt);
      return;
    }

    publish_feedback(goal_handle, 0.55, "post_processing");

    moveit::core::RobotState reference_state(move_group_->getRobotModel());
    if (has_plan_start_state)
    {
      if (!moveit::core::robotStateMsgToRobotState(plan_start_state_msg, reference_state, true))
      {
        abort_with_message(goal_handle, started_at, "failed to convert plan start_state for post-processing");
        return;
      }
    }
    else
    {
      reference_state = current_robot_state;
    }

    robot_trajectory::RobotTrajectory robot_trajectory(move_group_->getRobotModel(), kPlanningGroup);
    robot_trajectory.setRobotTrajectoryMsg(reference_state, planned_trajectory_msg);

    std::string time_parameterization_note;
    std::string ruckig_reason;
      const bool ruckig_ok =
      trajectory_post_processor_.apply_ruckig_smoothing(robot_trajectory, velocity_scale, acceleration_scale, ruckig_reason);
    const bool ruckig_applied = ruckig_ok &&
      ruckig_reason.find("unavailable") == std::string::npos &&
      ruckig_reason.find("skipped") == std::string::npos;

    if (ruckig_applied)
    {
      time_parameterization_note = "time_parameterization=ruckig";
    }
    else
    {
      if (robot_trajectory.getWayPointCount() >= 2U)
      {
        std::string totg_reason;
        if (!trajectory_post_processor_.apply_totg(
              robot_trajectory,
              velocity_scale,
              acceleration_scale,
              totg_reason))
        {
          std::string failure_detail =
            ruckig_reason.empty() ? "Ruckig unavailable" : ("Ruckig status: " + ruckig_reason);
          abort_with_message(
            goal_handle,
            started_at,
            "time parameterization failed; " + failure_detail +
            "; TOTG fallback failed: " + totg_reason);
          return;
        }
        time_parameterization_note = "time_parameterization=totg_fallback";
      }
      else
      {
        time_parameterization_note = "time_parameterization=none_single_waypoint";
      }
    }

    moveit_msgs::msg::RobotTrajectory postprocessed_msg;
    robot_trajectory.getRobotTrajectoryMsg(postprocessed_msg);
    trajectory_msgs::msg::JointTrajectory output_traj = postprocessed_msg.joint_trajectory;

    std::size_t dispatched_point_count = 0U;
    std::size_t dispatched_segment_count = 0U;
    const auto dispatch_result = apply_budget_quality_and_dispatch(
      output_traj,
      effective_primitive,
      cartesian_fraction,
      goal_handle,
      goal_sequence,
      dispatched_point_count,
      dispatched_segment_count);
    if (dispatch_result.status == StageStatus::kCanceled)
    {
      cancel_with_message(goal_handle, started_at, "trajectory dispatch canceled: " + dispatch_result.reason);
      return;
    }
    if (dispatch_result.status != StageStatus::kSuccess)
    {
      abort_with_message(goal_handle, started_at, "trajectory dispatch failed: " + dispatch_result.reason);
      return;
    }

    publish_feedback(goal_handle, 0.95, "trajectory_execution_complete");

    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = true;

    std::ostringstream message;
    message << "execution success; primitive=" << primitive
            << ", planner_id=" << planner_selection.planner_id
            << ", points=" << dispatched_point_count
            << ", segments=" << dispatched_segment_count;

    if (cartesian_fraction >= 0.0)
    {
      message << ", cartesian_fraction=" << cartesian_fraction;
    }

    if (!time_parameterization_note.empty())
    {
      message << ", " << time_parameterization_note;
    }
    if (!ruckig_reason.empty())
    {
      message << ", ruckig_status=" << ruckig_reason;
    }
    if (!dispatch_result.note.empty())
    {
      message << ", " << dispatch_result.note;
    }

    result->message = message.str();
    set_result_timing(started_at, *result);
    goal_handle->succeed(result);
  }

  /// GET_POSE service handler — query-only, no motion, no planning.
  /// Returns current TCP pose from the current RobotState FK in base_link.
  /// This callback MUST NOT trigger motion planning, execution, or state mutation.
  void handle_get_current_pose(
    const std::shared_ptr<GetCurrentPose::Request> request,
    std::shared_ptr<GetCurrentPose::Response> response)
  {
    // Default empty/missing reference_frame to base_link
    std::string frame = request->reference_frame;
    if (frame.empty())
    {
      frame = "base_link";
    }

    // Fail-closed: only base_link is supported in v1
    if (frame != "base_link")
    {
      response->success = false;
      response->message =
        "unsupported reference_frame '" + frame +
        "'; only 'base_link' is supported";
      RCLCPP_WARN(get_logger(), "GET_POSE rejected: %s", response->message.c_str());
      return;
    }

    // Ensure MoveGroup is available (read-only operation)
    std::string move_group_reason;
    if (!ensure_move_group(move_group_reason))
    {
      response->success = false;
      response->message =
        "cannot read current pose: MoveGroup unavailable — " + move_group_reason;
      RCLCPP_ERROR(get_logger(), "GET_POSE failed: %s", response->message.c_str());
      return;
    }

    // Read current TCP pose — no planning, no execution, no side effects
    geometry_msgs::msg::PoseStamped current_stamped;
    std::string current_pose_reason;
    if (!read_current_tcp_pose(current_stamped, current_pose_reason, 5.0))
    {
      response->success = false;
      response->message = "failed to read current TCP pose: " + current_pose_reason;
      RCLCPP_ERROR(get_logger(), "GET_POSE failed: %s", response->message.c_str());
      return;
    }

    if (current_stamped.header.frame_id != frame)
    {
      response->success = false;
      response->message =
        "current TCP pose is available in frame '" + current_stamped.header.frame_id +
        "'; expected '" + frame + "'";
      RCLCPP_ERROR(get_logger(), "GET_POSE failed: %s", response->message.c_str());
      return;
    }

    // Validate that the returned pose has a non-degenerate orientation
    const auto & q = current_stamped.pose.orientation;
    const double qnorm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (qnorm_sq <= 1e-12)
    {
      response->success = false;
      response->message =
        "current pose has invalid/zero orientation; robot state may be unavailable";
      RCLCPP_ERROR(get_logger(), "GET_POSE failed: %s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "current TCP pose in frame: " + frame;
    response->current_pose = current_stamped.pose;

    RCLCPP_INFO(get_logger(),
      "GET_POSE success: position=(%.4f, %.4f, %.4f), "
      "orientation=(%.4f, %.4f, %.4f, %.4f), frame=%s",
      current_stamped.pose.position.x,
      current_stamped.pose.position.y,
      current_stamped.pose.position.z,
      q.x, q.y, q.z, q.w,
      frame.c_str());
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
