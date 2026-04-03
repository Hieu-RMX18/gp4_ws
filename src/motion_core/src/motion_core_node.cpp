#include <algorithm>
#include <chrono>
#include <cctype>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <interfaces/action/execute_motion.hpp>

#include "motion_core/ik_selector.hpp"
#include "motion_core/orientation_filter.hpp"
#include "motion_core/planner_router.hpp"
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
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  MotionCoreNode()
  : rclcpp::Node("motion_core_node"),
    seed_manager_(*this),
    quality_gate_(TrajectoryPostProcessor::kMaxTrajectoryPoints, QualityGate::kMinimumCartesianFraction)
  {
    action_server_ = rclcpp_action::create_server<ExecuteMotion>(
      this,
      "execute_motion",
      std::bind(&MotionCoreNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionCoreNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MotionCoreNode::handle_accepted, this, std::placeholders::_1));

    trajectory_action_name_ = declare_parameter<std::string>(
      "trajectory_action_name",
      "/controller_manager/follow_joint_trajectory");
    trajectory_action_timeout_sec_ = declare_parameter<double>(
      "trajectory_action_timeout_sec",
      30.0);
    trajectory_execution_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this,
      trajectory_action_name_);

    RCLCPP_INFO(
      get_logger(),
      "motion_core_node started (validated trajectories execute through follow_joint_trajectory)");
  }

  void initialize()
  {
    std::string reason;
    if (!ensure_move_group(reason)) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize MoveGroup in initialize(): %s", reason.c_str());
    }
  }

private:
  static constexpr double kMaxVelocityScale = 0.3;
  static constexpr double kMaxAccelerationScale = 0.2;
  static constexpr double kPlanningTimeSec = 5.0;
  static constexpr double kCartesianEefStep = 0.005;
  static constexpr double kCartesianJumpThreshold = 0.0;
  static constexpr const char * kPlanningGroup = "gp4_arm";

  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_execution_client_;

  PlannerRouter planner_router_;
  OrientationFilter orientation_filter_;
  SeedManager seed_manager_;
  IkSelector ik_selector_;
  TrajectoryPostProcessor trajectory_post_processor_;
  QualityGate quality_gate_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string trajectory_action_name_;
  double trajectory_action_timeout_sec_{30.0};

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
    return primitive == "HOME" || primitive == "PTP" || primitive == "LIN" || primitive == "CIRC";
  }

  struct PlannerSelection
  {
    std::string pipeline_id;
    std::string planner_id;
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

      RCLCPP_INFO(get_logger(), "Waiting for current robot state...");
      bool state_ok = false;
      for (int i = 0; i < 20 && !state_ok; ++i) {
        auto s = move_group_->getCurrentState(0.5);
        if (s) { state_ok = true; }
        else { rclcpp::sleep_for(std::chrono::milliseconds(500)); }
      }
      if (!state_ok) {
        RCLCPP_WARN(get_logger(), "Could not get initial robot state - will retry per-request");
      }

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

  bool execute_joint_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::string & reason)
  {
    reason.clear();

    if (!trajectory_execution_client_)
    {
      reason = "follow_joint_trajectory client not initialized";
      return false;
    }

    if (!trajectory_execution_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      reason = "follow_joint_trajectory action unavailable at " + trajectory_action_name_;
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = trajectory;

    auto send_goal_future = trajectory_execution_client_->async_send_goal(goal);
    if (send_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      reason = "timed out sending trajectory to " + trajectory_action_name_;
      return false;
    }

    GoalHandleFollowJointTrajectory::SharedPtr controller_goal_handle = send_goal_future.get();
    if (!controller_goal_handle)
    {
      reason = "trajectory controller rejected goal";
      return false;
    }

    auto result_future = trajectory_execution_client_->async_get_result(controller_goal_handle);
    const auto result_timeout =
      std::chrono::duration<double>(std::max(trajectory_action_timeout_sec_, 1.0));
    if (result_future.wait_for(result_timeout) != std::future_status::ready)
    {
      trajectory_execution_client_->async_cancel_goal(controller_goal_handle);
      reason = "timed out waiting for controller result from " + trajectory_action_name_;
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (!wrapped_result.result)
    {
      reason = "trajectory controller returned no result";
      return false;
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      std::ostringstream status_message;
      status_message << "trajectory action returned result code "
                     << static_cast<int>(wrapped_result.code);
      if (!wrapped_result.result->error_string.empty())
      {
        status_message << ": " << wrapped_result.result->error_string;
      }
      reason = status_message.str();
      return false;
    }

    if (wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      std::ostringstream controller_error;
      controller_error << "controller error " << wrapped_result.result->error_code;
      if (!wrapped_result.result->error_string.empty())
      {
        controller_error << ": " << wrapped_result.result->error_string;
      }
      reason = controller_error.str();
      return false;
    }

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

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteMotion::Goal> goal)
  {
    if (!goal)
    {
      return rclcpp_action::GoalResponse::REJECT;
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

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteMotion>)
  {
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

    publish_feedback(goal_handle, 0.05, "goal_accepted");

    if (goal->require_approval)
    {
      abort_with_message(
        goal_handle,
        started_at,
        "require_approval flow is not yet implemented in Phase 4 execution server");
      return;
    }

    const std::string primitive = normalize_primitive(goal->primitive_type);
    if (!is_supported_primitive(primitive))
    {
      abort_with_message(
        goal_handle,
        started_at,
        "unsupported primitive_type for Phase 4: " + goal->primitive_type);
      return;
    }

    if (primitive == "CIRC")
    {
      abort_with_message(
        goal_handle,
        started_at,
        "CIRC planning semantics are not yet implemented in Phase 4; deferred to Phase 5 primitives");
      return;
    }

    std::string move_group_reason;
    if (!ensure_move_group(move_group_reason))
    {
      abort_with_message(goal_handle, started_at, move_group_reason);
      return;
    }

    const double velocity_scale =
      (goal->velocity_scale > 0.0) ? goal->velocity_scale : TrajectoryPostProcessor::kDefaultVelocityScaling;
    const double acceleration_scale =
      (goal->acceleration_scale > 0.0) ? goal->acceleration_scale : TrajectoryPostProcessor::kDefaultAccelerationScaling;

    std::string planner_id = goal->planner_id;
    if (planner_id.empty())
    {
      planner_id = planner_router_.route_planner((primitive == "HOME") ? "PTP" : primitive, false);
    }

    if (planner_id.empty())
    {
      abort_with_message(goal_handle, started_at, "unable to resolve planner_id for primitive " + primitive);
      return;
    }

    const PlannerSelection planner_selection = resolve_planner_selection(planner_id);

    move_group_->setPlanningTime(kPlanningTimeSec);
    if (!planner_selection.pipeline_id.empty())
    {
      move_group_->setPlanningPipelineId(planner_selection.pipeline_id);
    }
    move_group_->setPlannerId(planner_selection.planner_id);
    move_group_->setMaxVelocityScalingFactor(velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();

    publish_feedback(goal_handle, 0.2, "planning_request_prepared");

    geometry_msgs::msg::Pose normalized_pose;
    const bool has_joint_target = !goal->joint_target.empty();
    const bool pose_required = is_pose_goal_required(primitive, has_joint_target);

    if (pose_required)
    {
      normalized_pose = goal->target_pose;
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

      if (primitive == "PTP")
      {
        if (!move_group_->setJointValueTarget(ik_solution))
        {
          abort_with_message(goal_handle, started_at, "failed to set IK-derived joint target for PTP");
          return;
        }
      }
    }

    moveit_msgs::msg::RobotTrajectory planned_trajectory_msg;
    moveit_msgs::msg::RobotState plan_start_state_msg;
    bool has_plan_start_state = false;
    double cartesian_fraction = QualityGate::kFractionNotApplicable;

    if (primitive == "HOME")
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
      const auto plan_code = move_group_->plan(plan);
      if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
      {
        abort_with_message(goal_handle, started_at, "planning failed for HOME primitive");
        return;
      }

      planned_trajectory_msg = plan.trajectory_;
      plan_start_state_msg = plan.start_state_;
      has_plan_start_state = true;
    }
    else if (primitive == "PTP")
    {
      if (has_joint_target)
      {
        if (!move_group_->setJointValueTarget(goal->joint_target))
        {
          abort_with_message(goal_handle, started_at, "invalid joint_target for PTP goal");
          return;
        }
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_code = move_group_->plan(plan);
      if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
      {
        abort_with_message(goal_handle, started_at, "planning failed for PTP primitive");
        return;
      }

      planned_trajectory_msg = plan.trajectory_;
      plan_start_state_msg = plan.start_state_;
      has_plan_start_state = true;
    }
    else if (primitive == "LIN")
    {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(normalized_pose);

      cartesian_fraction = move_group_->computeCartesianPath(
        waypoints,
        kCartesianEefStep,
        kCartesianJumpThreshold,
        planned_trajectory_msg,
        true);

      if (cartesian_fraction < 0.0)
      {
        abort_with_message(goal_handle, started_at, "computeCartesianPath returned error for LIN primitive");
        return;
      }
    }

    if (planned_trajectory_msg.joint_trajectory.points.empty())
    {
      abort_with_message(goal_handle, started_at, "planner returned empty joint trajectory");
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
      moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(5.0);
      if (!current_state)
      {
        abort_with_message(goal_handle, started_at, "failed to fetch current robot state for post-processing");
        return;
      }

      reference_state = *current_state;
    }

    robot_trajectory::RobotTrajectory robot_trajectory(move_group_->getRobotModel(), kPlanningGroup);
    robot_trajectory.setRobotTrajectoryMsg(reference_state, planned_trajectory_msg);

    std::string post_reason;
    if (robot_trajectory.getWayPointCount() >= 2U)
    {
      if (!trajectory_post_processor_.apply_totg(robot_trajectory, velocity_scale, acceleration_scale, post_reason))
      {
        abort_with_message(goal_handle, started_at, "TOTG failed: " + post_reason);
        return;
      }
    }
    else
    {
      post_reason = "TOTG skipped for single-waypoint trajectory";
    }

    moveit_msgs::msg::RobotTrajectory postprocessed_msg;
    robot_trajectory.getRobotTrajectoryMsg(postprocessed_msg);
    trajectory_msgs::msg::JointTrajectory output_traj = postprocessed_msg.joint_trajectory;

    if (!trajectory_post_processor_.downsample_to_max_points(
          output_traj, TrajectoryPostProcessor::kMaxTrajectoryPoints, post_reason))
    {
      abort_with_message(goal_handle, started_at, "downsampling failed: " + post_reason);
      return;
    }

    std::string ruckig_reason;
    if (!trajectory_post_processor_.apply_ruckig_smoothing(output_traj, ruckig_reason))
    {
      abort_with_message(goal_handle, started_at, "Ruckig smoothing failed: " + ruckig_reason);
      return;
    }

    std::string quality_reason;
    if (!quality_gate_.validate_plan(output_traj, cartesian_fraction, quality_reason))
    {
      abort_with_message(goal_handle, started_at, "quality gate failed: " + quality_reason);
      return;
    }

    publish_feedback(goal_handle, 0.75, "trajectory_execution_requested");

    std::string execution_reason;
    if (!execute_joint_trajectory(output_traj, execution_reason))
    {
      abort_with_message(goal_handle, started_at, "trajectory execution failed: " + execution_reason);
      return;
    }

    publish_feedback(goal_handle, 0.95, "trajectory_execution_complete");

    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = true;

    std::ostringstream message;
    message << "execution success; primitive=" << primitive
            << ", planner_id=" << planner_selection.planner_id
            << ", points=" << output_traj.points.size();

    if (cartesian_fraction >= 0.0)
    {
      message << ", cartesian_fraction=" << cartesian_fraction;
    }

    if (!ruckig_reason.empty())
    {
      message << ", ruckig_status=" << ruckig_reason;
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
