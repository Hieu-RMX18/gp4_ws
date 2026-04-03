// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#include "primitives/primitive_blended_sequence.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <moveit/kinematic_constraints/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "motion_core/orientation_filter.hpp"
#include "motion_core/planner_router.hpp"
#include "motion_core/trajectory_post_processor.hpp"
#include "primitives/primitive_approach.hpp"
#include "primitives/primitive_circ.hpp"
#include "primitives/primitive_home.hpp"
#include "primitives/primitive_lin.hpp"
#include "primitives/primitive_ptp.hpp"
#include "primitives/primitive_retract.hpp"

namespace primitives
{
namespace
{
using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;

constexpr double kWaitForServersTimeoutSec = 10.0;
constexpr double kDefaultVelocityCap = 0.3;
constexpr double kDefaultAccelerationCap = 0.2;
constexpr double kDefaultVelocityScale = 0.3;
constexpr double kDefaultAccelerationScale = 0.2;
constexpr double kSequenceActionServerCheckTimeoutSec = 0.1;
constexpr double kInterimConstraintRadiusM = 1e-4;
constexpr std::size_t kExpectedJointCount = 6U;
constexpr char kSequenceActionName[] = "/sequence_move_group";

bool has_pose_target(const geometry_msgs::msg::Pose & pose)
{
  return pose.position.x != 0.0 ||
         pose.position.y != 0.0 ||
         pose.position.z != 0.0 ||
         pose.orientation.x != 0.0 ||
         pose.orientation.y != 0.0 ||
         pose.orientation.z != 0.0 ||
         pose.orientation.w != 0.0;
}

double sanitize_positive_scale(double value, double fallback)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    return fallback;
  }

  return std::min(value, 1.0);
}

double read_scale_parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  double fallback)
{
  if (!node)
  {
    return fallback;
  }

  if (!node->has_parameter(name))
  {
    node->declare_parameter<double>(name, fallback);
  }

  double value = fallback;
  node->get_parameter(name, value);
  return sanitize_positive_scale(value, fallback);
}

double resolve_scale(double requested, double fallback, double cap)
{
  const double resolved = sanitize_positive_scale(requested, fallback);
  return std::min(resolved, cap);
}

bool is_supported_step_type(PrimitiveType type)
{
  switch (type)
  {
    case PrimitiveType::HOME:
    case PrimitiveType::PTP:
    case PrimitiveType::LIN:
    case PrimitiveType::APPROACH:
    case PrimitiveType::RETRACT:
    case PrimitiveType::CIRC:
      return true;
    case PrimitiveType::BLENDED_SEQUENCE:
    case PrimitiveType::UNKNOWN:
    default:
      return false;
  }
}

PrimitiveResult make_result_failure(PrimitiveFailReason reason, const std::string & message)
{
  PrimitiveResult failure;
  failure.success = false;
  failure.reason = reason;
  failure.message = message;
  return failure;
}

std::string to_string(PrimitiveFailReason reason)
{
  switch (reason)
  {
    case PrimitiveFailReason::UNKNOWN:
      return "UNKNOWN";
    case PrimitiveFailReason::NAMED_TARGET_NOT_FOUND:
      return "NAMED_TARGET_NOT_FOUND";
    case PrimitiveFailReason::IK_FAILED:
      return "IK_FAILED";
    case PrimitiveFailReason::CARTESIAN_FRACTION_LOW:
      return "CARTESIAN_FRACTION_LOW";
    case PrimitiveFailReason::JOINT_COUNT_MISMATCH:
      return "JOINT_COUNT_MISMATCH";
    case PrimitiveFailReason::INVALID_ORIENTATION:
      return "INVALID_ORIENTATION";
    case PrimitiveFailReason::WRIST_FLIP_DETECTED:
      return "WRIST_FLIP_DETECTED";
    case PrimitiveFailReason::TRAJECTORY_TOO_LONG:
      return "TRAJECTORY_TOO_LONG";
    case PrimitiveFailReason::DEGENERATE_GEOMETRY:
      return "DEGENERATE_GEOMETRY";
    case PrimitiveFailReason::QUALITY_GATE_REJECTED:
      return "QUALITY_GATE_REJECTED";
    case PrimitiveFailReason::SUB_PRIMITIVE_FAILED:
      return "SUB_PRIMITIVE_FAILED";
    case PrimitiveFailReason::PLANNING_TIMEOUT:
      return "PLANNING_TIMEOUT";
    case PrimitiveFailReason::WORKSPACE_VIOLATION:
      return "WORKSPACE_VIOLATION";
    case PrimitiveFailReason::INVALID_DISTANCE_PARAM:
      return "INVALID_DISTANCE_PARAM";
    case PrimitiveFailReason::TOTG_FAILED:
      return "TOTG_FAILED";
    default:
      return "UNKNOWN";
  }
}

PrimitiveFailReason map_moveit_error(const int32_t error_code)
{
  if (error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    return PrimitiveFailReason::UNKNOWN;
  }

  if (error_code == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT ||
      error_code == moveit_msgs::msg::MoveItErrorCodes::COMMUNICATION_FAILURE ||
      error_code == moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE)
  {
    return PrimitiveFailReason::PLANNING_TIMEOUT;
  }

  if (error_code == moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION)
  {
    return PrimitiveFailReason::IK_FAILED;
  }

  return PrimitiveFailReason::UNKNOWN;
}

std::string step_failure_message(std::size_t step_index, const PrimitiveResult & sub_result)
{
  std::ostringstream stream;
  stream << "step[" << step_index << "] failed: " << to_string(sub_result.reason);
  if (!sub_result.message.empty())
  {
    stream << " (" << sub_result.message << ")";
  }

  return stream.str();
}

geometry_msgs::msg::Pose offset_along_eef_local_z_world(
  const geometry_msgs::msg::Pose & reference_pose,
  double signed_distance)
{
  geometry_msgs::msg::Pose offset_pose = reference_pose;

  const tf2::Quaternion orientation(
    reference_pose.orientation.x,
    reference_pose.orientation.y,
    reference_pose.orientation.z,
    reference_pose.orientation.w);

  const tf2::Vector3 eef_z_world = tf2::quatRotate(orientation, tf2::Vector3(0.0, 0.0, 1.0));
  offset_pose.position.x += signed_distance * eef_z_world.x();
  offset_pose.position.y += signed_distance * eef_z_world.y();
  offset_pose.position.z += signed_distance * eef_z_world.z();

  return offset_pose;
}

class MoveGroupBlendedSequenceBackend final : public BlendedSequenceExecutionBackend
{
public:
  explicit MoveGroupBlendedSequenceBackend(MoveGroupInterface & mgi)
  : mgi_(mgi),
    node_(std::make_shared<rclcpp::Node>(
      "primitive_blended_sequence_backend",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)))
  {
  }

  bool sequence_action_available() override
  {
    ensure_client();
    return sequence_client_->wait_for_action_server(
      std::chrono::duration<double>(kSequenceActionServerCheckTimeoutSec));
  }

  PrimitiveResult execute_sequence_action(const std::vector<SequenceStep> & steps) override
  {
    ensure_client();

    if (!sequence_client_->wait_for_action_server(std::chrono::duration<double>(kWaitForServersTimeoutSec)))
    {
      return make_result_failure(
        PrimitiveFailReason::PLANNING_TIMEOUT,
        "MoveGroupSequence action server unavailable after 10.0 seconds");
    }

    MoveGroupSequence::Goal goal;
    PrimitiveResult build_failure;
    if (!build_sequence_goal(steps, goal, build_failure))
    {
      return build_failure;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);

    const auto goal_future = sequence_client_->async_send_goal(goal);
    if (executor.spin_until_future_complete(
        goal_future,
        std::chrono::duration<double>(kWaitForServersTimeoutSec)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      executor.remove_node(node_);
      return make_result_failure(
        PrimitiveFailReason::PLANNING_TIMEOUT,
        "MoveGroupSequence goal submission timed out");
    }

    const auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      executor.remove_node(node_);
      return make_result_failure(
        PrimitiveFailReason::PLANNING_TIMEOUT,
        "MoveGroupSequence goal rejected by server");
    }

    const auto result_future = sequence_client_->async_get_result(goal_handle);
    if (executor.spin_until_future_complete(
        result_future,
        std::chrono::duration<double>(kWaitForServersTimeoutSec)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      executor.remove_node(node_);
      return make_result_failure(
        PrimitiveFailReason::PLANNING_TIMEOUT,
        "MoveGroupSequence result timed out");
    }

    executor.remove_node(node_);

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result)
    {
      return make_result_failure(
        PrimitiveFailReason::PLANNING_TIMEOUT,
        "MoveGroupSequence action did not finish successfully");
    }

    const auto & response = wrapped_result.result->response;
    if (response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      return make_result_failure(
        map_moveit_error(response.error_code.val),
        "MoveGroupSequence planning failed with MoveIt error code: " +
        std::to_string(response.error_code.val));
    }

    std::size_t merged_points = 0;
    for (const auto & trajectory : response.planned_trajectories)
    {
      merged_points += trajectory.joint_trajectory.points.size();
    }

    PrimitiveResult success;
    success.success = true;
    success.reason = PrimitiveFailReason::UNKNOWN;
    success.message = "blended_sequence planned with MoveGroupSequenceAction";
    success.planning_time_sec = response.planning_time;
    success.trajectory_points = merged_points;
    return success;
  }

  PrimitiveResult execute_substep(const SequenceStep & step) override
  {
    switch (step.type)
    {
      case PrimitiveType::HOME:
      {
        PrimitiveHome primitive;
        return primitive.execute(mgi_);
      }

      case PrimitiveType::PTP:
      {
        PrimitivePtp primitive;
        PTPGoal goal;
        goal.joint_target = step.joint_target;
        goal.pose_target = step.target_pose;
        goal.velocity_scale = step.velocity_scale;
        goal.acceleration_scale = step.acceleration_scale;
        return primitive.execute(goal, mgi_);
      }

      case PrimitiveType::LIN:
      {
        PrimitiveLin primitive;
        LINGoal goal;
        goal.target_pose = step.target_pose;
        goal.velocity_scale = step.velocity_scale;
        goal.acceleration_scale = step.acceleration_scale;
        return primitive.execute(goal, mgi_);
      }

      case PrimitiveType::APPROACH:
      {
        PrimitiveApproach primitive;
        ApproachGoal goal;
        goal.target_pose = step.target_pose;
        goal.approach_distance = step.approach_distance;
        goal.velocity_scale = step.velocity_scale;
        goal.acceleration_scale = step.acceleration_scale;
        return primitive.execute(goal, mgi_);
      }

      case PrimitiveType::RETRACT:
      {
        PrimitiveRetract primitive;
        RetractGoal goal;
        goal.retract_distance = step.retract_distance;
        goal.velocity_scale = step.velocity_scale;
        goal.acceleration_scale = step.acceleration_scale;
        return primitive.execute(goal, mgi_);
      }

      case PrimitiveType::CIRC:
      {
        PrimitiveCirc primitive;
        CIRCGoal goal;
        goal.auxiliary_point = step.auxiliary_pose;
        goal.goal_pose = step.target_pose;
        goal.velocity_scale = step.velocity_scale;
        goal.acceleration_scale = step.acceleration_scale;
        return primitive.execute(goal, mgi_);
      }

      case PrimitiveType::BLENDED_SEQUENCE:
      case PrimitiveType::UNKNOWN:
      default:
        return make_result_failure(
          PrimitiveFailReason::UNKNOWN,
          "unsupported sequence step type for substep execution");
    }
  }

private:
  struct PathConstraintGuard
  {
    explicit PathConstraintGuard(MoveGroupInterface & mgi)
    : mgi_ref(mgi)
    {
    }

    ~PathConstraintGuard()
    {
      if (enabled)
      {
        mgi_ref.clearPathConstraints();
      }
    }

    void activate()
    {
      enabled = true;
    }

    MoveGroupInterface & mgi_ref;
    bool enabled{ false };
  };

  void ensure_client()
  {
    if (!sequence_client_)
    {
      sequence_client_ = rclcpp_action::create_client<MoveGroupSequence>(node_, kSequenceActionName);
    }
  }

  bool build_sequence_goal(
    const std::vector<SequenceStep> & steps,
    MoveGroupSequence::Goal & goal,
    PrimitiveResult & failure)
  {
    moveit_msgs::msg::MotionSequenceRequest request;
    request.items.clear();
    request.items.reserve(steps.size());

    for (std::size_t index = 0; index < steps.size(); ++index)
    {
      moveit_msgs::msg::MotionSequenceItem item;
      item.blend_radius = steps[index].blend_radius;

      std::string reason;
      PrimitiveFailReason step_reason = PrimitiveFailReason::UNKNOWN;
      if (!build_motion_plan_request(steps[index], item.req, reason, step_reason))
      {
        failure = make_result_failure(step_reason, "step[" + std::to_string(index) + "] failed: " + reason);
        return false;
      }

      request.items.push_back(std::move(item));
    }

    moveit_msgs::msg::PlanningOptions planning_options;
    planning_options.plan_only = true;
    planning_options.replan = false;

    goal.request = std::move(request);
    goal.planning_options = std::move(planning_options);
    return true;
  }

  bool build_motion_plan_request(
    const SequenceStep & step,
    moveit_msgs::msg::MotionPlanRequest & request,
    std::string & reason,
    PrimitiveFailReason & step_reason)
  {
    const auto velocity_cap = read_scale_parameter(
      node_,
      "primitives.max_velocity_scaling_factor",
      kDefaultVelocityCap);
    const auto acceleration_cap = read_scale_parameter(
      node_,
      "primitives.max_acceleration_scaling_factor",
      kDefaultAccelerationCap);
    const auto default_velocity = read_scale_parameter(
      node_,
      "primitives.default_velocity_scaling_factor",
      kDefaultVelocityScale);
    const auto default_acceleration = read_scale_parameter(
      node_,
      "primitives.default_acceleration_scaling_factor",
      kDefaultAccelerationScale);

    const double velocity_scale = resolve_scale(step.velocity_scale, default_velocity, velocity_cap);
    const double acceleration_scale = resolve_scale(
      step.acceleration_scale,
      default_acceleration,
      acceleration_cap);

    mgi_.setMaxVelocityScalingFactor(velocity_scale);
    mgi_.setMaxAccelerationScalingFactor(acceleration_scale);
    mgi_.setStartStateToCurrentState();
    mgi_.clearPoseTargets();
    mgi_.setPoseReferenceFrame("world");

    PathConstraintGuard path_constraint_guard(mgi_);

    std::string planner_id;
    switch (step.type)
    {
      case PrimitiveType::HOME:
      {
        planner_id = planner_router_.route_planner("PTP", false);
        if (!mgi_.setNamedTarget("home"))
        {
          reason = "named target 'home' is unavailable";
          step_reason = PrimitiveFailReason::NAMED_TARGET_NOT_FOUND;
          return false;
        }
        break;
      }

      case PrimitiveType::PTP:
      {
        planner_id = planner_router_.route_planner("PTP", false);
        if (!step.joint_target.empty())
        {
          if (step.joint_target.size() != kExpectedJointCount)
          {
            reason = "PTP joint_target must have exactly 6 values";
            step_reason = PrimitiveFailReason::JOINT_COUNT_MISMATCH;
            return false;
          }

          if (!mgi_.setJointValueTarget(step.joint_target))
          {
            reason = "MoveGroupInterface rejected PTP joint target";
            step_reason = PrimitiveFailReason::UNKNOWN;
            return false;
          }
        }
        else
        {
          if (!has_pose_target(step.target_pose))
          {
            reason = "no target specified";
            step_reason = PrimitiveFailReason::UNKNOWN;
            return false;
          }

          geometry_msgs::msg::Pose pose = step.target_pose;
          if (!orientation_filter_.normalize_and_validate(pose, reason))
          {
            step_reason = PrimitiveFailReason::INVALID_ORIENTATION;
            return false;
          }

          if (!mgi_.setPoseTarget(pose, mgi_.getEndEffectorLink()))
          {
            reason = "MoveGroupInterface rejected PTP pose target";
            step_reason = PrimitiveFailReason::IK_FAILED;
            return false;
          }
        }
        break;
      }

      case PrimitiveType::LIN:
      {
        planner_id = planner_router_.route_planner("LIN", false);

        geometry_msgs::msg::Pose pose = step.target_pose;
        if (!orientation_filter_.normalize_and_validate(pose, reason))
        {
          step_reason = PrimitiveFailReason::INVALID_ORIENTATION;
          return false;
        }

        if (!mgi_.setPoseTarget(pose, mgi_.getEndEffectorLink()))
        {
          reason = "MoveGroupInterface rejected LIN target pose";
          step_reason = PrimitiveFailReason::UNKNOWN;
          return false;
        }
        break;
      }

      case PrimitiveType::APPROACH:
      {
        planner_id = planner_router_.route_planner("LIN", false);

        if (step.approach_distance <= 0.0)
        {
          reason = "approach_distance must be greater than 0";
          step_reason = PrimitiveFailReason::INVALID_DISTANCE_PARAM;
          return false;
        }

        geometry_msgs::msg::Pose pose = step.target_pose;
        if (!orientation_filter_.normalize_and_validate(pose, reason))
        {
          step_reason = PrimitiveFailReason::INVALID_ORIENTATION;
          return false;
        }

        if (!mgi_.setPoseTarget(pose, mgi_.getEndEffectorLink()))
        {
          reason = "MoveGroupInterface rejected APPROACH target pose";
          step_reason = PrimitiveFailReason::UNKNOWN;
          return false;
        }
        break;
      }

      case PrimitiveType::RETRACT:
      {
        planner_id = planner_router_.route_planner("LIN", false);

        if (step.retract_distance <= 0.0)
        {
          reason = "retract_distance must be greater than 0";
          step_reason = PrimitiveFailReason::INVALID_DISTANCE_PARAM;
          return false;
        }

        auto current_pose = mgi_.getCurrentPose(mgi_.getEndEffectorLink()).pose;
        if (!orientation_filter_.normalize_and_validate(current_pose, reason))
        {
          step_reason = PrimitiveFailReason::INVALID_ORIENTATION;
          return false;
        }

        const auto retract_pose = offset_along_eef_local_z_world(current_pose, step.retract_distance);

        if (!mgi_.setPoseTarget(retract_pose, mgi_.getEndEffectorLink()))
        {
          reason = "MoveGroupInterface rejected RETRACT target pose";
          step_reason = PrimitiveFailReason::UNKNOWN;
          return false;
        }
        break;
      }

      case PrimitiveType::CIRC:
      {
        planner_id = "CIRC";

        geometry_msgs::msg::Pose auxiliary = step.auxiliary_pose;
        if (!orientation_filter_.normalize_and_validate(auxiliary, reason))
        {
          step_reason = PrimitiveFailReason::INVALID_ORIENTATION;
          return false;
        }

        geometry_msgs::msg::Pose goal_pose = step.target_pose;
        if (!orientation_filter_.normalize_and_validate(goal_pose, reason))
        {
          step_reason = PrimitiveFailReason::INVALID_ORIENTATION;
          return false;
        }

        moveit_msgs::msg::Constraints path_constraints;
        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id = "world";
        pc.link_name = mgi_.getEndEffectorLink();
        if (pc.link_name.empty())
        {
          reason = "end effector link is empty; cannot set CIRC interim constraint";
          step_reason = PrimitiveFailReason::UNKNOWN;
          return false;
        }

        pc.weight = 1.0;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        primitive.dimensions = { kInterimConstraintRadiusM };

        geometry_msgs::msg::Pose primitive_pose;
        primitive_pose.orientation.w = 1.0;
        primitive_pose.position = auxiliary.position;

        pc.constraint_region.primitives.push_back(primitive);
        pc.constraint_region.primitive_poses.push_back(primitive_pose);

        // Non-negotiable PILZ CIRC convention:
        // Use path_constraints name "interim" (on-arc waypoint), NOT "center".
        // PILZ CIRC with "center" always selects the shorter arc between start and goal,
        // which is not controllable and can produce unexpected obstacle-near motion.
        // Using "interim" forces passage through an explicit on-arc point and gives
        // deterministic control over which arc segment is executed.
        path_constraints.name = "interim";
        path_constraints.position_constraints.push_back(pc);

        mgi_.setPathConstraints(path_constraints);
        path_constraint_guard.activate();

        if (!mgi_.setPoseTarget(goal_pose, mgi_.getEndEffectorLink()))
        {
          reason = "MoveGroupInterface rejected CIRC goal pose target";
          step_reason = PrimitiveFailReason::UNKNOWN;
          return false;
        }

        break;
      }

      case PrimitiveType::BLENDED_SEQUENCE:
      case PrimitiveType::UNKNOWN:
      default:
        reason = "unsupported sequence step type";
        step_reason = PrimitiveFailReason::UNKNOWN;
        return false;
    }

    if (planner_id.empty())
    {
      reason = "planner router did not resolve planner id";
      step_reason = PrimitiveFailReason::UNKNOWN;
      return false;
    }

    mgi_.setPlannerId(planner_id);
    mgi_.constructMotionPlanRequest(request);

    request.group_name = mgi_.getName();
    request.planner_id = planner_id;
    request.max_velocity_scaling_factor = velocity_scale;
    request.max_acceleration_scaling_factor = acceleration_scale;
    request.allowed_planning_time = std::max(mgi_.getPlanningTime(), 1.0);
    request.num_planning_attempts = 1;

    reason.clear();
    step_reason = PrimitiveFailReason::UNKNOWN;
    return true;
  }

  MoveGroupInterface & mgi_;
  rclcpp::Node::SharedPtr node_;

  motion_core::PlannerRouter planner_router_;
  motion_core::OrientationFilter orientation_filter_;
  rclcpp_action::Client<MoveGroupSequence>::SharedPtr sequence_client_;
};
}  // namespace

PrimitiveResult PrimitiveBlendedSequence::execute(const std::vector<SequenceStep> & steps, MoveGroupInterface & mgi)
{
  MoveGroupBlendedSequenceBackend backend(mgi);
  return execute(steps, backend);
}

PrimitiveResult PrimitiveBlendedSequence::execute(
  const std::vector<SequenceStep> & steps,
  BlendedSequenceExecutionBackend & backend)
{
  if (steps.empty())
  {
    return make_failure(
      PrimitiveFailReason::UNKNOWN,
      "blended_sequence requires at least one step");
  }

  for (std::size_t index = 0; index < steps.size(); ++index)
  {
    if (steps[index].blend_radius < 0.0)
    {
      return make_failure(
        PrimitiveFailReason::UNKNOWN,
        "step[" + std::to_string(index) + "] failed: invalid blend_radius < 0");
    }

    if (!is_supported_step_type(steps[index].type))
    {
      return make_failure(
        PrimitiveFailReason::UNKNOWN,
        "step[" + std::to_string(index) + "] failed: unsupported primitive type " +
        to_string(steps[index].type));
    }
  }

  if (backend.sequence_action_available())
  {
    return backend.execute_sequence_action(steps);
  }

  std::size_t merged_points = 0;
  double total_planning_time = 0.0;

  for (std::size_t index = 0; index < steps.size(); ++index)
  {
    const PrimitiveResult sub_result = backend.execute_substep(steps[index]);
    if (!sub_result.success)
    {
      return make_failure(
        PrimitiveFailReason::SUB_PRIMITIVE_FAILED,
        step_failure_message(index, sub_result));
    }

    total_planning_time += sub_result.planning_time_sec;
    merged_points += sub_result.trajectory_points;

    if (merged_points > motion_core::TrajectoryPostProcessor::kMaxTrajectoryPoints)
    {
      return make_failure(
        PrimitiveFailReason::TRAJECTORY_TOO_LONG,
        "merged staged trajectory has " + std::to_string(merged_points) +
        " points (> 200)");
    }
  }

  PrimitiveResult success;
  success.success = true;
  success.reason = PrimitiveFailReason::UNKNOWN;
  success.message = "blended_sequence executed in degraded mode (staged)";
  success.planning_time_sec = total_planning_time;
  success.trajectory_points = merged_points;
  return success;
}

PrimitiveResult PrimitiveBlendedSequence::execute(const ExecuteMotionGoal & goal, MoveGroupInterface & mgi)
{
  (void)goal;
  (void)mgi;

  return make_failure(
    PrimitiveFailReason::UNKNOWN,
    "ExecuteMotion goal for BLENDED_SEQUENCE lacks sequence steps; use typed SequenceStep API");
}
}  // namespace primitives
