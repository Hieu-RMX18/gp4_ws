#include "motion_core/primitive_router_dispatch.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "motion_core/execute_motion_action_support.hpp"
#include "motion_core/move_rel_validator.hpp"
#include "motion_core/trajectory_assembler.hpp"

#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace motion_core {
PrimitiveRouterDispatch::PrimitiveRouterDispatch(
    rclcpp::Logger logger,
    std::function<
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface>()>
        move_group_provider,
    const PlannerRouter &planner_router,
    const OrientationFilter &orientation_filter, SeedManager &seed_manager,
    IkSelector &ik_selector, TrajectoryPostProcessor &trajectory_post_processor,
    ReadCurrentTcpPoseFn read_current_tcp_pose,
    JointPositionGuard joint_position_guard)
    : logger_(std::move(logger)),
      move_group_provider_(std::move(move_group_provider)),
      planner_router_(planner_router), orientation_filter_(orientation_filter),
      seed_manager_(seed_manager), ik_selector_(ik_selector),
      trajectory_post_processor_(trajectory_post_processor),
      read_current_tcp_pose_(std::move(read_current_tcp_pose)),
      joint_position_guard_(std::move(joint_position_guard)) {}

PrimitiveRouterDispatch::PlannerSelection
PrimitiveRouterDispatch::resolve_planner_selection(
    const std::string &planner_id) {
  std::string normalized = planner_id;
  normalized.erase(std::remove_if(normalized.begin(), normalized.end(),
                                  [](unsigned char c) {
                                    return std::isspace(c) != 0 || c == '_' ||
                                           c == '-';
                                  }),
                   normalized.end());

  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

  if (normalized == "PILZLIN" || normalized == "LIN") {
    return {"pilz_industrial_motion_planner", "LIN"};
  }

  if (normalized == "PILZPTP" || normalized == "PTP") {
    return {"pilz_industrial_motion_planner", "PTP"};
  }

  if (normalized == "PILZCIRC" || normalized == "CIRC") {
    return {"pilz_industrial_motion_planner", "CIRC"};
  }

  if (normalized == "OMPLRRTCONNECT" || normalized == "RRTCONNECT") {
    return {"ompl", "RRTConnect"};
  }

  return {"", planner_id};
}

bool PrimitiveRouterDispatch::is_pose_goal_required(
    const std::string &primitive, bool has_joint_target) {
  if (primitive == "LIN") {
    return true;
  }

  if (primitive == "PTP") {
    return !has_joint_target;
  }

  return false;
}

double PrimitiveRouterDispatch::quaternion_norm_sq(
    const geometry_msgs::msg::Quaternion &q) {
  return (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
}

double
PrimitiveRouterDispatch::max_abs_value(const std::vector<double> &values) {
  double max_value = 0.0;
  for (const double value : values) {
    max_value = std::max(max_value, std::abs(value));
  }
  return max_value;
}

std::string PrimitiveRouterDispatch::format_joint_vector(
    const std::vector<double> &joints) {
  std::ostringstream stream;
  stream << "[";
  for (std::size_t index = 0; index < joints.size(); ++index) {
    if (index > 0U) {
      stream << ", ";
    }
    stream << std::fixed << std::setprecision(4) << joints[index];
  }
  stream << "]";
  return stream.str();
}

void PrimitiveRouterDispatch::log_joint_branch_selection(
    const std::string &primitive, std::uint64_t sequence,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &current, const std::vector<double> &requested,
    const BranchPreservedJointVectorResult &branch_result) const {
  RCLCPP_INFO(logger_,
              "%s goal_seq=%lu branch-preserved target selection: current=%s "
              "requested=%s chosen=%s max_abs_delta=%.4f",
              primitive.c_str(), static_cast<unsigned long>(sequence),
              format_joint_vector(current).c_str(),
              format_joint_vector(requested).c_str(),
              format_joint_vector(branch_result.chosen_targets).c_str(),
              max_abs_value(branch_result.deltas_from_current));

  for (std::size_t index = 0; index < branch_result.chosen_targets.size();
       ++index) {
    const std::string joint_name = index < joint_names.size()
                                       ? joint_names[index]
                                       : ("joint_" + std::to_string(index));
    RCLCPP_DEBUG(logger_,
                 "%s goal_seq=%lu joint=%s current=%.6f requested=%.6f "
                 "chosen=%.6f delta=%.6f helper=%s",
                 primitive.c_str(), static_cast<unsigned long>(sequence),
                 joint_name.c_str(), current[index], requested[index],
                 branch_result.chosen_targets[index],
                 branch_result.deltas_from_current[index],
                 branch_result.helper_used[index].c_str());
  }
}

PrimitiveRouterDispatch::PlanningResult
PrimitiveRouterDispatch::post_process_trajectory(
    const moveit_msgs::msg::RobotTrajectory &planned_trajectory_msg,
    const moveit_msgs::msg::RobotState &plan_start_state_msg,
    bool has_plan_start_state,
    const moveit::core::RobotState &current_robot_state, double velocity_scale,
    double acceleration_scale, const std::string &start_state_failure_message,
    const std::string &time_parameterization_failure_prefix) const {
  PlanningResult result;
  const auto move_group = move_group_provider_();
  if (!move_group) {
    result.reason = "MoveGroup unavailable during post-processing";
    return result;
  }

  moveit::core::RobotState reference_state(move_group->getRobotModel());
  if (has_plan_start_state) {
    if (!moveit::core::robotStateMsgToRobotState(plan_start_state_msg,
                                                 reference_state, true)) {
      result.reason = start_state_failure_message;
      return result;
    }
  } else {
    reference_state = current_robot_state;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(
      move_group->getRobotModel(), kPlanningGroup);
  robot_trajectory.setRobotTrajectoryMsg(reference_state,
                                         planned_trajectory_msg);

  std::string ruckig_reason;
  const bool ruckig_ok = trajectory_post_processor_.apply_ruckig_smoothing(
      robot_trajectory, velocity_scale, acceleration_scale, ruckig_reason);
  const bool ruckig_applied =
      ruckig_ok && ruckig_reason.find("unavailable") == std::string::npos &&
      ruckig_reason.find("skipped") == std::string::npos;

  if (ruckig_applied) {
    result.time_parameterization_note = "time_parameterization=ruckig";
  } else {
    if (robot_trajectory.getWayPointCount() >= 2U) {
      std::string totg_reason;
      if (!trajectory_post_processor_.apply_totg(
              robot_trajectory, velocity_scale, acceleration_scale,
              totg_reason)) {
        const std::string failure_detail =
            ruckig_reason.empty() ? "Ruckig unavailable"
                                  : ("Ruckig status: " + ruckig_reason);
        result.reason = time_parameterization_failure_prefix + failure_detail +
                        "; TOTG fallback failed: " + totg_reason;
        return result;
      }
      result.time_parameterization_note = "time_parameterization=totg_fallback";
    } else {
      result.time_parameterization_note =
          "time_parameterization=none_single_waypoint";
    }
  }

  moveit_msgs::msg::RobotTrajectory postprocessed_msg;
  robot_trajectory.getRobotTrajectoryMsg(postprocessed_msg);
  result.trajectory = postprocessed_msg.joint_trajectory;
  result.ruckig_reason = ruckig_reason;
  result.status = PlanningStatus::kSuccess;
  return result;
}

PrimitiveRouterDispatch::PlanningResult
PrimitiveRouterDispatch::plan_for_primitive(const PlanningRequest &request) {
  PlanningResult result;
  if (!request.goal) {
    result.reason = "missing execute_motion goal";
    return result;
  }
  if (!request.plan_with_interruption) {
    result.reason = "missing planning callback";
    return result;
  }
  if (!request.interrupt_reason) {
    result.reason = "missing interrupt callback";
    return result;
  }

  const auto move_group = move_group_provider_();
  if (!move_group) {
    result.reason = "MoveGroup unavailable";
    return result;
  }

  const auto &goal = request.goal;
  const std::string &primitive = request.primitive;
  const std::string &effective_primitive = request.effective_primitive;

  const std::string planning_interrupt =
      request.interrupt_reason("planning_setup");
  if (!planning_interrupt.empty()) {
    result.status = PlanningStatus::kCanceled;
    result.reason = planning_interrupt;
    return result;
  }

  if (primitive == "MOVE_JOINT") {
    const int joint_idx = goal->joint_index;
    const double target_angle = goal->joint_angle;
    if (joint_idx < 0 || static_cast<std::size_t>(joint_idx) >=
                             request.current_joint_positions.size()) {
      result.reason =
          "MOVE_JOINT: joint_index " + std::to_string(joint_idx) +
          " out of range [0, " +
          std::to_string(request.current_joint_positions.size() - 1U) + "]";
      return result;
    }

    std::vector<double> requested_joint_positions =
        request.current_joint_positions;
    requested_joint_positions[static_cast<std::size_t>(joint_idx)] =
        target_angle;
    const auto branch_result = choose_branch_preserved_joint_vector(
        request.active_joint_models, request.current_joint_positions,
        requested_joint_positions);
    if (!branch_result.success) {
      result.reason =
          "MOVE_JOINT branch preservation failed: " + branch_result.reason;
      return result;
    }

    log_joint_branch_selection("MOVE_JOINT", request.goal_sequence,
                               request.active_joint_names,
                               request.current_joint_positions,
                               requested_joint_positions, branch_result);

    move_group->setPlanningTime(kPlanningTimeSec);
    const std::string ptp_planner = planner_router_.route_planner("PTP", false);
    const PlannerSelection ptp_selection = resolve_planner_selection(
        ptp_planner.empty() ? "PILZ_PTP" : ptp_planner);
    RCLCPP_INFO(
        logger_,
        "MOVE_JOINT goal_seq=%lu planner_selected pipeline=%s planner=%s",
        static_cast<unsigned long>(request.goal_sequence),
        ptp_selection.pipeline_id.empty() ? "<default>"
                                          : ptp_selection.pipeline_id.c_str(),
        ptp_selection.planner_id.c_str());
    if (!ptp_selection.pipeline_id.empty()) {
      move_group->setPlanningPipelineId(ptp_selection.pipeline_id);
    }
    move_group->setPlannerId(ptp_selection.planner_id);
    move_group->setMaxVelocityScalingFactor(request.velocity_scale);
    move_group->setMaxAccelerationScalingFactor(request.acceleration_scale);
    move_group->setStartState(request.current_robot_state);
    move_group->clearPoseTargets();

    if (!move_group->setJointValueTarget(branch_result.chosen_targets)) {
      result.reason = "MOVE_JOINT: failed to set joint target for PTP planning";
      return result;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result =
        request.plan_with_interruption(plan, "MOVE_JOINT planning");
    if (plan_result.status == PlanningStatus::kCanceled) {
      result.status = PlanningStatus::kCanceled;
      result.reason = plan_result.reason;
      return result;
    }
    if (plan_result.status != PlanningStatus::kSuccess) {
      result.reason = "MOVE_JOINT: " + plan_result.reason + " for joint[" +
                      std::to_string(joint_idx) + "] -> " +
                      std::to_string(target_angle);
      return result;
    }

    result = post_process_trajectory(
        plan.trajectory_, plan.start_state_, true, request.current_robot_state,
        request.velocity_scale, request.acceleration_scale,
        "MOVE_JOINT: failed to convert plan start_state",
        "MOVE_JOINT time parameterization failed; ");
    if (result.status != PlanningStatus::kSuccess) {
      return result;
    }

    result.dispatch_primitive = primitive;
    result.planner_id = goal->planner_id.empty() ? "PTP" : goal->planner_id;
    result.is_move_joint = true;
    result.move_joint_index = joint_idx;
    result.move_joint_target_angle = target_angle;
    return result;
  }

  if (primitive == "BLENDED_SEQUENCE") {
    return plan_blended_sequence(request, move_group);
  }

  std::string planner_id = goal->planner_id;
  if (planner_id.empty()) {
    planner_id = planner_router_.route_planner(
        (effective_primitive == "HOME")       ? "PTP"
        : (effective_primitive == "MOVE_REL") ? "LIN"
        : (effective_primitive == "CIRC")     ? "CIRC"
                                              : effective_primitive,
        false);
  }

  if (planner_id.empty()) {
    result.reason = "unable to resolve planner_id for primitive " + primitive;
    return result;
  }

  const PlannerSelection planner_selection =
      resolve_planner_selection(planner_id);
  RCLCPP_INFO(logger_,
              "execute_motion goal_seq=%lu planner_selected primitive=%s "
              "pipeline=%s planner=%s",
              static_cast<unsigned long>(request.goal_sequence),
              effective_primitive.c_str(),
              planner_selection.pipeline_id.empty()
                  ? "<default>"
                  : planner_selection.pipeline_id.c_str(),
              planner_selection.planner_id.c_str());

  move_group->setPlanningTime(kPlanningTimeSec);
  if (!planner_selection.pipeline_id.empty()) {
    move_group->setPlanningPipelineId(planner_selection.pipeline_id);
  }
  move_group->setPlannerId(planner_selection.planner_id);
  move_group->setMaxVelocityScalingFactor(request.velocity_scale);
  move_group->setMaxAccelerationScalingFactor(request.acceleration_scale);
  move_group->setStartState(request.current_robot_state);
  move_group->clearPoseTargets();

  geometry_msgs::msg::Pose normalized_pose;
  const bool has_joint_target = !goal->joint_target.empty();
  bool pose_required =
      is_pose_goal_required(effective_primitive, has_joint_target);
  bool move_rel_resolved = false;

  if (effective_primitive == "MOVE_REL") {
    std::string rel_reason;
    if (!validate_move_rel_frame(goal->reference_frame, rel_reason)) {
      result.reason = rel_reason;
      return result;
    }

    const double dx = goal->delta_x;
    const double dy = goal->delta_y;
    const double dz = goal->delta_z;
    if (!validate_move_rel_deltas(dx, dy, dz, rel_reason)) {
      result.reason = rel_reason;
      return result;
    }

    geometry_msgs::msg::PoseStamped current_stamped;
    if (!read_current_tcp_pose_(current_stamped, rel_reason, 5.0)) {
      result.reason =
          "MOVE_REL: failed to read current TCP pose: " + rel_reason;
      return result;
    }

    const auto &current = current_stamped.pose;
    if (quaternion_norm_sq(current.orientation) <= 1e-12) {
      result.reason = "MOVE_REL: current pose has invalid orientation; cannot "
                      "proceed safely";
      return result;
    }

    normalized_pose = compute_move_rel_target(current, dx, dy, dz);
    if (!validate_move_rel_target_bounds(normalized_pose, rel_reason)) {
      result.reason = rel_reason;
      return result;
    }

    RCLCPP_INFO(logger_,
                "MOVE_REL resolved: delta=(%.4f, %.4f, %.4f), current=(%.4f, "
                "%.4f, %.4f), target=(%.4f, %.4f, %.4f)",
                dx, dy, dz, current.position.x, current.position.y,
                current.position.z, normalized_pose.position.x,
                normalized_pose.position.y, normalized_pose.position.z);

    move_rel_resolved = true;
    pose_required = true;
  }

  if (pose_required) {
    if (!move_rel_resolved) {
      normalized_pose = goal->target_pose;
      if (quaternion_norm_sq(normalized_pose.orientation) <= 1e-12) {
        geometry_msgs::msg::PoseStamped current_stamped;
        std::string current_pose_reason;
        if (!read_current_tcp_pose_(current_stamped, current_pose_reason,
                                    5.0)) {
          result.reason = "orientation unresolved: command omitted orientation "
                          "and current pose is unavailable: " +
                          current_pose_reason;
          return result;
        }
        normalized_pose.orientation = current_stamped.pose.orientation;
        RCLCPP_WARN(logger_, "Goal orientation omitted; using current "
                             "end-effector orientation for deterministic IK.");
      }
    }

    std::string orientation_reason;
    if (!orientation_filter_.normalize_and_validate(normalized_pose,
                                                    orientation_reason)) {
      result.reason = "orientation rejected: " + orientation_reason;
      return result;
    }

    std::vector<double> seed_state;
    if (!seed_manager_.get_seed_state(effective_primitive, seed_state)) {
      result.reason =
          "IK seed unavailable: /yaskawa/joint_states missing/stale and "
          "fallback seed is disabled or unavailable";
      return result;
    }

    std::vector<double> ik_solution;
    std::string ik_reason;
    if (!ik_selector_.solve_ik(normalized_pose, seed_state, ik_solution,
                               ik_reason)) {
      result.reason = "IK solve failed: " + ik_reason;
      return result;
    }

    if (effective_primitive == "PTP") {
      const auto branch_result = choose_branch_preserved_joint_vector(
          request.active_joint_models, request.current_joint_positions,
          ik_solution);
      if (!branch_result.success) {
        result.reason = "failed to branch-preserve IK-derived PTP target: " +
                        branch_result.reason;
        return result;
      }

      log_joint_branch_selection(
          "PTP", request.goal_sequence, request.active_joint_names,
          request.current_joint_positions, ik_solution, branch_result);

      RCLCPP_INFO(logger_,
                  "execute_motion goal_seq=%lu IK-derived PTP target "
                  "current_seed=%s ik_solution=%s",
                  static_cast<unsigned long>(request.goal_sequence),
                  format_joint_vector(seed_state).c_str(),
                  format_joint_vector(ik_solution).c_str());
      if (!move_group->setJointValueTarget(branch_result.chosen_targets)) {
        result.reason = "failed to set IK-derived joint target for PTP";
        return result;
      }
    }
  }

  const std::string pre_plan_interrupt = request.interrupt_reason("pre_plan");
  if (!pre_plan_interrupt.empty()) {
    result.status = PlanningStatus::kCanceled;
    result.reason = pre_plan_interrupt;
    return result;
  }

  moveit_msgs::msg::RobotTrajectory planned_trajectory_msg;
  moveit_msgs::msg::RobotState plan_start_state_msg;
  bool has_plan_start_state = false;
  double cartesian_fraction = QualityGate::kFractionNotApplicable;

  if (effective_primitive == "CIRC") {
    if (goal->waypoints.empty()) {
      result.reason = "CIRC requires at least 1 auxiliary waypoint";
      return result;
    }

    geometry_msgs::msg::Pose aux_pose = goal->waypoints[0];
    geometry_msgs::msg::Pose final_pose = goal->target_pose;
    if (quaternion_norm_sq(final_pose.orientation) <= 1e-12) {
      geometry_msgs::msg::PoseStamped current_stamped;
      std::string current_reason;
      if (!read_current_tcp_pose_(current_stamped, current_reason, 5.0)) {
        result.reason = "CIRC: cannot resolve orientation for final pose: " +
                        current_reason;
        return result;
      }
      final_pose.orientation = current_stamped.pose.orientation;
    }
    if (quaternion_norm_sq(aux_pose.orientation) <= 1e-12) {
      aux_pose.orientation = final_pose.orientation;
    }

    auto ensure_unit_quaternion = [&](geometry_msgs::msg::Pose &pose,
                                      const char *which) -> bool {
      const double n2 = quaternion_norm_sq(pose.orientation);
      if (n2 < 0.98 || n2 > 1.02) {
        result.reason = std::string("CIRC: ") + which +
                        " orientation is not a unit quaternion (norm^2=" +
                        std::to_string(n2) + ")";
        return false;
      }
      const double n = std::sqrt(n2);
      pose.orientation.x /= n;
      pose.orientation.y /= n;
      pose.orientation.z /= n;
      pose.orientation.w /= n;
      return true;
    };
    if (!ensure_unit_quaternion(final_pose, "target_pose")) {
      return result;
    }
    if (!ensure_unit_quaternion(aux_pose, "auxiliary waypoint")) {
      return result;
    }

    if (planner_selection.pipeline_id != "pilz_industrial_motion_planner" ||
        planner_selection.planner_id != "CIRC") {
      result.reason = std::string("CIRC: planner routing failed, expected pilz "
                                  "CIRC got pipeline='") +
                      planner_selection.pipeline_id + "' planner='" +
                      planner_selection.planner_id + "'";
      return result;
    }

    RCLCPP_INFO(logger_, "CIRC: planning arc via Pilz CIRC planner");

    // Pilz CIRC requires an interim path constraint, NOT setPoseTargets.
    const std::string ee_link = move_group->getEndEffectorLink();
    if (ee_link.empty()) {
      result.reason =
          "CIRC: end effector link is empty; cannot set interim constraint";
      return result;
    }

    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::PositionConstraint pc;
    pc.header.frame_id = "world";
    pc.link_name = ee_link;
    pc.weight = 1.0;

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions = {1e-4};

    geometry_msgs::msg::Pose sphere_pose;
    sphere_pose.orientation.w = 1.0;
    sphere_pose.position = aux_pose.position;

    pc.constraint_region.primitives.push_back(sphere);
    pc.constraint_region.primitive_poses.push_back(sphere_pose);

    // Non-negotiable PILZ CIRC convention:
    // Use path_constraints name "interim" (on-arc waypoint), NOT "center".
    // "center" selects the shorter arc unpredictably; "interim" forces passage
    // through the explicit on-arc point for deterministic control.
    path_constraints.name = "interim";
    path_constraints.position_constraints.push_back(pc);
    move_group->setPathConstraints(path_constraints);

    if (!move_group->setPoseTarget(final_pose, ee_link)) {
      move_group->clearPathConstraints();
      result.reason = "CIRC: MoveGroupInterface rejected goal pose target";
      return result;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result =
        request.plan_with_interruption(plan, "CIRC planning");
    move_group->clearPathConstraints();

    if (plan_result.status == PlanningStatus::kCanceled) {
      result.status = PlanningStatus::kCanceled;
      result.reason = plan_result.reason;
      return result;
    }
    if (plan_result.status != PlanningStatus::kSuccess) {
      result.reason = "CIRC: Pilz CIRC planning failed: " + plan_result.reason;
      return result;
    }

    planned_trajectory_msg = plan.trajectory_;
    plan_start_state_msg = plan.start_state_;
    has_plan_start_state = true;
    cartesian_fraction = 1.0;
    RCLCPP_INFO(logger_, "CIRC: planned fraction=%.3f, points=%zu",
                cartesian_fraction,
                planned_trajectory_msg.joint_trajectory.points.size());
  } else if (effective_primitive == "CARTESIAN_PATH") {
    if (goal->waypoints.empty()) {
      result.reason = "CARTESIAN_PATH requires non-empty waypoints array";
      return result;
    }

    RCLCPP_INFO(logger_,
                "CARTESIAN_PATH: planning smooth path through %zu waypoints",
                goal->waypoints.size());

    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
    cartesian_waypoints.reserve(goal->waypoints.size());
    for (const auto &waypoint : goal->waypoints) {
      cartesian_waypoints.push_back(waypoint);
    }

    cartesian_fraction = move_group->computeCartesianPath(
        cartesian_waypoints, kCartesianEefStep, kCartesianJumpThreshold,
        planned_trajectory_msg, true);
    if (cartesian_fraction < 0.0) {
      result.reason = "CARTESIAN_PATH: computeCartesianPath failed";
      return result;
    }

    RCLCPP_INFO(logger_,
                "CARTESIAN_PATH: planned with fraction=%.3f, points=%zu",
                cartesian_fraction,
                planned_trajectory_msg.joint_trajectory.points.size());
  } else if (effective_primitive == "HOME") {
    if (!move_group->setNamedTarget("home")) {
      result.reason = "HOME: named target 'home' unavailable in SRDF";
      return result;
    }

    move_group->setPlanningTime(kPlanningTimeSec);
    const std::string home_planner =
        planner_router_.route_planner("PTP", false);
    const PlannerSelection home_selection = resolve_planner_selection(
        home_planner.empty() ? "PILZ_PTP" : home_planner);
    if (!home_selection.pipeline_id.empty()) {
      move_group->setPlanningPipelineId(home_selection.pipeline_id);
    }
    move_group->setPlannerId(home_selection.planner_id);
    move_group->setMaxVelocityScalingFactor(request.velocity_scale);
    move_group->setMaxAccelerationScalingFactor(request.acceleration_scale);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result =
        request.plan_with_interruption(plan, "HOME planning");
    if (plan_result.status == PlanningStatus::kCanceled) {
      result.status = PlanningStatus::kCanceled;
      result.reason = plan_result.reason;
      return result;
    }
    if (plan_result.status != PlanningStatus::kSuccess) {
      result.reason =
          "planning failed for HOME primitive: " + plan_result.reason;
      return result;
    }
    planned_trajectory_msg = plan.trajectory_;
    plan_start_state_msg = plan.start_state_;
    has_plan_start_state = true;
  } else if (effective_primitive == "PTP") {
    if (has_joint_target) {
      const auto branch_result = choose_branch_preserved_joint_vector(
          request.active_joint_models, request.current_joint_positions,
          goal->joint_target);
      if (!branch_result.success) {
        result.reason = "invalid branch-preserved joint_target for PTP goal: " +
                        branch_result.reason;
        return result;
      }

      log_joint_branch_selection(
          "PTP", request.goal_sequence, request.active_joint_names,
          request.current_joint_positions, goal->joint_target, branch_result);

      if (!move_group->setJointValueTarget(branch_result.chosen_targets)) {
        result.reason = "invalid joint_target for PTP goal";
        return result;
      }
    } else {
      move_group->setStartState(request.current_robot_state);
      move_group->clearPoseTargets();
    }

    RCLCPP_INFO(
        logger_,
        "execute_motion goal_seq=%lu PTP planning context: planner_id=%s "
        "pipeline=%s start_state=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        static_cast<unsigned long>(request.goal_sequence),
        move_group->getPlannerId().c_str(),
        move_group->getPlanningPipelineId().c_str(),
        request.current_joint_positions[0], request.current_joint_positions[1],
        request.current_joint_positions[2], request.current_joint_positions[3],
        request.current_joint_positions[4], request.current_joint_positions[5]);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result =
        request.plan_with_interruption(plan, "PTP planning");
    RCLCPP_INFO(
        logger_,
        "execute_motion goal_seq=%lu PTP plan() returned status=%d reason='%s'",
        static_cast<unsigned long>(request.goal_sequence),
        static_cast<int>(plan_result.status), plan_result.reason.c_str());
    if (plan_result.status == PlanningStatus::kCanceled) {
      result.status = PlanningStatus::kCanceled;
      result.reason = plan_result.reason;
      return result;
    }
    if (plan_result.status != PlanningStatus::kSuccess) {
      result.reason =
          "planning failed for PTP primitive: " + plan_result.reason;
      return result;
    }
    planned_trajectory_msg = plan.trajectory_;
    plan_start_state_msg = plan.start_state_;
    has_plan_start_state = true;
  } else if (effective_primitive == "LIN" ||
             effective_primitive == "MOVE_REL") {
    if (planner_selection.pipeline_id == "pilz_industrial_motion_planner" &&
        planner_selection.planner_id == "LIN") {
      move_group->setPoseTarget(normalized_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_result = request.plan_with_interruption(
          plan, effective_primitive + " planning");
      if (plan_result.status == PlanningStatus::kCanceled) {
        result.status = PlanningStatus::kCanceled;
        result.reason = plan_result.reason;
        return result;
      }
      if (plan_result.status != PlanningStatus::kSuccess) {
        RCLCPP_ERROR(logger_,
                     "Pilz LIN planning failed for goal_seq=%lu primitive=LIN "
                     "planner=%s: %s. "
                     "No fallback. Caller must replan or retry with PTP.",
                     static_cast<unsigned long>(request.goal_sequence),
                     planner_selection.planner_id.c_str(),
                     plan_result.reason.c_str());
        result.reason =
            std::string("Pilz LIN failed (no fallback): ") + plan_result.reason;
        return result;
      } else {
        planned_trajectory_msg = plan.trajectory_;
        plan_start_state_msg = plan.start_state_;
        has_plan_start_state = true;
        cartesian_fraction = 1.0;
      }
    } else {
      RCLCPP_ERROR(
          logger_,
          "LIN primitive requires Pilz planner but planner_id=%s is not LIN. "
          "No computeCartesianPath fallback. goal_seq=%lu",
          planner_selection.planner_id.c_str(),
          static_cast<unsigned long>(request.goal_sequence));
      result.reason = "LIN primitive requires Pilz LIN planner (no fallback)";
      return result;
    }
  } else {
    result.reason = "unsupported planning primitive: " + effective_primitive;
    return result;
  }

  if (planned_trajectory_msg.joint_trajectory.points.empty()) {
    result.reason = "planner returned empty joint trajectory";
    return result;
  }

  std::string guard_reason;
  if (!joint_position_guard_.check_trajectory(
          planned_trajectory_msg.joint_trajectory, guard_reason,
          request.joint_position_guard_mode)) {
    result.reason = "pre-downsample " + guard_reason;
    return result;
  }

  const std::string post_plan_interrupt = request.interrupt_reason("post_plan");
  if (!post_plan_interrupt.empty()) {
    result.status = PlanningStatus::kCanceled;
    result.reason = post_plan_interrupt;
    return result;
  }

  result = post_process_trajectory(
      planned_trajectory_msg, plan_start_state_msg, has_plan_start_state,
      request.current_robot_state, request.velocity_scale,
      request.acceleration_scale, "failed to convert plan start_state",
      "time-parameterization failed");
  if (result.status != PlanningStatus::kSuccess) {
    return result;
  }

  result.dispatch_primitive = effective_primitive;
  result.planner_id = planner_selection.planner_id;
  result.cartesian_fraction = cartesian_fraction;
  return result;
}

// ── BLENDED_SEQUENCE: MoveGroupSequence action + TrajectoryAssembler ──

moveit_msgs::msg::MotionSequenceItem
PrimitiveRouterDispatch::build_sequence_item(
    const interfaces::msg::SequenceStep &step,
    const std::string &pipeline_id, const std::string &planner_id,
    double velocity_scale, double acceleration_scale) {
  moveit_msgs::msg::MotionSequenceItem item;

  // Request
  item.req.group_name = kPlanningGroup;
  item.req.planner_id = planner_id;
  item.req.pipeline_id = pipeline_id;
  item.req.max_velocity_scaling_factor = velocity_scale;
  item.req.max_acceleration_scaling_factor = acceleration_scale;
  item.req.num_planning_attempts = 1;
  item.req.allowed_planning_time = kPlanningTimeSec;

  // Goal constraints: pose target
  moveit_msgs::msg::PositionConstraint pc;
  pc.link_name = "tool0";
  pc.constraint_region.primitives.emplace_back();
  pc.constraint_region.primitives.back().type =
      shape_msgs::msg::SolidPrimitive::SPHERE;
  pc.constraint_region.primitives.back().dimensions = {0.01};
  pc.constraint_region.primitive_poses.push_back(step.target_pose);

  moveit_msgs::msg::OrientationConstraint oc;
  oc.link_name = "tool0";
  oc.orientation = step.target_pose.orientation;
  oc.absolute_x_axis_tolerance = 0.01;
  oc.absolute_y_axis_tolerance = 0.01;
  oc.absolute_z_axis_tolerance = 0.01;

  moveit_msgs::msg::Constraints goal_constraints;
  goal_constraints.position_constraints.push_back(std::move(pc));
  goal_constraints.orientation_constraints.push_back(std::move(oc));
  item.req.goal_constraints.push_back(std::move(goal_constraints));

  // Blend radius (link_blend for MoveIt sequence)
  item.blend_radius = step.blend_radius_m;

  return item;
}

PrimitiveRouterDispatch::PlanningResult
PrimitiveRouterDispatch::plan_blended_sequence(
    const PlanningRequest &request,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
    const {
  (void)move_group; // node created locally; MoveGroup not needed directly
  PlanningResult result;
  const auto &goal = request.goal;

  if (!goal || goal->sequence_steps.size() < 2U) {
    result.reason = "BLENDED_SEQUENCE requires at least 2 typed sequence_steps";
    return result;
  }
  if (goal->sequence_steps.back().blend_radius_m != 0.0) {
    result.reason =
        "BLENDED_SEQUENCE requires last blend_radius_m to be 0.0";
    return result;
  }

  for (std::size_t i = 0; i < goal->sequence_steps.size(); ++i) {
    const auto &step = goal->sequence_steps[i];
    const std::string step_primitive =
        ExecuteMotionActionSupport::normalize_primitive(step.primitive_type);
    if (step_primitive != "LIN" && step_primitive != "PTP") {
      result.reason = "BLENDED_SEQUENCE step[" + std::to_string(i) +
                      "] unsupported primitive_type '" + step.primitive_type +
                      "'; only LIN/PTP are supported";
      return result;
    }
    if (step.blend_radius_m < 0.0) {
      result.reason = "BLENDED_SEQUENCE step[" + std::to_string(i) +
                      "] blend_radius_m must be >= 0.0";
      return result;
    }
    geometry_msgs::msg::Pose step_pose = step.target_pose;
    std::string orientation_reason;
    if (!orientation_filter_.normalize_and_validate(step_pose,
                                                    orientation_reason)) {
      result.reason = "BLENDED_SEQUENCE step[" + std::to_string(i) +
                      "] orientation rejected: " + orientation_reason;
      return result;
    }
  }

  // Build goal
  using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
  MoveGroupSequence::Goal sequence_goal;
  sequence_goal.request.items.reserve(goal->sequence_steps.size());

  for (std::size_t i = 0; i < goal->sequence_steps.size(); ++i) {
    const auto &step = goal->sequence_steps[i];
    const std::string step_primitive =
        ExecuteMotionActionSupport::normalize_primitive(step.primitive_type);
    std::string step_planner_id = step.planner_id;
    if (step_planner_id.empty()) {
      step_planner_id = planner_router_.route_planner(step_primitive, false);
    }
    const PlannerSelection selection = resolve_planner_selection(step_planner_id);
    if (step_primitive == "LIN" &&
        (selection.pipeline_id != "pilz_industrial_motion_planner" ||
         selection.planner_id != "LIN")) {
      result.reason = "BLENDED_SEQUENCE LIN step[" + std::to_string(i) +
                      "] requires Pilz LIN planner";
      return result;
    }

    const double v_scale =
        step.velocity_scale > 0.0 ? step.velocity_scale : request.velocity_scale;
    const double a_scale = step.acceleration_scale > 0.0
                               ? step.acceleration_scale
                               : request.acceleration_scale;
    sequence_goal.request.items.push_back(
        build_sequence_item(step, selection.pipeline_id, selection.planner_id,
                            v_scale, a_scale));
  }

  // Create action client (getNodeHandle is not exported in some MoveIt
  // builds; create a local node instead).
  auto node = std::make_shared<rclcpp::Node>(
      "primitive_router_blended_sequence",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));
  auto client = rclcpp_action::create_client<MoveGroupSequence>(
      node, "move_group_sequence");

  constexpr double kActionWaitSec = 10.0;
  if (!client->wait_for_action_server(
          std::chrono::duration<double>(kActionWaitSec))) {
    result.reason = "MoveGroupSequence action server unavailable after " +
                    std::to_string(static_cast<int>(kActionWaitSec)) + "s";
    return result;
  }

  // Interrupt check before sending
  const std::string pre_send_interrupt =
      request.interrupt_reason("blended_sequence_plan");
  if (!pre_send_interrupt.empty()) {
    result.status = PlanningStatus::kCanceled;
    result.reason = pre_send_interrupt;
    return result;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto goal_future = client->async_send_goal(sequence_goal);
  if (executor.spin_until_future_complete(
          goal_future,
          std::chrono::duration<double>(kActionWaitSec)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    executor.remove_node(node);
    result.reason = "MoveGroupSequence goal submission timed out";
    return result;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    executor.remove_node(node);
    result.reason = "MoveGroupSequence goal rejected by server";
    return result;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (executor.spin_until_future_complete(
          result_future,
          std::chrono::duration<double>(kActionWaitSec)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    executor.remove_node(node);
    result.reason = "MoveGroupSequence result timed out";
    return result;
  }

  executor.remove_node(node);

  auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped_result.result) {
    result.reason = "MoveGroupSequence action did not finish successfully";
    return result;
  }

  const auto &response = wrapped_result.result->response;
  if (response.error_code.val !=
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    result.reason = "MoveGroupSequence planning failed with MoveIt error code: " +
                    std::to_string(response.error_code.val);
    return result;
  }

  // Extract and merge trajectories
  std::vector<trajectory_msgs::msg::JointTrajectory> segments;
  segments.reserve(response.planned_trajectories.size());
  for (const auto &planned : response.planned_trajectories) {
    segments.push_back(planned.joint_trajectory);
  }

  auto merge_result = TrajectoryAssembler::merge(segments);
  if (!merge_result.success) {
    result.reason =
        "TrajectoryAssembler failed: " + merge_result.error_message;
    return result;
  }

  if (merge_result.trajectory.points.empty()) {
    result.reason = "BLENDED_SEQUENCE produced an empty merged trajectory";
    return result;
  }

  std::string guard_reason;
  if (!joint_position_guard_.check_trajectory(
          merge_result.trajectory, guard_reason,
          request.joint_position_guard_mode)) {
    result.reason = "pre-downsample " + guard_reason;
    return result;
  }

  moveit_msgs::msg::RobotTrajectory merged_robot_trajectory;
  merged_robot_trajectory.joint_trajectory = merge_result.trajectory;

  result = post_process_trajectory(
      merged_robot_trajectory,
      moveit_msgs::msg::RobotState{}, // no explicit start state for merged
      false, request.current_robot_state, request.velocity_scale,
      request.acceleration_scale,
      "BLENDED_SEQUENCE: failed to convert merged start_state",
      "BLENDED_SEQUENCE time parameterization failed; ");
  if (result.status != PlanningStatus::kSuccess) {
    return result;
  }

  result.dispatch_primitive = "BLENDED_SEQUENCE";
  result.planner_id = "PILZ_SEQUENCE";
  result.cartesian_fraction = 1.0;
  result.time_parameterization_note =
      "BLENDED_SEQUENCE planned via MoveGroupSequenceAction";
  return result;
}
} // namespace motion_core
