#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "interfaces/action/execute_motion.hpp"
#include "motion_core/angle_branch_utils.hpp"
#include "motion_core/ik_selector.hpp"
#include "motion_core/joint_position_guard.hpp"
#include "motion_core/orientation_filter.hpp"
#include "motion_core/planner_router.hpp"
#include "motion_core/quality_gate.hpp"
#include "motion_core/seed_manager.hpp"
#include "motion_core/trajectory_post_processor.hpp"

namespace motion_core {
class PrimitiveRouterDispatch {
public:
  enum class PlanningStatus {
    kSuccess,
    kFailure,
    kCanceled,
  };

  struct PlanningStageResult {
    PlanningStatus status = PlanningStatus::kFailure;
    std::string reason;
  };

  using PlanWithInterruptionFn = std::function<PlanningStageResult(
      moveit::planning_interface::MoveGroupInterface::Plan &,
      const std::string &)>;
  using ReadCurrentTcpPoseFn = std::function<bool(
      geometry_msgs::msg::PoseStamped &, std::string &, double)>;
  using InterruptReasonFn = std::function<std::string(const std::string &)>;

  struct PlanningRequest {
    explicit PlanningRequest(const moveit::core::RobotState &state)
        : current_robot_state(state) {}

    std::shared_ptr<const interfaces::action::ExecuteMotion::Goal> goal;
    std::string primitive;
    std::string effective_primitive;
    std::uint64_t goal_sequence = 0U;
    double velocity_scale = TrajectoryPostProcessor::kDefaultVelocityScaling;
    double acceleration_scale =
        TrajectoryPostProcessor::kDefaultAccelerationScaling;
    moveit::core::RobotState current_robot_state;
    std::vector<double> current_joint_positions;
    std::vector<const moveit::core::JointModel *> active_joint_models;
    std::vector<std::string> active_joint_names;
    PlanWithInterruptionFn plan_with_interruption;
    InterruptReasonFn interrupt_reason;
    JointPositionGuard::Mode joint_position_guard_mode =
        JointPositionGuard::Mode::Default;
  };

  struct PlanningResult {
    PlanningStatus status = PlanningStatus::kFailure;
    std::string reason;
    std::string dispatch_primitive;
    std::string planner_id;
    trajectory_msgs::msg::JointTrajectory trajectory;
    double cartesian_fraction = QualityGate::kFractionNotApplicable;
    std::string time_parameterization_note;
    std::string ruckig_reason;
    bool is_move_joint = false;
    int move_joint_index = -1;
    double move_joint_target_angle = 0.0;
  };

  PrimitiveRouterDispatch(
      rclcpp::Logger logger,
      std::function<
          std::shared_ptr<moveit::planning_interface::MoveGroupInterface>()>
          move_group_provider,
      const PlannerRouter &planner_router,
      const OrientationFilter &orientation_filter, SeedManager &seed_manager,
      IkSelector &ik_selector,
      TrajectoryPostProcessor &trajectory_post_processor,
      ReadCurrentTcpPoseFn read_current_tcp_pose,
      JointPositionGuard joint_position_guard = JointPositionGuard{});

  PlanningResult plan_for_primitive(const PlanningRequest &request);

private:
  struct PlannerSelection {
    std::string pipeline_id;
    std::string planner_id;
  };

  static constexpr double kPlanningTimeSec = 5.0;
  static constexpr double kCartesianEefStep = 0.005;
  static constexpr double kCartesianEefStepRelaxed = 0.010;
  static constexpr double kCartesianJumpThreshold = 1.5;
  static constexpr const char *kPlanningGroup = "gp4_arm";

  static PlannerSelection
  resolve_planner_selection(const std::string &planner_id);
  static bool is_pose_goal_required(const std::string &primitive,
                                    bool has_joint_target);
  static double quaternion_norm_sq(const geometry_msgs::msg::Quaternion &q);
  static double max_abs_value(const std::vector<double> &values);
  static std::string format_joint_vector(const std::vector<double> &joints);

  void log_joint_branch_selection(
      const std::string &primitive, std::uint64_t sequence,
      const std::vector<std::string> &joint_names,
      const std::vector<double> &current, const std::vector<double> &requested,
      const BranchPreservedJointVectorResult &branch_result) const;

  PlanningResult post_process_trajectory(
      const moveit_msgs::msg::RobotTrajectory &planned_trajectory_msg,
      const moveit_msgs::msg::RobotState &plan_start_state_msg,
      bool has_plan_start_state,
      const moveit::core::RobotState &current_robot_state,
      double velocity_scale, double acceleration_scale,
      const std::string &start_state_failure_message,
      const std::string &time_parameterization_failure_prefix) const;

  rclcpp::Logger logger_;
  std::function<
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface>()>
      move_group_provider_;
  const PlannerRouter &planner_router_;
  const OrientationFilter &orientation_filter_;
  SeedManager &seed_manager_;
  IkSelector &ik_selector_;
  TrajectoryPostProcessor &trajectory_post_processor_;
  ReadCurrentTcpPoseFn read_current_tcp_pose_;
  JointPositionGuard joint_position_guard_;
};
} // namespace motion_core
