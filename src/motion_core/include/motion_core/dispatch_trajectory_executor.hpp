#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "interfaces/action/dispatch_trajectory.hpp"
#include "motion_core/execution_orchestrator.hpp"
#include "motion_core/quality_gate.hpp"
#include "motion_core/trajectory_post_processor.hpp"

namespace motion_core {
class DispatchTrajectoryExecutor {
public:
  using DispatchTrajectory = interfaces::action::DispatchTrajectory;

  enum class Status {
    kSuccess,
    kFailure,
    kCanceled,
  };

  struct Result {
    Status status = Status::kFailure;
    std::string reason;
    std::string note;
  };

  struct DispatchMetadata {
    std::string command_id;
    std::string primitive;
    std::string planner_id;
    builtin_interfaces::msg::Time source_joint_state_stamp;
    bool enforce_start_state_match = true;
    bool extended_mode = false;
  };

  using InterruptReasonFn = std::function<std::string(const std::string &)>;
  using PublishFeedbackFn = std::function<void(double, const std::string &)>;
  using UpdatePhaseFn =
      std::function<void(ExecutionPhase, const std::string &)>;

  DispatchTrajectoryExecutor(
      rclcpp::Logger logger,
      rclcpp_action::Client<DispatchTrajectory>::SharedPtr dispatch_client,
      std::string dispatch_action_name, double dispatch_timeout_sec,
      std::size_t safe_budget_points, QualityGate &quality_gate,
      TrajectoryPostProcessor &trajectory_post_processor);

  Result apply_budget_quality_and_dispatch(
      trajectory_msgs::msg::JointTrajectory trajectory,
      const std::string &primitive, const DispatchMetadata &dispatch_metadata,
      double cartesian_fraction, const InterruptReasonFn &interrupt_reason,
      const PublishFeedbackFn &publish_feedback,
      const UpdatePhaseFn &update_phase, std::size_t &reported_point_count,
      std::size_t &reported_segment_count,
      JointPositionGuard::Mode mode = JointPositionGuard::Mode::Default);

private:
  static bool is_geometry_sensitive_primitive(const std::string &primitive);
  static bool is_finite_trajectory_vector(const std::vector<double> &values);

  bool validate_dispatch_segment_contract(
      const trajectory_msgs::msg::JointTrajectory &segment,
      std::string &reason) const;

  bool validate_mitigated_segments_contract(
      const trajectory_msgs::msg::JointTrajectory &original,
      const std::vector<trajectory_msgs::msg::JointTrajectory> &segments,
      std::string &reason) const;

  bool split_trajectory_for_dispatch(
      const trajectory_msgs::msg::JointTrajectory &input_trajectory,
      std::size_t max_points_per_segment,
      std::vector<trajectory_msgs::msg::JointTrajectory> &output_segments,
      std::string &reason) const;

  Result dispatch_to_hw_adapter(
      const trajectory_msgs::msg::JointTrajectory &trajectory,
      const DispatchMetadata &metadata,
      const InterruptReasonFn &interrupt_reason,
      const UpdatePhaseFn &update_phase) const;

  rclcpp::Logger logger_;
  rclcpp_action::Client<DispatchTrajectory>::SharedPtr dispatch_client_;
  std::string dispatch_action_name_;
  double dispatch_timeout_sec_;
  std::size_t safe_budget_points_;
  QualityGate &quality_gate_;
  TrajectoryPostProcessor &trajectory_post_processor_;
};
} // namespace motion_core
