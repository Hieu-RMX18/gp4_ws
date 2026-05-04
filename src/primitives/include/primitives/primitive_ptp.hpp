// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "primitives/primitive_base.hpp"
#include "primitives/primitive_types.hpp"

namespace primitives {
struct PTPGoal {
  std::vector<double> joint_target;     // optional
  geometry_msgs::msg::Pose pose_target; // optional
  double velocity_scale = 0.0;
  double acceleration_scale = 0.0;
};

struct PtpScalingConfig {
  double velocity_cap = 0.3;
  double acceleration_cap = 0.2;
  double default_velocity = 0.3;
  double default_acceleration = 0.2;
};

class PtpExecutionBackend {
public:
  virtual ~PtpExecutionBackend() = default;

  virtual bool wait_for_servers(std::string &reason) = 0;
  virtual bool configure_ptp_planner(std::string &reason) = 0;
  virtual bool set_joint_target(const std::vector<double> &target,
                                std::string &reason) = 0;
  virtual bool normalize_pose(geometry_msgs::msg::Pose &pose,
                              std::string &reason) = 0;
  virtual bool solve_pose_to_joints(const geometry_msgs::msg::Pose &pose,
                                    std::vector<double> &joint_solution,
                                    std::string &reason) = 0;
  virtual PtpScalingConfig scaling_config() const = 0;
  virtual PrimitiveResult plan_with_pipeline(double velocity_scale,
                                             double acceleration_scale) = 0;
};

class PrimitivePtp final : public PrimitiveBase {
public:
  PrimitiveType type() const override { return PrimitiveType::PTP; }

  PrimitiveResult execute(const PTPGoal &goal, MoveGroupInterface &mgi);
  PrimitiveResult execute(const ExecuteMotionGoal &goal,
                          MoveGroupInterface &mgi) override;

  PrimitiveResult execute(const PTPGoal &goal, PtpExecutionBackend &backend);
};
} // namespace primitives
