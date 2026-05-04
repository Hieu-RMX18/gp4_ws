// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include <geometry_msgs/msg/pose.hpp>

#include "primitives/primitive_base.hpp"
#include "primitives/primitive_types.hpp"

namespace primitives {
struct CIRCGoal {
  geometry_msgs::msg::Pose auxiliary_point; // on-arc waypoint (interim)
  geometry_msgs::msg::Pose goal_pose;
  double velocity_scale = 0.0;
  double acceleration_scale = 0.0;
};

struct CircScalingConfig {
  double velocity_cap = 0.3;
  double acceleration_cap = 0.2;
  double default_velocity = 0.3;
  double default_acceleration = 0.2;
};

class CircExecutionBackend {
public:
  virtual ~CircExecutionBackend() = default;

  virtual bool wait_for_servers(std::string &reason) = 0;
  virtual bool normalize_pose(geometry_msgs::msg::Pose &pose,
                              std::string &reason) = 0;
  virtual bool get_current_pose_world(geometry_msgs::msg::Pose &pose,
                                      std::string &reason) = 0;
  virtual bool configure_circ_planner(std::string &reason) = 0;
  virtual bool
  set_interim_path_constraint(const geometry_msgs::msg::Pose &auxiliary_point,
                              std::string &reason) = 0;
  virtual bool set_goal_pose(const geometry_msgs::msg::Pose &goal_pose,
                             std::string &reason) = 0;
  virtual void clear_path_constraints() = 0;
  virtual CircScalingConfig scaling_config() const = 0;
  virtual PrimitiveResult plan_with_pipeline(double velocity_scale,
                                             double acceleration_scale) = 0;
};

class PrimitiveCirc final : public PrimitiveBase {
public:
  PrimitiveType type() const override { return PrimitiveType::CIRC; }

  PrimitiveResult execute(const CIRCGoal &goal, MoveGroupInterface &mgi);
  PrimitiveResult execute(const CIRCGoal &goal, CircExecutionBackend &backend);

  PrimitiveResult execute(const ExecuteMotionGoal &goal,
                          MoveGroupInterface &mgi) override;
};
} // namespace primitives
