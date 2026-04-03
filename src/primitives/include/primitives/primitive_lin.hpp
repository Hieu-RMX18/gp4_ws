// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "primitives/primitive_base.hpp"
#include "primitives/primitive_types.hpp"

namespace primitives
{
struct LINGoal
{
  geometry_msgs::msg::Pose target_pose;
  double velocity_scale = 0.0;
  double acceleration_scale = 0.0;
};

struct LinearScalingConfig
{
  double velocity_cap = 0.3;
  double acceleration_cap = 0.2;
  double default_velocity = 0.3;
  double default_acceleration = 0.2;
};

class LinearExecutionBackend
{
public:
  virtual ~LinearExecutionBackend() = default;

  virtual bool wait_for_servers(std::string & reason) = 0;
  virtual bool normalize_pose(geometry_msgs::msg::Pose & pose, std::string & reason) = 0;
  virtual bool get_current_pose_world(geometry_msgs::msg::Pose & pose, std::string & reason) = 0;
  virtual bool compute_cartesian_path(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double & fraction,
    std::string & reason) = 0;
  virtual LinearScalingConfig scaling_config() const = 0;
  virtual PrimitiveResult postprocess_and_validate(
    double velocity_scale,
    double acceleration_scale,
    double cartesian_fraction) = 0;
};

std::unique_ptr<LinearExecutionBackend> make_move_group_linear_backend(MoveGroupInterface & mgi);

class PrimitiveLin final : public PrimitiveBase
{
public:
  PrimitiveType type() const override {return PrimitiveType::LIN;}

  PrimitiveResult execute(const LINGoal & goal, MoveGroupInterface & mgi);
  PrimitiveResult execute(const LINGoal & goal, LinearExecutionBackend & backend);
  PrimitiveResult execute_waypoints(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double velocity_scale,
    double acceleration_scale,
    LinearExecutionBackend & backend);

  PrimitiveResult execute(const ExecuteMotionGoal & goal, MoveGroupInterface & mgi) override;
};
}  // namespace primitives
