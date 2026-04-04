// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
//
// V4 J9: YAML-driven planning scene manager.
// Loads collision objects from scene_objects.yaml on startup.
// Fail-closed: if scene is not loaded, planning is blocked.
#pragma once

#include <atomic>
#include <string>

#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

namespace motion_core
{
enum class SceneLoadResult : uint8_t
{
  OK,
  FILE_NOT_FOUND,
  PARSE_ERROR,
  APPLY_FAILED,
  SCENE_NOT_READY
};

const char * scene_load_result_name(SceneLoadResult result);

class PlanningSceneManager
{
public:
  explicit PlanningSceneManager(rclcpp::Logger logger);

  /// Load collision objects from YAML file and apply to planning scene.
  SceneLoadResult load_and_apply(const std::string & yaml_path);

  /// V4 fail-closed: planning blocked if this returns false.
  bool is_scene_loaded() const { return scene_loaded_.load(); }

  std::size_t object_count() const { return applied_object_count_; }

private:
  rclcpp::Logger logger_;
  std::atomic<bool> scene_loaded_{false};
  std::size_t applied_object_count_{0};
};
}  // namespace motion_core
