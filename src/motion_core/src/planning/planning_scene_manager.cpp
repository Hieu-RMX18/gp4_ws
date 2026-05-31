// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include "motion_core/planning_scene_manager.hpp"

#include <fstream>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

namespace motion_core {

const char *scene_load_result_name(SceneLoadResult result) {
  switch (result) {
  case SceneLoadResult::OK:
    return "OK";
  case SceneLoadResult::FILE_NOT_FOUND:
    return "FILE_NOT_FOUND";
  case SceneLoadResult::PARSE_ERROR:
    return "PARSE_ERROR";
  case SceneLoadResult::APPLY_FAILED:
    return "APPLY_FAILED";
  case SceneLoadResult::SCENE_NOT_READY:
    return "SCENE_NOT_READY";
  }
  return "UNKNOWN";
}

PlanningSceneManager::PlanningSceneManager(rclcpp::Logger logger)
    : logger_(logger) {}

/// Parse one collision object from a YAML node.
static bool parse_one_object(const std::string &name, const YAML::Node &node,
                             moveit_msgs::msg::CollisionObject &object,
                             std::string &parsed_type, std::string &reason,
                             const rclcpp::Logger &logger) {
  (void)logger;
  reason.clear();

  if (!node.IsMap()) {
    reason = "expected a map";
    return false;
  }

  const std::string type = node["type"].as<std::string>("");
  parsed_type = type;
  const std::string frame_id = node["frame_id"].as<std::string>("base_link");

  object.id = name;
  object.header.frame_id = frame_id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Parse pose
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;

  if (node["pose"]) {
    const auto &p = node["pose"];
    if (p["position"] && p["position"].IsSequence() &&
        p["position"].size() == 3) {
      pose.position.x = p["position"][0].as<double>();
      pose.position.y = p["position"][1].as<double>();
      pose.position.z = p["position"][2].as<double>();
    }
    if (p["orientation"] && p["orientation"].IsSequence() &&
        p["orientation"].size() == 4) {
      pose.orientation.x = p["orientation"][0].as<double>();
      pose.orientation.y = p["orientation"][1].as<double>();
      pose.orientation.z = p["orientation"][2].as<double>();
      pose.orientation.w = p["orientation"][3].as<double>();
    }
  }

  if (type == "mesh") {
    std::string mesh_resource;
    if (node["resource"]) {
      mesh_resource = node["resource"].as<std::string>("");
    }
    if (mesh_resource.empty() && node["mesh_resource"]) {
      mesh_resource = node["mesh_resource"].as<std::string>("");
    }
    if (mesh_resource.empty()) {
      reason = "mesh requires resource (or mesh_resource)";
      return false;
    }

    Eigen::Vector3d scale(1.0, 1.0, 1.0);
    if (node["scale"]) {
      if (!node["scale"].IsSequence() || node["scale"].size() != 3) {
        reason = "mesh scale must be [sx, sy, sz]";
        return false;
      }
      scale.x() = node["scale"][0].as<double>();
      scale.y() = node["scale"][1].as<double>();
      scale.z() = node["scale"][2].as<double>();
    }

    std::unique_ptr<shapes::Mesh> mesh(
        shapes::createMeshFromResource(mesh_resource, scale));
    if (!mesh) {
      reason = "failed to load mesh resource '" + mesh_resource + "'";
      return false;
    }

    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(mesh.get(), shape_msg)) {
      reason =
          "failed to construct shape message from mesh '" + mesh_resource + "'";
      return false;
    }

    const auto *mesh_msg = boost::get<shape_msgs::msg::Mesh>(&shape_msg);
    if (mesh_msg == nullptr) {
      reason = "mesh resource did not convert to shape_msgs/Mesh";
      return false;
    }

    object.meshes.push_back(*mesh_msg);
    object.mesh_poses.push_back(pose);
    return true;
  }

  // Parse primitive shape
  shape_msgs::msg::SolidPrimitive primitive;
  if (type == "box") {
    if (!node["dimensions"] || !node["dimensions"].IsSequence() ||
        node["dimensions"].size() != 3) {
      reason = "box requires dimensions: [x, y, z]";
      return false;
    }
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] =
        node["dimensions"][0].as<double>();
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] =
        node["dimensions"][1].as<double>();
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] =
        node["dimensions"][2].as<double>();
  } else if (type == "cylinder") {
    if (!node["dimensions"] || !node["dimensions"].IsSequence() ||
        node["dimensions"].size() != 2) {
      reason = "cylinder requires dimensions: [radius, height]";
      return false;
    }
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
        node["dimensions"][0].as<double>();
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
        node["dimensions"][1].as<double>();
  } else {
    reason = "unsupported type '" + type + "' (supported: box, cylinder, mesh)";
    return false;
  }

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  return true;
}

SceneLoadResult
PlanningSceneManager::load_and_apply(const std::string &yaml_path) {
  scene_loaded_.store(false);
  applied_object_count_ = 0;

  {
    std::ifstream test_file(yaml_path);
    if (!test_file.good()) {
      RCLCPP_ERROR(logger_, "PlanningSceneManager: file not found: %s",
                   yaml_path.c_str());
      return SceneLoadResult::FILE_NOT_FOUND;
    }
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception &ex) {
    RCLCPP_ERROR(logger_, "PlanningSceneManager: YAML parse error: %s",
                 ex.what());
    return SceneLoadResult::PARSE_ERROR;
  }

  const auto objects_node = root["collision_objects"];
  if (!objects_node || !objects_node.IsMap()) {
    RCLCPP_ERROR(
        logger_,
        "PlanningSceneManager: 'collision_objects' missing or not a map");
    return SceneLoadResult::PARSE_ERROR;
  }

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  for (const auto &entry : objects_node) {
    const std::string name = entry.first.as<std::string>();
    moveit_msgs::msg::CollisionObject obj;
    std::string parsed_type;
    std::string reason;

    if (!parse_one_object(name, entry.second, obj, parsed_type, reason,
                          logger_)) {
      RCLCPP_WARN(logger_, "PlanningSceneManager: skipping '%s': %s",
                  name.c_str(), reason.c_str());
      continue;
    }

    RCLCPP_INFO(logger_,
                "PlanningSceneManager: parsed '%s' (type=%s, frame=%s)",
                name.c_str(), parsed_type.c_str(), obj.header.frame_id.c_str());
    collision_objects.push_back(std::move(obj));
  }

  if (collision_objects.empty()) {
    RCLCPP_WARN(logger_, "PlanningSceneManager: no valid objects parsed");
    return SceneLoadResult::PARSE_ERROR;
  }

  try {
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObjects(collision_objects);
    applied_object_count_ = collision_objects.size();
    scene_loaded_.store(true);

    RCLCPP_INFO(logger_, "PlanningSceneManager: applied %zu collision objects",
                applied_object_count_);
    return SceneLoadResult::OK;
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(logger_, "PlanningSceneManager: apply failed: %s", ex.what());
    return SceneLoadResult::APPLY_FAILED;
  }
}

} // namespace motion_core
