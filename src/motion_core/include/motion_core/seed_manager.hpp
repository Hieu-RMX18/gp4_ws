#pragma once

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace motion_core
{
class SeedManager
{
public:
  explicit SeedManager(rclcpp::Node & node);

  /// V4 D2: Get seed state using priority:
  /// 1. current joint state
  /// 2. last successful seed by primitive family
  /// 3. commissioning named-target fallback (park_safe/ready-style seed)
  /// 4. refreshed current state (retry)
  bool get_seed_state(std::vector<double> & seed) const;

  /// V4 D2: Get seed with context of which primitive family is requesting it.
  bool get_seed_state(const std::string & primitive_family, std::vector<double> & seed) const;

  /// Return only the latest ordered joint positions from /yaskawa/joint_states.
  /// No cache or named-target fallback is applied.
  bool get_current_joint_positions(std::vector<double> & positions) const;

  /// V4 D2: Cache a successful IK solution for a primitive family.
  void cache_successful_seed(const std::string & primitive_family, const std::vector<double> & seed);

private:
  static std::vector<std::string> default_joint_names();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool extract_ordered_positions(std::vector<double> & seed) const;
  bool fallback_named_target_seed(std::vector<double> & seed) const;
  bool fallback_cached_seed(const std::string & primitive_family, std::vector<double> & seed) const;

  rclcpp::Logger logger_;
  std::vector<std::string> ordered_joint_names_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool has_joint_state_{ false };

  // V4 D2: Per-primitive-family seed cache
  mutable std::mutex seed_cache_mutex_;
  std::map<std::string, std::vector<double>> last_successful_seeds_;
};
}  // namespace motion_core
