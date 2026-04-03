#pragma once

#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace motion_core
{
class SeedManager
{
public:
  explicit SeedManager(rclcpp::Node & node);

  bool get_seed_state(std::vector<double> & seed) const;

private:
  static std::vector<std::string> default_joint_names();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool fallback_named_target_seed(std::vector<double> & seed) const;

  rclcpp::Logger logger_;
  std::vector<std::string> ordered_joint_names_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool has_joint_state_{ false };
};
}  // namespace motion_core
