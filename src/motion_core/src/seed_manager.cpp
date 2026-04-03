#include "motion_core/seed_manager.hpp"

#include <algorithm>

namespace motion_core
{
SeedManager::SeedManager(rclcpp::Node & node)
: logger_(node.get_logger()),
  ordered_joint_names_(default_joint_names())
{
  joint_state_sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
    "/yaskawa/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&SeedManager::joint_state_callback, this, std::placeholders::_1));
}

bool SeedManager::get_seed_state(std::vector<double> & seed) const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);

  if (!has_joint_state_)
  {
    return fallback_named_target_seed(seed);
  }

  seed.clear();
  seed.reserve(ordered_joint_names_.size());

  for (const auto & joint_name : ordered_joint_names_)
  {
    const auto it = std::find(latest_joint_state_.name.begin(), latest_joint_state_.name.end(), joint_name);
    if (it == latest_joint_state_.name.end())
    {
      seed.clear();
      RCLCPP_WARN(
        logger_,
        "Seed unavailable: joint '%s' missing in latest /yaskawa/joint_states.",
        joint_name.c_str());
      return false;
    }

    const std::size_t index = static_cast<std::size_t>(std::distance(latest_joint_state_.name.begin(), it));
    if (index >= latest_joint_state_.position.size())
    {
      seed.clear();
      RCLCPP_WARN(
        logger_,
        "Seed unavailable: position index for joint '%s' is out of range.",
        joint_name.c_str());
      return false;
    }

    seed.push_back(latest_joint_state_.position[index]);
  }

  return true;
}

std::vector<std::string> SeedManager::default_joint_names()
{
  return {
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t"
  };
}

void SeedManager::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = *msg;
  has_joint_state_ = true;
}

bool SeedManager::fallback_named_target_seed(std::vector<double> & seed) const
{
  seed.clear();

  RCLCPP_WARN(
    logger_,
    "Seed unavailable: no /yaskawa/joint_states yet; named-target fallback hook is not integrated in SeedManager.");

  return false;
}
}  // namespace motion_core
