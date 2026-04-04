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
  return get_seed_state("", seed);
}

bool SeedManager::get_seed_state(const std::string & primitive_family, std::vector<double> & seed) const
{
  // V4 D2 Priority 1: current joint state
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (has_joint_state_ && extract_ordered_positions(seed))
    {
      return true;
    }
  }

  // V4 D2 Priority 2: last successful seed by primitive family
  if (!primitive_family.empty() && fallback_cached_seed(primitive_family, seed))
  {
    RCLCPP_WARN(
      logger_,
      "Using cached seed from last successful %s IK solution.",
      primitive_family.c_str());
    return true;
  }

  // V4 D2 Priority 3: named-target fallback (home, ready_down)
  if (fallback_named_target_seed(seed))
  {
    RCLCPP_WARN(
      logger_,
      "Using named-target fallback seed (joint state unavailable).");
    return true;
  }

  // V4 D2 Priority 4: refreshed current state (retry) — already attempted in priority 1
  RCLCPP_WARN(
    logger_,
    "Seed unavailable: no joint state, no cached seed, no named-target fallback.");
  return false;
}

void SeedManager::cache_successful_seed(
  const std::string & primitive_family,
  const std::vector<double> & seed)
{
  if (primitive_family.empty() || seed.empty())
  {
    return;
  }

  std::lock_guard<std::mutex> lock(seed_cache_mutex_);
  last_successful_seeds_[primitive_family] = seed;
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

bool SeedManager::extract_ordered_positions(std::vector<double> & seed) const
{
  // NOTE: must be called with joint_state_mutex_ held
  seed.clear();
  seed.reserve(ordered_joint_names_.size());

  for (const auto & joint_name : ordered_joint_names_)
  {
    const auto it = std::find(
      latest_joint_state_.name.begin(),
      latest_joint_state_.name.end(),
      joint_name);
    if (it == latest_joint_state_.name.end())
    {
      seed.clear();
      RCLCPP_WARN(
        logger_,
        "Seed unavailable: joint '%s' missing in latest /yaskawa/joint_states.",
        joint_name.c_str());
      return false;
    }

    const std::size_t index = static_cast<std::size_t>(
      std::distance(latest_joint_state_.name.begin(), it));
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

bool SeedManager::fallback_named_target_seed(std::vector<double> & seed) const
{
  seed.clear();

  // V4 D2 Priority 3: Named-target fallback using known SRDF group states.
  // 'home' target from the real GP4 robot (SRDF group_state "home").
  // These values must match the SRDF exactly.
  static const std::vector<double> kHomeSeed = {
    1.5477395698141883,   // joint_1_s
    -0.15883329466662804, // joint_2_l
    -0.15854787143360877, // joint_3_u
    0.0,                  // joint_4_r
    -1.6017466450445892,  // joint_5_b
    0.05361262853660316   // joint_6_t
  };

  if (kHomeSeed.size() == ordered_joint_names_.size())
  {
    seed = kHomeSeed;
    return true;
  }

  return false;
}

bool SeedManager::fallback_cached_seed(
  const std::string & primitive_family,
  std::vector<double> & seed) const
{
  std::lock_guard<std::mutex> lock(seed_cache_mutex_);
  const auto it = last_successful_seeds_.find(primitive_family);
  if (it != last_successful_seeds_.end() && !it->second.empty())
  {
    seed = it->second;
    return true;
  }
  return false;
}

}  // namespace motion_core
