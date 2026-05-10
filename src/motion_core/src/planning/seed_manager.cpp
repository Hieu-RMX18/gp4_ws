#include "motion_core/seed_manager.hpp"

#include <algorithm>
#include <cstdint>
#include <sstream>

namespace motion_core {
SeedManager::SeedManager(rclcpp::Node &node)
    : logger_(node.get_logger()), clock_(node.get_clock()),
      ordered_joint_names_(default_joint_names()) {
  const auto joint_state_max_age_ms =
      node.declare_parameter<int64_t>("joint_state_max_age_ms", 200);
  joint_state_max_age_ = std::chrono::milliseconds(
      joint_state_max_age_ms > 0 ? joint_state_max_age_ms : 200);
  bool use_sim_time = false;
  (void)node.get_parameter("use_sim_time", use_sim_time);
  allow_fallback_seed_ =
      node.declare_parameter<bool>("allow_fallback_seed", use_sim_time);

  joint_state_sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
      "/yaskawa/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&SeedManager::joint_state_callback, this,
                std::placeholders::_1));
}

bool SeedManager::get_seed_state(std::vector<double> &seed) const {
  return get_seed_state("", seed);
}

bool SeedManager::get_seed_state(const std::string &primitive_family,
                                 std::vector<double> &seed) const {
  std::string current_state_reason = "latest /yaskawa/joint_states unavailable";

  // V4 D2 Priority 1: current joint state
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    std::chrono::milliseconds age{0};
    if (is_joint_state_fresh_locked(age, current_state_reason) &&
        extract_ordered_positions(seed)) {
      return true;
    }
  }

  if (!allow_fallback_seed_) {
    RCLCPP_WARN(
        logger_,
        "Seed unavailable: %s; allow_fallback_seed=false (fail closed).",
        current_state_reason.c_str());
    return false;
  }

  // V4 D2 Priority 2: last successful seed by primitive family
  if (!primitive_family.empty() &&
      fallback_cached_seed(primitive_family, seed)) {
    RCLCPP_WARN(logger_,
                "Using cached seed from last successful %s IK solution.",
                primitive_family.c_str());
    return true;
  }

  // V4 D2 Priority 3: commissioning named-target fallback (ready)
  if (fallback_named_target_seed(seed)) {
    RCLCPP_WARN(logger_,
                "Using named-target fallback seed (joint state unavailable).");
    return true;
  }

  // V4 D2 Priority 4: refreshed current state (retry) — already attempted in
  // priority 1
  RCLCPP_WARN(
      logger_,
      "Seed unavailable: %s; no cached seed and no named-target fallback.",
      current_state_reason.c_str());
  return false;
}

bool SeedManager::get_current_joint_positions(
    std::vector<double> &positions) const {
  builtin_interfaces::msg::Time stamp;
  return get_current_joint_positions(positions, stamp);
}

bool SeedManager::get_current_joint_positions(
    std::vector<double> &positions,
    builtin_interfaces::msg::Time &stamp) const {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  std::chrono::milliseconds age{0};
  std::string reason;
  if (!is_joint_state_fresh_locked(age, reason)) {
    positions.clear();
    stamp = builtin_interfaces::msg::Time();
    RCLCPP_WARN(logger_, "Current joint position request rejected: %s",
                reason.c_str());
    return false;
  }

  if (!extract_ordered_positions(positions)) {
    stamp = builtin_interfaces::msg::Time();
    return false;
  }
  stamp = latest_joint_state_.header.stamp;
  return true;
}

void SeedManager::cache_successful_seed(const std::string &primitive_family,
                                        const std::vector<double> &seed) {
  if (primitive_family.empty() || seed.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(seed_cache_mutex_);
  last_successful_seeds_[primitive_family] = seed;
}

std::vector<std::string> SeedManager::default_joint_names() {
  return {"joint_1_s", "joint_2_l", "joint_3_u",
          "joint_4_r", "joint_5_b", "joint_6_t"};
}

void SeedManager::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = *msg;
  receive_time_ = clock_->now();
  has_joint_state_ = true;
}

bool SeedManager::is_joint_state_fresh_locked(std::chrono::milliseconds &age,
                                              std::string &reason) const {
  // NOTE: must be called with joint_state_mutex_ held.
  if (!has_joint_state_) {
    age = std::chrono::milliseconds::max();
    reason = "no /yaskawa/joint_states sample received";
    return false;
  }

  const auto age_ns = (clock_->now() - receive_time_).nanoseconds();
  const auto non_negative_age_ns = age_ns < 0 ? 0 : age_ns;
  age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::nanoseconds(non_negative_age_ns));
  if (age > joint_state_max_age_) {
    std::ostringstream stream;
    stream << "/yaskawa/joint_states is stale: age=" << age.count()
           << " ms exceeds max_age=" << joint_state_max_age_.count() << " ms";
    reason = stream.str();
    return false;
  }

  reason.clear();
  return true;
}

bool SeedManager::extract_ordered_positions(std::vector<double> &seed) const {
  // NOTE: must be called with joint_state_mutex_ held
  seed.clear();
  seed.reserve(ordered_joint_names_.size());

  for (const auto &joint_name : ordered_joint_names_) {
    const auto it = std::find(latest_joint_state_.name.begin(),
                              latest_joint_state_.name.end(), joint_name);
    if (it == latest_joint_state_.name.end()) {
      seed.clear();
      RCLCPP_WARN(logger_,
                  "Seed unavailable: joint '%s' missing in latest "
                  "/yaskawa/joint_states.",
                  joint_name.c_str());
      return false;
    }

    const std::size_t index = static_cast<std::size_t>(
        std::distance(latest_joint_state_.name.begin(), it));
    if (index >= latest_joint_state_.position.size()) {
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

bool SeedManager::fallback_named_target_seed(std::vector<double> &seed) const {
  seed.clear();

  // V4 D2 Priority 3: deterministic commissioning seed used when no current
  // joint state or cached seed is available. Aligned with SRDF group_state
  // "ready" (operator-defined commissioning pose, all joints within
  // operational_joint_limits).
  static const std::vector<double> kHomeSeed = {
      1.938101818035138,    // joint_1_s
      0.0903533622099061,   // joint_2_l
      -0.15852595742235326, // joint_3_u
      0.0,                  // joint_4_r
      -1.1752774274713826,  // joint_5_b
      0.05333592949720888   // joint_6_t
  };

  if (kHomeSeed.size() == ordered_joint_names_.size()) {
    seed = kHomeSeed;
    return true;
  }

  return false;
}

bool SeedManager::fallback_cached_seed(const std::string &primitive_family,
                                       std::vector<double> &seed) const {
  std::lock_guard<std::mutex> lock(seed_cache_mutex_);
  const auto it = last_successful_seeds_.find(primitive_family);
  if (it != last_successful_seeds_.end() && !it->second.empty()) {
    seed = it->second;
    return true;
  }
  return false;
}

} // namespace motion_core
