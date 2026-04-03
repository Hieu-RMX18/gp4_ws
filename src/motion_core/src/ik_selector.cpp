#include "motion_core/ik_selector.hpp"

#include <algorithm>
#include <array>
#include <sstream>
#include <utility>

#include <moveit/robot_state/robot_state.h>

namespace motion_core
{
namespace
{
constexpr std::size_t kMaxIkAttempts = 3;
constexpr double kIkTimeoutSec = 0.05;

std::string join_messages(const std::vector<std::string> & messages)
{
  if (messages.empty())
  {
    return "";
  }

  std::ostringstream stream;
  for (std::size_t i = 0; i < messages.size(); ++i)
  {
    if (i > 0)
    {
      stream << "; ";
    }
    stream << messages[i];
  }

  return stream.str();
}
}  // namespace

IkSelector::IkSelector(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
  std::string planning_group)
: move_group_(std::move(move_group)),
  planning_group_(std::move(planning_group))
{
}

void IkSelector::set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
{
  move_group_ = std::move(move_group);
}

void IkSelector::set_planning_group(const std::string & planning_group)
{
  planning_group_ = planning_group;
}

bool IkSelector::solve_ik(
  const geometry_msgs::msg::Pose & target_pose,
  const std::vector<double> & seed_state,
  std::vector<double> & joint_solution,
  std::string & reason)
{
  joint_solution.clear();
  reason.clear();

  if (!move_group_)
  {
    reason = "IK selector is not configured with MoveGroupInterface";
    return false;
  }

  if (planning_group_.empty())
  {
    reason = "planning group is empty";
    return false;
  }

  moveit::core::RobotStatePtr initial_state = move_group_->getCurrentState(5.0);
  if (!initial_state)
  {
    reason = "failed to read current robot state from MoveIt";
    return false;
  }

  const moveit::core::JointModelGroup * joint_model_group = initial_state->getJointModelGroup(planning_group_);
  if (!joint_model_group)
  {
    reason = "joint model group not found: " + planning_group_;
    return false;
  }

  const std::size_t expected_dof = joint_model_group->getVariableCount();
  if (seed_state.size() != expected_dof)
  {
    std::ostringstream stream;
    stream << "seed size mismatch for group '" << planning_group_ << "': expected "
           << expected_dof << ", got " << seed_state.size();
    reason = stream.str();
    return false;
  }

  std::vector<std::string> attempt_logs;
  std::size_t attempt_count = 0;

  auto run_attempt = [&](const moveit::core::RobotState & reference_state,
                         const std::vector<double> & attempt_seed,
                         const std::string & label) -> bool {
    if (attempt_seed.size() != expected_dof)
    {
      std::ostringstream stream;
      stream << label << " skipped: seed size " << attempt_seed.size() << " does not match "
             << expected_dof;
      attempt_logs.push_back(stream.str());
      return false;
    }

    ++attempt_count;

    moveit::core::RobotState working_state(reference_state);
    working_state.setJointGroupPositions(joint_model_group, attempt_seed);
    working_state.update();

    const bool solved = working_state.setFromIK(joint_model_group, target_pose, kIkTimeoutSec);
    if (!solved)
    {
      std::ostringstream stream;
      stream << label << " failed";
      attempt_logs.push_back(stream.str());
      return false;
    }

    working_state.copyJointGroupPositions(joint_model_group, joint_solution);
    return true;
  };

  if (run_attempt(*initial_state, seed_state, "attempt 1/current_seed"))
  {
    return true;
  }

  if (attempt_count < kMaxIkAttempts)
  {
    std::vector<double> fallback_seed;
    std::string fallback_reason;
    if (build_fallback_seed(fallback_seed, fallback_reason))
    {
      if (run_attempt(*initial_state, fallback_seed, "attempt 2/fallback_seed_hook"))
      {
        return true;
      }
    }
    else
    {
      attempt_logs.push_back("attempt 2/fallback_seed_hook skipped: " + fallback_reason);
    }
  }

  if (attempt_count < kMaxIkAttempts)
  {
    moveit::core::RobotStatePtr refreshed_state = move_group_->getCurrentState(5.0);
    if (!refreshed_state)
    {
      attempt_logs.push_back("attempt 3/refreshed_current_state skipped: failed to refresh robot state");
    }
    else
    {
      std::vector<double> refreshed_seed;
      refreshed_state->copyJointGroupPositions(joint_model_group, refreshed_seed);
      if (run_attempt(*refreshed_state, refreshed_seed, "attempt 3/refreshed_current_state"))
      {
        return true;
      }
    }
  }

  std::ostringstream stream;
  stream << "IK failed after " << attempt_count << " attempt(s); " << join_messages(attempt_logs);
  reason = stream.str();
  return false;
}

bool IkSelector::build_fallback_seed(std::vector<double> & fallback_seed, std::string & reason) const
{
  fallback_seed.clear();

  if (!move_group_)
  {
    reason = "MoveGroupInterface is not set";
    return false;
  }

  const auto named_targets = move_group_->getNamedTargets();
  const std::array<std::string, 2> preferred_targets = { "ready", "home" };

  for (const auto & target_name : preferred_targets)
  {
    const bool available = std::find(named_targets.begin(), named_targets.end(), target_name) != named_targets.end();
    if (!available)
    {
      continue;
    }

    const auto named_values = move_group_->getNamedTargetValues(target_name);
    if (named_values.empty())
    {
      continue;
    }

    std::vector<double> ordered_seed;
    ordered_seed.reserve(named_values.size());

    for (const auto & joint_name : move_group_->getJointNames())
    {
      const auto it = named_values.find(joint_name);
      if (it == named_values.end())
      {
        ordered_seed.clear();
        break;
      }
      ordered_seed.push_back(it->second);
    }

    if (!ordered_seed.empty())
    {
      fallback_seed = ordered_seed;
      reason.clear();
      return true;
    }
  }

  reason = "named-target fallback hook did not find usable 'ready' or 'home' target";
  return false;
}
}  // namespace motion_core
