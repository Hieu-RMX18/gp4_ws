#include "motion_core/angle_branch_utils.hpp"

#include <cmath>
#include <sstream>

#include <angles/angles.h>

namespace motion_core
{
namespace
{
constexpr double kBoundsTolerance = 1e-9;

bool is_large_limit_interval(const double lower_limit, const double upper_limit)
{
  return lower_limit < -M_PI || upper_limit > M_PI || (upper_limit - lower_limit) > (2.0 * M_PI);
}

bool is_within_bounds(const double value, const double lower_limit, const double upper_limit)
{
  return value >= (lower_limit - kBoundsTolerance) && value <= (upper_limit + kBoundsTolerance);
}
}  // namespace

double normalize_angle_for_display(const double angle)
{
  return angles::normalize_angle(angle);
}

std::vector<double> normalize_joint_vector_for_display(const std::vector<double> & joints)
{
  std::vector<double> normalized;
  normalized.reserve(joints.size());
  for (const double joint : joints)
  {
    normalized.push_back(normalize_angle_for_display(joint));
  }
  return normalized;
}

BranchPreservedAngleResult choose_branch_preserved_angle(
  const double current,
  const double requested,
  const double lower_limit,
  const double upper_limit)
{
  BranchPreservedAngleResult result;

  if (!std::isfinite(current) || !std::isfinite(requested) ||
      !std::isfinite(lower_limit) || !std::isfinite(upper_limit))
  {
    result.reason = "branch-preserved angle selection requires finite current, requested, and limit values";
    return result;
  }

  double delta = 0.0;
  bool success = false;

  if (is_large_limit_interval(lower_limit, upper_limit))
  {
    if (lower_limit > upper_limit)
    {
      result.reason =
        "large-limit angle helper requires ordered joint bounds (lower_limit <= upper_limit)";
      return result;
    }

    result.helper_used = "shortest_angular_distance_with_large_limits";
    success = angles::shortest_angular_distance_with_large_limits(
      current, requested, lower_limit, upper_limit, delta);
  }
  else
  {
    result.helper_used = "shortest_angular_distance_with_limits";
    success = angles::shortest_angular_distance_with_limits(
      current, requested, lower_limit, upper_limit, delta);
  }

  if (!success)
  {
    std::ostringstream stream;
    stream << "no legal equivalent target exists for current=" << current
           << ", requested=" << requested
           << ", bounds=[" << lower_limit << ", " << upper_limit << "]";
    result.reason = stream.str();
    return result;
  }

  const double chosen_target = current + delta;
  if (!is_within_bounds(chosen_target, lower_limit, upper_limit))
  {
    std::ostringstream stream;
    stream << "chosen target " << chosen_target << " violates bounds ["
           << lower_limit << ", " << upper_limit << "]";
    result.reason = stream.str();
    return result;
  }

  result.success = true;
  result.chosen_target = chosen_target;
  result.delta_from_current = delta;
  result.reason.clear();
  return result;
}

BranchPreservedJointVectorResult choose_branch_preserved_joint_vector(
  const std::vector<const moveit::core::JointModel *> & joint_models,
  const std::vector<double> & current,
  const std::vector<double> & requested)
{
  BranchPreservedJointVectorResult result;

  if (joint_models.size() != current.size() || current.size() != requested.size())
  {
    std::ostringstream stream;
    stream << "joint vector size mismatch: models=" << joint_models.size()
           << ", current=" << current.size()
           << ", requested=" << requested.size();
    result.reason = stream.str();
    return result;
  }

  result.chosen_targets.reserve(requested.size());
  result.deltas_from_current.reserve(requested.size());
  result.helper_used.reserve(requested.size());

  for (std::size_t index = 0; index < joint_models.size(); ++index)
  {
    const auto * joint_model = joint_models[index];
    if (joint_model == nullptr)
    {
      result.reason = "joint model pointer is null at index " + std::to_string(index);
      return result;
    }

    if (joint_model->getVariableCount() != 1U)
    {
      result.reason =
        "branch-preserved joint vectors currently support single-DOF joints only; unsupported joint '" +
        joint_model->getName() + "'";
      return result;
    }

    const double current_value = current[index];
    const double requested_value = requested[index];

    if (joint_model->getType() != moveit::core::JointModel::REVOLUTE)
    {
      result.chosen_targets.push_back(requested_value);
      result.deltas_from_current.push_back(requested_value - current_value);
      result.helper_used.emplace_back("non_revolute_passthrough");
      continue;
    }

    const auto & bounds = joint_model->getVariableBounds();
    if (bounds.empty())
    {
      result.reason = "joint '" + joint_model->getName() + "' does not expose bounds";
      return result;
    }

    const auto & bound = bounds.front();
    if (!bound.position_bounded_)
    {
      const double delta = angles::shortest_angular_distance(current_value, requested_value);
      result.chosen_targets.push_back(current_value + delta);
      result.deltas_from_current.push_back(delta);
      result.helper_used.emplace_back("shortest_angular_distance_unbounded");
      continue;
    }

    auto angle_result = choose_branch_preserved_angle(
      current_value,
      requested_value,
      bound.min_position_,
      bound.max_position_);
    if (!angle_result.success)
    {
      std::ostringstream stream;
      stream << "joint '" << joint_model->getName() << "' branch preservation failed: "
             << angle_result.reason;
      result.reason = stream.str();
      return result;
    }

    result.chosen_targets.push_back(angle_result.chosen_target);
    result.deltas_from_current.push_back(angle_result.delta_from_current);
    result.helper_used.push_back(angle_result.helper_used);
  }

  result.success = true;
  result.reason.clear();
  return result;
}
}  // namespace motion_core
