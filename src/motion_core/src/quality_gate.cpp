#include "motion_core/quality_gate.hpp"

#include <cmath>

namespace motion_core
{
QualityGate::QualityGate(std::size_t max_trajectory_points, double minimum_cartesian_fraction)
: max_trajectory_points_(max_trajectory_points),
  minimum_cartesian_fraction_(minimum_cartesian_fraction)
{
}

bool QualityGate::validate_plan(
  const trajectory_msgs::msg::JointTrajectory & traj,
  double fraction,
  std::string & reason) const
{
  reason.clear();

  if (traj.points.empty())
  {
    reason = "trajectory has no points";
    return false;
  }

  if (!wrist_flip_guard_.check_trajectory(traj, reason))
  {
    return false;
  }

  if (traj.points.size() > max_trajectory_points_)
  {
    reason = "trajectory exceeds point limit";
    return false;
  }

  // Safe convention: fraction < 0.0 means non-Cartesian, so fraction check is skipped.
  if (fraction >= 0.0)
  {
    if (!std::isfinite(fraction))
    {
      reason = "cartesian fraction is non-finite";
      return false;
    }

    if (fraction > 1.0)
    {
      reason = "cartesian fraction is above 1.0";
      return false;
    }

    if (fraction < minimum_cartesian_fraction_)
    {
      reason = "cartesian fraction below minimum threshold";
      return false;
    }
  }

  reason.clear();
  return true;
}
}  // namespace motion_core
