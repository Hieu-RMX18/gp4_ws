#include "motion_core/wrist_flip_guard.hpp"

#include <cmath>
#include <sstream>

namespace motion_core
{
bool WristFlipGuard::check_trajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::string & reason) const
{
  reason.clear();

  if (traj.points.empty())
  {
    reason = "trajectory has no points";
    return false;
  }

  if (traj.joint_names.empty())
  {
    reason = "trajectory has no joint names";
    return false;
  }

  const std::size_t expected_joint_count = traj.joint_names.size();

  for (std::size_t i = 0; i < traj.points.size(); ++i)
  {
    const auto & point = traj.points[i];
    if (point.positions.size() != expected_joint_count)
    {
      std::ostringstream stream;
      stream << "malformed trajectory point " << i << ": expected " << expected_joint_count
             << " positions, got " << point.positions.size();
      reason = stream.str();
      return false;
    }

    for (std::size_t joint_idx = 0; joint_idx < point.positions.size(); ++joint_idx)
    {
      if (!std::isfinite(point.positions[joint_idx]))
      {
        std::ostringstream stream;
        stream << "malformed trajectory point " << i << ": non-finite position at joint index "
               << joint_idx;
        reason = stream.str();
        return false;
      }
    }
  }

  if (traj.points.size() < 2)
  {
    return true;
  }

  for (std::size_t i = 1; i < traj.points.size(); ++i)
  {
    const auto & previous = traj.points[i - 1];
    const auto & current = traj.points[i];

    for (std::size_t joint_idx = 0; joint_idx < current.positions.size(); ++joint_idx)
    {
      const double delta = std::abs(current.positions[joint_idx] - previous.positions[joint_idx]);
      if (delta > kMaxJointDeltaRad)
      {
        std::ostringstream stream;
        stream << "wrist flip guard reject at segment " << (i - 1) << "->" << i
               << ", joint '" << traj.joint_names[joint_idx]
               << "': delta=" << delta << " rad > " << kMaxJointDeltaRad << " rad";
        reason = stream.str();
        return false;
      }
    }
  }

  return true;
}
}  // namespace motion_core
