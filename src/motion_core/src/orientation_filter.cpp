#include "motion_core/orientation_filter.hpp"

#include <cmath>

namespace motion_core
{
bool OrientationFilter::normalize_and_validate(geometry_msgs::msg::Pose & pose, std::string & reason) const
{
  reason.clear();

  const auto & q_in = pose.orientation;
  const double norm_sq =
    (q_in.x * q_in.x) + (q_in.y * q_in.y) + (q_in.z * q_in.z) + (q_in.w * q_in.w);

  if (norm_sq <= 1e-12)
  {
    reason = "quaternion norm is zero";
    return false;
  }

  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  pose.orientation.x *= inv_norm;
  pose.orientation.y *= inv_norm;
  pose.orientation.z *= inv_norm;
  pose.orientation.w *= inv_norm;

  return validate_policy_extension(pose, reason);
}

bool OrientationFilter::validate_policy_extension(
  const geometry_msgs::msg::Pose & pose,
  std::string & reason) const
{
  (void)pose;
  reason.clear();
  return true;
}
}  // namespace motion_core
