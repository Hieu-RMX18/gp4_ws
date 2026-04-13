#include "motion_core/trajectory_post_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#if __has_include(<moveit/trajectory_processing/ruckig_traj_smoothing.h>) && __has_include(<ruckig/ruckig.hpp>)
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#define MOTION_CORE_HAS_RUCKIG 1
#else
#define MOTION_CORE_HAS_RUCKIG 0
#endif

namespace motion_core
{
namespace
{
double resolve_scale(double requested, double fallback)
{
  return (requested > 0.0) ? requested : fallback;
}

bool is_valid_scale(double value)
{
  return std::isfinite(value) && value > 0.0 && value <= 1.0;
}
}  // namespace

bool TrajectoryPostProcessor::apply_totg(
  robot_trajectory::RobotTrajectory & traj,
  double vel_scale,
  double acc_scale,
  std::string & reason) const
{
  reason.clear();

  if (traj.getWayPointCount() < 2U)
  {
    reason = "TOTG requires at least 2 waypoints";
    return false;
  }

  const double resolved_vel = resolve_scale(vel_scale, kDefaultVelocityScaling);
  const double resolved_acc = resolve_scale(acc_scale, kDefaultAccelerationScaling);

  if (!is_valid_scale(resolved_vel))
  {
    std::ostringstream stream;
    stream << "invalid velocity scaling: " << resolved_vel;
    reason = stream.str();
    return false;
  }

  if (!is_valid_scale(resolved_acc))
  {
    std::ostringstream stream;
    stream << "invalid acceleration scaling: " << resolved_acc;
    reason = stream.str();
    return false;
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(traj, resolved_vel, resolved_acc))
  {
    reason = "TOTG computeTimeStamps failed";
    return false;
  }

  return true;
}

bool TrajectoryPostProcessor::downsample_to_max_points(
  trajectory_msgs::msg::JointTrajectory & traj,
  std::size_t max_points,
  std::string & reason) const
{
  reason.clear();

  if (traj.points.empty())
  {
    reason = "trajectory has no points";
    return false;
  }

  if (max_points == 0)
  {
    reason = "max_points must be greater than 0";
    return false;
  }

  const std::size_t point_count = traj.points.size();
  if (point_count <= max_points)
  {
    return true;
  }

  if (max_points < 2)
  {
    reason = "cannot downsample to less than 2 points while preserving endpoints";
    return false;
  }

  trajectory_msgs::msg::JointTrajectory downsampled;
  downsampled.header = traj.header;
  downsampled.joint_names = traj.joint_names;
  downsampled.points.reserve(max_points);

  const double step = static_cast<double>(point_count - 1) / static_cast<double>(max_points - 1);
  std::size_t last_index = 0;

  for (std::size_t i = 0; i < max_points; ++i)
  {
    std::size_t index;
    if (i == 0)
    {
      index = 0;
    }
    else if (i == max_points - 1)
    {
      index = point_count - 1;
    }
    else
    {
      index = static_cast<std::size_t>(std::llround(static_cast<double>(i) * step));
      if (index <= last_index)
      {
        index = std::min(last_index + 1, point_count - 1);
      }
    }

    downsampled.points.push_back(traj.points[index]);
    last_index = index;
  }

  if (downsampled.points.front().positions != traj.points.front().positions ||
      downsampled.points.back().positions != traj.points.back().positions)
  {
    reason = "downsampling failed to preserve boundary points";
    return false;
  }

  traj.points = std::move(downsampled.points);
  return true;
}

bool TrajectoryPostProcessor::apply_ruckig_smoothing(
  robot_trajectory::RobotTrajectory & traj,
  double vel_scale,
  double acc_scale,
  std::string & reason) const
{
  reason.clear();
  if (traj.getWayPointCount() == 0) {
    reason = "trajectory has no waypoints";
    return false;
  }

  const double resolved_vel = resolve_scale(vel_scale, kDefaultVelocityScaling);
  const double resolved_acc = resolve_scale(acc_scale, kDefaultAccelerationScaling);

  if (!is_valid_scale(resolved_vel)) {
    std::ostringstream stream;
    stream << "invalid velocity scaling for Ruckig: " << resolved_vel;
    reason = stream.str();
    return false;
  }
  if (!is_valid_scale(resolved_acc)) {
    std::ostringstream stream;
    stream << "invalid acceleration scaling for Ruckig: " << resolved_acc;
    reason = stream.str();
    return false;
  }

#if MOTION_CORE_HAS_RUCKIG
  trajectory_processing::RuckigSmoothing smoother;
  if (!smoother.applySmoothing(traj, resolved_vel, resolved_acc)) {
    reason = "RuckigSmoothing::applySmoothing failed";
    return false;
  }
  reason = "Ruckig smoothing applied successfully";
  return true;
#else
  reason = "Ruckig smoothing unavailable in this build; optional step skipped.";
  return true;
#endif
}
}  // namespace motion_core
