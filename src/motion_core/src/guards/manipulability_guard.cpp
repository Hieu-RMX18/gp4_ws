#include "motion_core/manipulability_guard.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace motion_core {

double normalize_yoshikawa_index(const double raw_index,
                                 const double reference_length_m) {
  if (!std::isfinite(raw_index) || raw_index < 0.0 ||
      !std::isfinite(reference_length_m) || reference_length_m <= 0.0) {
    return 0.0;
  }

  const double reference_volume =
      reference_length_m * reference_length_m * reference_length_m;
  if (!std::isfinite(reference_volume) || reference_volume <= 0.0) {
    return 0.0;
  }

  return raw_index / reference_volume;
}

bool check_manipulability_samples(
    const double floor, const std::vector<ManipulabilitySample> &samples,
    std::string &reason) {
  reason.clear();
  if (samples.size() <= 1) {
    return true;
  }

  constexpr double kRecoveryTolerance = 1e-6;
  const double start_w = samples.front().value;
  const bool recovery_from_low_manipulability = start_w < floor;

  if (recovery_from_low_manipulability) {
    for (std::size_t i = 1; i < samples.size(); ++i) {
      const auto &sample = samples[i];
      if (sample.value + kRecoveryTolerance < start_w) {
        std::ostringstream stream;
        stream << "manipulability_guard recovery rejected at point["
               << sample.index << "]: w=" << std::fixed
               << std::setprecision(4) << sample.value
               << " below recovery start " << std::fixed
               << std::setprecision(4) << start_w;
        reason = stream.str();
        return false;
      }
    }

    const auto &final_sample = samples.back();
    if (final_sample.value < floor) {
      std::ostringstream stream;
      stream << "manipulability_guard recovery failed: final w=" << std::fixed
             << std::setprecision(4) << final_sample.value << " < floor "
             << std::fixed << std::setprecision(4) << floor;
      reason = stream.str();
      return false;
    }
    return true;
  }

  for (std::size_t i = 1; i < samples.size(); ++i) {
    const auto &sample = samples[i];
    if (sample.value < floor) {
      std::ostringstream stream;
      stream << "manipulability_guard reject at point[" << sample.index
             << "]: w=" << std::fixed << std::setprecision(4) << sample.value
             << " < floor " << std::fixed << std::setprecision(4) << floor;
      reason = stream.str();
      return false;
    }
  }

  return true;
}

ManipulabilityGuard ManipulabilityGuard::disabled() {
  return ManipulabilityGuard(false);
}

ManipulabilityGuard::ManipulabilityGuard(bool enabled) : enabled_(enabled) {}

ManipulabilityGuard::ManipulabilityGuard(moveit::core::RobotModelConstPtr model,
                                         std::string group_name, double floor,
                                         std::size_t sample_every_n_points,
                                         double reference_length_m)
    : enabled_(true), model_(std::move(model)),
      group_name_(std::move(group_name)), floor_(floor),
      sample_every_n_points_(sample_every_n_points > 0 ? sample_every_n_points
                                                       : 1),
      reference_length_m_(reference_length_m) {}

double ManipulabilityGuard::compute_yoshikawa_index(
    const std::vector<double> &joint_positions) const {
  if (!enabled_ || !model_) {
    return 0.0;
  }

  moveit::core::RobotState state(model_);
  const auto *group = state.getJointModelGroup(group_name_);
  if (!group) {
    return 0.0;
  }

  state.setJointGroupPositions(group, joint_positions);
  state.update();

  Eigen::MatrixXd jacobian;
  if (!state.getJacobian(group,
                         state.getLinkModel(group->getLinkModelNames().back()),
                         Eigen::Vector3d::Zero(), jacobian)) {
    return 0.0;
  }

  const Eigen::MatrixXd jjt = jacobian * jacobian.transpose();
  const double raw_index = std::sqrt(std::max(jjt.determinant(), 0.0));
  return normalize_yoshikawa_index(raw_index, reference_length_m_);
}

bool ManipulabilityGuard::check_trajectory(
    const trajectory_msgs::msg::JointTrajectory &traj,
    std::string &reason) const {
  reason.clear();

  if (!enabled_) {
    return true;
  }
  if (traj.points.size() <= 1) {
    return true;
  }
  if (!model_) {
    reason = "manipulability_guard enabled without robot model";
    return false;
  }

  const double start_w = compute_yoshikawa_index(traj.points.front().positions);
  std::vector<ManipulabilitySample> samples;
  samples.push_back({0, start_w});

  std::vector<std::size_t> sample_indices;
  for (std::size_t i = sample_every_n_points_; i < traj.points.size();
       i += sample_every_n_points_) {
    sample_indices.push_back(i);
  }
  if (sample_indices.empty() ||
      sample_indices.back() != traj.points.size() - 1U) {
    sample_indices.push_back(traj.points.size() - 1U);
  }

  for (const std::size_t i : sample_indices) {
    const auto &point = traj.points[i];
    const double w = compute_yoshikawa_index(point.positions);
    samples.push_back({i, w});
  }

  return check_manipulability_samples(floor_, samples, reason);
}

} // namespace motion_core
