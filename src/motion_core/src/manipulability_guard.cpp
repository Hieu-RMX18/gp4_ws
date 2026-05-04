#include "motion_core/manipulability_guard.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace motion_core {

ManipulabilityGuard::ManipulabilityGuard(moveit::core::RobotModelConstPtr model,
                                         std::string group_name, double floor,
                                         std::size_t sample_every_n_points)
    : model_(std::move(model)), group_name_(std::move(group_name)),
      floor_(floor),
      sample_every_n_points_(sample_every_n_points > 0 ? sample_every_n_points
                                                       : 1) {}

double ManipulabilityGuard::compute_yoshikawa_index(
    const std::vector<double> &joint_positions) const {
  if (!model_) {
    return 1.0;
  }

  moveit::core::RobotState state(model_);
  const auto *group = state.getJointModelGroup(group_name_);
  if (!group) {
    return 1.0;
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
  return std::sqrt(std::max(jjt.determinant(), 0.0));
}

bool ManipulabilityGuard::check_trajectory(
    const trajectory_msgs::msg::JointTrajectory &traj,
    std::string &reason) const {
  reason.clear();

  if (!model_) {
    return true;
  }

  for (std::size_t i = 0; i < traj.points.size(); i += sample_every_n_points_) {
    const auto &point = traj.points[i];
    const double w = compute_yoshikawa_index(point.positions);
    if (w < floor_) {
      std::ostringstream stream;
      stream << "manipulability_guard reject at point[" << i
             << "]: w=" << std::fixed << std::setprecision(4) << w
             << " < floor " << std::fixed << std::setprecision(4) << floor_;
      reason = stream.str();
      return false;
    }
  }

  return true;
}

} // namespace motion_core
