#ifndef MOTION_CORE__TRAJECTORY_VALIDATOR_HPP_
#define MOTION_CORE__TRAJECTORY_VALIDATOR_HPP_

#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core {

bool is_finite_vector(const std::vector<double> &values);

bool validate_trajectory_structure(
    const trajectory_msgs::msg::JointTrajectory &traj, std::string &reason);

} // namespace motion_core

#endif // MOTION_CORE__TRAJECTORY_VALIDATOR_HPP_
