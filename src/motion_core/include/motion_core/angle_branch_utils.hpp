#pragma once

#include <string>
#include <vector>

#include <moveit/robot_model/joint_model.h>

namespace motion_core {
struct BranchPreservedAngleResult {
  bool success = false;
  double chosen_target = 0.0;
  double delta_from_current = 0.0;
  std::string reason;
  std::string helper_used;
};

struct BranchPreservedJointVectorResult {
  bool success = false;
  std::vector<double> chosen_targets;
  std::vector<double> deltas_from_current;
  std::vector<std::string> helper_used;
  std::string reason;
};

double normalize_angle_for_display(double angle);

std::vector<double>
normalize_joint_vector_for_display(const std::vector<double> &joints);

BranchPreservedAngleResult choose_branch_preserved_angle(double current,
                                                         double requested,
                                                         double lower_limit,
                                                         double upper_limit);

BranchPreservedJointVectorResult choose_branch_preserved_joint_vector(
    const std::vector<const moveit::core::JointModel *> &joint_models,
    const std::vector<double> &current, const std::vector<double> &requested);
} // namespace motion_core
