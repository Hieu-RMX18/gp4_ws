#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace motion_core
{
class IkSelector
{
public:
  explicit IkSelector(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group = nullptr,
    std::string planning_group = "gp4_arm");

  void set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);
  void set_planning_group(const std::string & planning_group);

  bool solve_ik(
    const geometry_msgs::msg::Pose & target_pose,
    const std::vector<double> & seed_state,
    std::vector<double> & joint_solution,
    std::string & reason);

private:
  bool build_fallback_seed(std::vector<double> & fallback_seed, std::string & reason) const;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string planning_group_;
};
}  // namespace motion_core
