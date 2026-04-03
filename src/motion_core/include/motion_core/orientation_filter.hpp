#pragma once

#include <string>

#include <geometry_msgs/msg/pose.hpp>

namespace motion_core
{
class OrientationFilter
{
public:
  bool normalize_and_validate(geometry_msgs::msg::Pose & pose, std::string & reason) const;

private:
  bool validate_policy_extension(const geometry_msgs::msg::Pose & pose, std::string & reason) const;
};
}  // namespace motion_core
