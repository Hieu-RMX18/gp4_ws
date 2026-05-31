#pragma once

#include <string>

namespace motion_core {
class PlannerRouter {
public:
  std::string route_planner(const std::string &primitive_type,
                            bool has_obstacle_context = false) const;
};
} // namespace motion_core
