#include "motion_core/planner_router.hpp"

#include <algorithm>
#include <cctype>

namespace motion_core
{
namespace
{
std::string normalized_primitive(std::string value)
{
  value.erase(
    std::remove_if(value.begin(), value.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }),
    value.end());

  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

  return value;
}
}  // namespace

std::string PlannerRouter::route_planner(
  const std::string & primitive_type,
  bool has_obstacle_context) const
{
  if (has_obstacle_context)
  {
    return "OMPL_RRTConnect";
  }

  const std::string primitive = normalized_primitive(primitive_type);

  if (primitive == "COMPLEX")
  {
    return "OMPL_RRTConnect";
  }

  if (primitive == "LIN" || primitive == "MOVE_REL")
  {
    return "PILZ_LIN";
  }
  if (
    primitive == "PTP" ||
    primitive == "MOVE_JOINT" ||
    primitive == "MOVE_JOINTS")
  {
    return "PILZ_PTP";
  }
  if (primitive == "CIRC")
  {
    return "PILZ_CIRC";
  }

  return "";
}
}  // namespace motion_core
