#include "motion_core/mtc_pick_place_server.hpp"

namespace motion_core
{

MtcPickPlaceResult make_mtc_unavailable_result(const std::string & reason)
{
  return MtcPickPlaceResult{false, "capability_unavailable", "MTC unavailable: " + reason};
}

}  // namespace motion_core
