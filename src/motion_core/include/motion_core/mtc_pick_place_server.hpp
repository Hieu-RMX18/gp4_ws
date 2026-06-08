#pragma once

#include <string>

namespace motion_core
{

struct MtcPickPlaceResult
{
  bool ok{false};
  std::string status;
  std::string message;
};

MtcPickPlaceResult make_mtc_unavailable_result(const std::string & reason);

}  // namespace motion_core
