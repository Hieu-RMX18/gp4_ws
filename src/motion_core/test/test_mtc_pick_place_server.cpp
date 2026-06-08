#include <gtest/gtest.h>

#include "motion_core/mtc_pick_place_server.hpp"

TEST(MtcPickPlaceServer, ReportsUnavailableWhenMtcIsNotCompiled)
{
  const auto result = motion_core::make_mtc_unavailable_result("missing dependency");
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.status, "capability_unavailable");
  EXPECT_NE(result.message.find("missing dependency"), std::string::npos);
}
