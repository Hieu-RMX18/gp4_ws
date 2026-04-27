#include <gtest/gtest.h>

#include <string>

#include "motion_core/execute_motion_action_support.hpp"

namespace
{
TEST(ExecuteMotionActionSupportTest, NormalizePrimitiveUppercases)
{
  EXPECT_EQ(motion_core::ExecuteMotionActionSupport::normalize_primitive("lin"), "LIN");
  EXPECT_EQ(motion_core::ExecuteMotionActionSupport::normalize_primitive("Home"), "HOME");
  EXPECT_EQ(motion_core::ExecuteMotionActionSupport::normalize_primitive("PTP"), "PTP");
}

TEST(ExecuteMotionActionSupportTest, SupportedPrimitivesIncludeCoreSet)
{
  EXPECT_TRUE(motion_core::ExecuteMotionActionSupport::is_supported_primitive("LIN"));
  EXPECT_TRUE(motion_core::ExecuteMotionActionSupport::is_supported_primitive("PTP"));
  EXPECT_TRUE(motion_core::ExecuteMotionActionSupport::is_supported_primitive("HOME"));
  EXPECT_FALSE(motion_core::ExecuteMotionActionSupport::is_supported_primitive("UNKNOWN"));
}
}  // namespace
