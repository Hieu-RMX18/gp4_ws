#include <gtest/gtest.h>

#include <string>

#include "motion_core/execute_motion_action_support.hpp"

namespace
{
TEST(ExecuteMotionActionSupportTest, ApprovalRejectionMessageDescribesDeprecatedDirectField)
{
  const std::string message = motion_core::ExecuteMotionActionSupport::approval_rejected_message();

  EXPECT_NE(message.find("require_approval=true"), std::string::npos);
  EXPECT_NE(message.find("deprecated unsupported field"), std::string::npos);
  EXPECT_NE(message.find("direct callers must send require_approval=false"), std::string::npos);
  EXPECT_NE(message.find("plan-only is not supported by this action"), std::string::npos);
}
}  // namespace
