#include <string>

#include <gtest/gtest.h>

#include "motion_core/execution_orchestrator.hpp"

namespace motion_core
{
TEST(ExecutionOrchestratorTest, RejectsSecondGoalWhileFirstIsActive)
{
  ExecutionOrchestrator orchestrator;

  const auto first = orchestrator.begin_goal("PTP");
  ASSERT_TRUE(first.acquired);

  const auto second = orchestrator.begin_goal("LIN");
  EXPECT_FALSE(second.acquired);
  EXPECT_NE(second.reason.find("active goal"), std::string::npos);
}

TEST(ExecutionOrchestratorTest, StopRequestTracksActiveGoalAndClearsOnFinish)
{
  ExecutionOrchestrator orchestrator;

  const auto started = orchestrator.begin_goal("PTP");
  ASSERT_TRUE(started.acquired);
  orchestrator.update_phase(started.sequence, ExecutionPhase::kDispatchWait, "dispatch pending");

  std::string reason;
  EXPECT_TRUE(orchestrator.request_stop(reason));
  EXPECT_TRUE(orchestrator.stop_requested(started.sequence));
  EXPECT_NE(reason.find("dispatch_wait"), std::string::npos);

  orchestrator.finish_goal(started.sequence, "completed");
  EXPECT_FALSE(orchestrator.stop_requested(started.sequence));
  EXPECT_FALSE(orchestrator.snapshot().active);
}

TEST(ExecutionOrchestratorTest, StopWithoutActiveGoalFailsClosed)
{
  ExecutionOrchestrator orchestrator;

  std::string reason;
  EXPECT_FALSE(orchestrator.request_stop(reason));
  EXPECT_NE(reason.find("no active"), std::string::npos);
}

TEST(ExecutionOrchestratorTest, ExposesExecutingPhaseName)
{
  EXPECT_STREQ(execution_phase_name(ExecutionPhase::kExecuting), "executing");
}
}  // namespace motion_core
