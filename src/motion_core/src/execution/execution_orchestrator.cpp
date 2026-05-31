#include "motion_core/execution_orchestrator.hpp"

#include <sstream>

namespace motion_core {
const char *execution_phase_name(const ExecutionPhase phase) {
  switch (phase) {
  case ExecutionPhase::kIdle:
    return "idle";
  case ExecutionPhase::kAccepted:
    return "accepted";
  case ExecutionPhase::kPlanning:
    return "planning";
  case ExecutionPhase::kDispatchWait:
    return "dispatch_wait";
  case ExecutionPhase::kExecuting:
    return "executing";
  default:
    return "unknown";
  }
}

ExecutionStartResult
ExecutionOrchestrator::begin_goal(const std::string &primitive) {
  std::lock_guard<std::mutex> lock(mutex_);

  ExecutionStartResult result;
  if (active_.active) {
    std::ostringstream stream;
    stream << "execute_motion already owns an active goal (sequence="
           << active_.sequence << ", primitive=" << active_.primitive
           << ", phase=" << execution_phase_name(active_.phase) << ")";
    result.reason = stream.str();
    return result;
  }

  result.acquired = true;
  result.sequence = next_sequence_++;

  active_.active = true;
  active_.sequence = result.sequence;
  active_.primitive = primitive;
  active_.phase = ExecutionPhase::kAccepted;
  active_.stop_requested = false;
  active_.detail = "goal accepted";
  result.reason.clear();
  return result;
}

void ExecutionOrchestrator::update_phase(const std::uint64_t sequence,
                                         const ExecutionPhase phase,
                                         const std::string &detail) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_.active || active_.sequence != sequence) {
    return;
  }

  active_.phase = phase;
  active_.detail = detail;
}

bool ExecutionOrchestrator::request_stop(std::string &reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_.active) {
    reason = "STOP requested but no active execute_motion goal owns execution";
    return false;
  }

  active_.stop_requested = true;
  std::ostringstream stream;
  stream << "STOP requested for execute_motion sequence=" << active_.sequence
         << " during phase=" << execution_phase_name(active_.phase);
  active_.detail = stream.str();
  reason = active_.detail;
  return true;
}

bool ExecutionOrchestrator::stop_requested(const std::uint64_t sequence) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_.active && active_.sequence == sequence &&
         active_.stop_requested;
}

ExecutionOrchestratorSnapshot ExecutionOrchestrator::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

void ExecutionOrchestrator::finish_goal(const std::uint64_t sequence,
                                        const std::string &detail) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_.active || active_.sequence != sequence) {
    return;
  }

  active_.active = false;
  active_.sequence = 0U;
  active_.primitive.clear();
  active_.phase = ExecutionPhase::kIdle;
  active_.stop_requested = false;
  active_.detail = detail;
}
} // namespace motion_core
