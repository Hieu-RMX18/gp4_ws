#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace motion_core {
enum class ExecutionPhase {
  kIdle,
  kAccepted,
  kPlanning,
  kDispatchWait,
  kExecuting,
};

const char *execution_phase_name(ExecutionPhase phase);

struct ExecutionOrchestratorSnapshot {
  bool active = false;
  std::uint64_t sequence = 0U;
  std::string primitive;
  ExecutionPhase phase = ExecutionPhase::kIdle;
  bool stop_requested = false;
  std::string detail;
};

struct ExecutionStartResult {
  bool acquired = false;
  std::uint64_t sequence = 0U;
  std::string reason;
};

class ExecutionOrchestrator {
public:
  ExecutionStartResult begin_goal(const std::string &primitive);
  void update_phase(std::uint64_t sequence, ExecutionPhase phase,
                    const std::string &detail = std::string());
  bool request_stop(std::string &reason);
  bool stop_requested(std::uint64_t sequence) const;
  ExecutionOrchestratorSnapshot snapshot() const;
  void finish_goal(std::uint64_t sequence,
                   const std::string &detail = std::string());

private:
  mutable std::mutex mutex_;
  std::uint64_t next_sequence_ = 1U;
  ExecutionOrchestratorSnapshot active_;
};
} // namespace motion_core
