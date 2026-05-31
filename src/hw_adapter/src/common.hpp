#pragma once
#include <chrono>
#include <sstream>
#include <string>

namespace hw_adapter {
inline std::string timeout_reason(const std::string &contract_type,
                                  const std::string &contract_name,
                                  const std::chrono::milliseconds timeout) {
  std::ostringstream oss;
  oss << "Timed out after " << timeout.count() << " ms waiting for "
      << contract_type << " '" << contract_name << "'";
  return oss.str();
}
} // namespace hw_adapter
