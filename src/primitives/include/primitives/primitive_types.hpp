// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <algorithm>
#include <cctype>
#include <string>

namespace primitives
{
enum class PrimitiveType
{
  HOME,
  PTP,
  LIN,
  APPROACH,
  RETRACT,
  CIRC,
  BLENDED_SEQUENCE,
  UNKNOWN
};

enum class PrimitiveFailReason
{
  UNKNOWN = 0,
  NAMED_TARGET_NOT_FOUND,
  IK_FAILED,
  CARTESIAN_FRACTION_LOW,
  JOINT_COUNT_MISMATCH,
  INVALID_ORIENTATION,
  WRIST_FLIP_DETECTED,
  TRAJECTORY_TOO_LONG,
  DEGENERATE_GEOMETRY,
  QUALITY_GATE_REJECTED,
  SUB_PRIMITIVE_FAILED,
  PLANNING_TIMEOUT,
  WORKSPACE_VIOLATION,
  INVALID_DISTANCE_PARAM,
  TOTG_FAILED
};

struct PrimitiveResult
{
  bool success = false;
  PrimitiveFailReason reason = PrimitiveFailReason::UNKNOWN;
  std::string message;  // human-readable detail
  double planning_time_sec = 0.0;
  std::size_t trajectory_points = 0;
};

inline PrimitiveType from_string(const std::string & s)
{
  std::string normalized;
  normalized.reserve(s.size());

  for (const char c : s)
  {
    if (std::isspace(static_cast<unsigned char>(c)) != 0)
    {
      continue;
    }

    if (c == '-')
    {
      normalized.push_back('_');
      continue;
    }

    normalized.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  }

  if (normalized == "HOME")
  {
    return PrimitiveType::HOME;
  }
  if (normalized == "PTP")
  {
    return PrimitiveType::PTP;
  }
  if (normalized == "LIN")
  {
    return PrimitiveType::LIN;
  }
  if (normalized == "APPROACH")
  {
    return PrimitiveType::APPROACH;
  }
  if (normalized == "RETRACT")
  {
    return PrimitiveType::RETRACT;
  }
  if (normalized == "CIRC")
  {
    return PrimitiveType::CIRC;
  }
  if (normalized == "BLENDED_SEQUENCE" || normalized == "BLENDEDSEQUENCE")
  {
    return PrimitiveType::BLENDED_SEQUENCE;
  }

  return PrimitiveType::UNKNOWN;
}

inline std::string to_string(PrimitiveType t)
{
  switch (t)
  {
    case PrimitiveType::HOME:
      return "HOME";
    case PrimitiveType::PTP:
      return "PTP";
    case PrimitiveType::LIN:
      return "LIN";
    case PrimitiveType::APPROACH:
      return "APPROACH";
    case PrimitiveType::RETRACT:
      return "RETRACT";
    case PrimitiveType::CIRC:
      return "CIRC";
    case PrimitiveType::BLENDED_SEQUENCE:
      return "BLENDED_SEQUENCE";
    case PrimitiveType::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

inline bool is_joint_space(PrimitiveType t)
{
  return t == PrimitiveType::HOME || t == PrimitiveType::PTP;
}

inline bool is_cartesian_linear(PrimitiveType t)
{
  return t == PrimitiveType::LIN || t == PrimitiveType::APPROACH || t == PrimitiveType::RETRACT;
}

inline bool is_geometric(PrimitiveType t)
{
  return t == PrimitiveType::CIRC;
}

inline bool is_composite(PrimitiveType t)
{
  return t == PrimitiveType::BLENDED_SEQUENCE;
}
}  // namespace primitives
