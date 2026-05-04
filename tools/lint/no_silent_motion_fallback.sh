#!/usr/bin/env bash
# W1.T6: Guard against any silent computeCartesianPath fallback in LIN handling.
#
# Checks:
#   1. The old fallback log message MUST NOT exist anywhere in src/.
#   2. computeCartesianPath calls MUST only appear in the CARTESIAN_PATH primitive scope.
#
# Exit 0 on clean, 1 on violation.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ERRORS=0

# Check 1: The old fallback phrase must be gone.
if grep -rn "attempting computeCartesianPath" "$REPO_ROOT/src/" 2>/dev/null; then
  echo "FAIL: 'attempting computeCartesianPath' still present in src/" >&2
  ERRORS=$((ERRORS + 1))
fi

# Check 2: In primitive_router_dispatch.cpp, exactly 1 call to
# computeCartesianPath must remain (the CARTESIAN_PATH primitive handler).
# More than 1 means a fallback was re-introduced.
TARGET="$REPO_ROOT/src/motion_core/src/primitive_router_dispatch.cpp"
if [ -f "$TARGET" ]; then
  CALL_COUNT=$(grep -c 'move_group->computeCartesianPath\|move_group_->computeCartesianPath' "$TARGET" 2>/dev/null || echo 0)
  if [ "$CALL_COUNT" -gt 1 ]; then
    echo "FAIL: primitive_router_dispatch.cpp has $CALL_COUNT computeCartesianPath calls (expected 1)" >&2
    grep -n 'computeCartesianPath' "$TARGET" >&2
    ERRORS=$((ERRORS + 1))
  elif [ "$CALL_COUNT" -eq 0 ]; then
    echo "WARN: primitive_router_dispatch.cpp has 0 computeCartesianPath calls — CARTESIAN_PATH primitive missing?" >&2
  fi
fi

if [ "$ERRORS" -gt 0 ]; then
  echo "no_silent_motion_fallback: $ERRORS violation(s) found" >&2
  exit 1
fi

echo "no_silent_motion_fallback: all checks passed."
exit 0
