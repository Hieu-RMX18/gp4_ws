#!/usr/bin/env bash
# Fail if velocity / acceleration / joint-limit literals appear in production source.
# Allowed locations: YAML, config dirs, test files, comments, well-known constants.
set -euo pipefail

OFFENDERS=$(rg -n \
  -e '\b(velocity_scale|vel_scale|acceleration_scale|accel_scale)\s*=\s*[0-9]' \
  -e '\bjoint_[0-9]_[a-z]\s*.*[-]?[0-9]+\.[0-9]' \
  -e '\b(max_velocity|max_acceleration|joint_limit)\s*=\s*[0-9]' \
  --type py --type cpp \
  src/ hmi/backend/ \
  --glob '!**/test*' \
  --glob '!**/config/**' \
  --glob '!**/*_test.*' \
  --glob '!**/*.yaml' \
  --glob '!**/*.yml' \
  2>/dev/null \
  | grep -v '^\s*//' \
  | grep -v '^\s*#' \
  | grep -v 'kDefault' \
  | grep -v 'M_PI' \
  | grep -v 'std::numeric_limits' \
  | grep -v 'epsilon' \
  || true)

if [ -n "$OFFENDERS" ]; then
  echo "ERROR: Hardcoded motion/safety numbers found in production source."
  echo "Move these to YAML config (SSOT rule)."
  echo ""
  echo "$OFFENDERS"
  exit 1
fi

echo "no-magic-motion-numbers: PASS"
exit 0
