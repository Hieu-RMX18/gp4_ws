#!/usr/bin/env bash
# Fail if any Python file in src/ exceeds 700 LOC or C++ file exceeds 900 LOC,
# unless listed in tools/lint/file_size_exceptions.txt.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXCEPTIONS_FILE="${SCRIPT_DIR}/file_size_exceptions.txt"
FAIL=0

check_files() {
  local pattern="$1"
  local limit="$2"
  local lang="$3"

  while IFS= read -r file; do
    loc=$(wc -l < "$file")
    if [ "$loc" -gt "$limit" ]; then
      if [ -f "$EXCEPTIONS_FILE" ] && grep -q "^${file} " "$EXCEPTIONS_FILE"; then
        continue
      fi
      echo "OVER BUDGET: ${file} (${loc} LOC, limit ${limit} for ${lang})"
      FAIL=1
    fi
  done < <(find src/ -name "$pattern" -not -path '*/test*' -not -path '*/__pycache__/*' 2>/dev/null)
}

check_files '*.py' 700 'Python'
check_files '*.cpp' 900 'C++'

if [ "$FAIL" -ne 0 ]; then
  echo ""
  echo "Add justified exceptions to ${EXCEPTIONS_FILE} or split the file."
  exit 1
fi

echo "file-size-budget: PASS"
exit 0
