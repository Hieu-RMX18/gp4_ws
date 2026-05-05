#!/usr/bin/env python3
"""Check for DEPRECATED tags whose removal_date has passed.

Exit 0 if all deprecations are still in their cooldown period.
Exit 1 if any deprecation is past its removal_date (cleanup wave missed it).

Part of the W6 recurring cleanup wave enforcement (5-mechanism anti-bloat layer 3).
"""

from __future__ import annotations

import re
import subprocess
import sys
from datetime import datetime, timezone


def main() -> int:
    today = datetime.now(timezone.utc).date()

    try:
        out = subprocess.check_output(
            [
                "rg",
                "-n",
                r"DEPRECATED.*removal_date",
                "--type", "py",
                "--type", "cpp",
                "src/",
                "hmi/backend/",
            ],
            text=True,
        )
    except subprocess.CalledProcessError:
        # rg exits 1 when no matches — that's fine, no deprecations at all.
        print("aged-deprecation-check: PASS (no DEPRECATED tags found)")
        return 0

    violations: list[tuple[str, int, str]] = []
    for line in out.splitlines():
        m = re.search(r"removal_date=(\d{4}-\d{2}-\d{2})", line)
        if not m:
            continue
        rd = datetime.strptime(m.group(1), "%Y-%m-%d").date()
        if rd < today:
            parts = line.split(":", 2)
            filepath = parts[0]
            lineno = int(parts[1]) if len(parts) > 1 else 0
            violations.append((filepath, lineno, m.group(1)))

    if violations:
        print(
            f"aged-deprecation-check: FAIL — {len(violations)} deprecation(s) "
            "past their removal_date:"
        )
        for filepath, lineno, date_str in violations:
            print(f"  {filepath}:{lineno}  removal_date={date_str}")
        return 1

    print("aged-deprecation-check: PASS (all deprecations still in cooldown)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
