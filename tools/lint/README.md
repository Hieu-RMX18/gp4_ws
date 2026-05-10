# GP4 Custom Lint Tools

Project-specific checks that run in CI. Thresholds and exceptions are **adjustable** as the codebase evolves — don't bypass them, update them.

## Checks

**`file_size_budget.sh`** — Python ≤ 700 LOC, C++ ≤ 900 LOC.
Adjust thresholds directly in the script. Files with a planned split → add to `file_size_exceptions.txt`:
```
src/path/to/file.py  <LOC>  <reason>  [target wave]
```

**`no_magic_motion_numbers.sh`** — Forbids hardcoded `velocity_scale`, `acceleration_scale`, and joint limits in production code. Motion params belong in YAML config only (SSOT rule).
To add an exclusion: append `--glob '!src/my_package/**'` or `| grep -v 'MY_CONSTANT'` to the script.

**`no_silent_motion_fallback.sh`** — Asserts that `computeCartesianPath` appears exactly once in `primitive_router_dispatch.cpp` (invariant established in W1).
If the file is renamed or split, update the `TARGET` variable and `CALL_COUNT` threshold in the script.

**`aged_deprecation_check.py`** — Detects `DEPRECATED` tags whose `removal_date` has passed.
Tag format: `# DEPRECATED removal_date=YYYY-MM-DD`. Add scan paths in `subprocess.check_output` if needed.

## Modifying or removing a check

If a check no longer fits, **do not change or delete it unilaterally**. Confirm with the user first:

> "Check `<name>` is failing because `<reason>`. I want to `<adjust threshold / add exclusion / remove check>` because `<technical reason>`. Confirm?"

Only proceed after the user agrees, and record the reason in the commit message.

## Running manually

```bash
bash tools/lint/file_size_budget.sh
bash tools/lint/no_magic_motion_numbers.sh
bash tools/lint/no_silent_motion_fallback.sh
python3 tools/lint/aged_deprecation_check.py
```

Requires: `rg` (ripgrep).
