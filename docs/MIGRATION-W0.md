# MIGRATION-W0 — Governance, Branch Consolidation, Perception Review

**Date:** 2026-05-04
**Wave:** W0
**Risk:** Zero runtime change

## What changed

1. **AGENTS.md** — Replaced 273-line detailed prompt with 65-line governance constitution.
   The detailed rules are preserved in `docs/plans/SUMMARY.md` and user memory.

2. **Branch topology:**
   - Renamed `chore/workspace-deep-clean-2026-04-26` → `ws-deep-rebuild-3526`
   - Tagged `rebuild-again-v2` and `rebuild-core-v2` as `backup/*` before local deletion
   - Surviving branches: `main`, `super-fix`, `hmi-pro`, `ws-deep-rebuild-3526`

3. **Governance files added:**
   - `.pre-commit-config.yaml` — ruff, mypy, clang-format, yamllint, detect-secrets, jscpd, custom hooks
   - `.github/workflows/ci.yml` — 11 CI jobs (2 stubs for W1)
   - `.github/PULL_REQUEST_TEMPLATE.md` — Mandatory search/reuse/deprecation/HMI fields
   - `tools/lint/no_magic_motion_numbers.sh` — SSOT enforcement
   - `tools/lint/file_size_budget.sh` — Python 500 LOC / C++ 700 LOC limits
   - `tools/lint/file_size_exceptions.txt` — 12 current offenders with target waves

4. **Audit documents:**
   - `docs/audit/NAMED_STATE_AUDIT.md` — 5 poses audited, park_safe=delete, home=escalate
   - `docs/hmi/HMI_ROS_INTERFACES.md` — 11 ROS surfaces inventoried with change sensitivity

5. **Perception review:**
   - `docs/perception/REVIEW_OF_36520035.md` — Analytical review of commit 36520035 for W4 input

## What did NOT change

- No `.py` or `.cpp` source files under `src/` were modified
- No ROS packages added or removed
- No runtime behavior changed
- No hardware-facing code touched

## How to roll back

```bash
# Restore original branch name
git branch -m ws-deep-rebuild-3526 chore/workspace-deep-clean-2026-04-26

# Restore deleted branches from backup tags
git branch rebuild-again-v2 backup/rebuild-again-v2-20260504
git branch rebuild-core-v2  backup/rebuild-core-v2-20260504

# Revert the W0 commit(s)
git revert -m 1 <W0 merge commit hash>

# Drop CI workflow if blocking
git rm .github/workflows/ci.yml && git commit -m "rollback: drop W0 CI"
```
