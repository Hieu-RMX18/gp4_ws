# W6 — First Cleanup Wave: Aged Deprecations, Duplication Audit, File Budget

**Wave class:** Maintenance / housekeeping
**Risk:** Low
**Estimated effort:** 2–3 working days
**Depends on:** W0 through W5 merged AND a 28-day cooldown for symbols deprecated in earlier waves
**Unblocks:** Routine: this wave repeats every 2 weeks per the 5-mechanism enforcement

---

## Goal

W6 is the first run of what becomes a recurring biweekly cleanup wave. Its purpose is to convert deprecation tags into actual deletions and to enforce file size, duplication, and dead-code budgets that have accumulated debt during the rebuild.

W6 does NOT introduce new behaviour. Every change either deletes code, splits a file, or refines a CI check. Runtime is unchanged.

If the agent is tempted to "improve" anything beyond cleanup scope, the response is: write it down for a future wave, do not include in W6.

---

## Discovery (paste raw output)

```bash
# A. Aged DEPRECATED tags
rg -n "DEPRECATED.*removal_date" --type py --type cpp src/ hmi/backend/

# B. For each result above, parse the date and check if it is past
python3 << 'EOF'
import re, subprocess, sys
from datetime import datetime

today = datetime.utcnow().date()
out = subprocess.check_output(
    ["rg", "-n", "DEPRECATED.*removal_date", "--type", "py", "--type", "cpp",
     "src/", "hmi/backend/"], text=True)
past = []
future = []
for line in out.splitlines():
    m = re.search(r"removal_date=(\d{4}-\d{2}-\d{2})", line)
    if not m:
        continue
    rd = datetime.strptime(m.group(1), "%Y-%m-%d").date()
    if rd < today:
        past.append((line.split(":")[0:2], rd))
    else:
        future.append((line.split(":")[0:2], rd))
print("PAST (deletion candidates):", len(past))
for p in past:
    print(" ", p)
print("FUTURE (still in cooldown):", len(future))
EOF

# C. For each PAST candidate, confirm zero references
# Per AGENTS.md R5, deletion requires rg <symbol> = 0 hits OUTSIDE the deprecated definition itself.
# Agent walks the PAST list, runs `rg <symbol>` for each, includes proof in the PR.

# D. Duplication audit — full
jscpd src/ hmi/backend/ --min-lines 20 --min-tokens 60 --reporters consoleFull

# E. Dead-code audit
vulture src/ hmi/backend/ --min-confidence 90

# F. File-size budget violators
bash tools/lint/file_size_budget.sh

# G. Branch audit
git branch -a
git for-each-ref --sort=-committerdate refs/heads/ --format='%(committerdate:short) %(refname:short)' | head -10

# H. Tools / hooks / SSOT health
pre-commit run --all-files 2>&1 | tail -30
python tools/validate_safety_chain.py
bash tools/lint/no_silent_motion_fallback.sh
```

---

## Tasks
**Special targets for first cleanup wave (post W0.T8):**

The W0.T8 named-state audit may have produced `delete` rows beyond `park_safe`. W1.T0 removed those. W6 verifies completion:
- `rg -e 'park_safe|PARK_SAFE' src/ hmi/` returns 0 productive hits.
- For each row in `NAMED_STATE_AUDIT.md` with status=`relocate`, the SRDF / config now contains the relocated values.
- For each row with status=`escalate`, either the human has resolved it (audit doc updated to delete or relocate) or the row is open and the open count is reported in `MIGRATION-W6.md`.
### W6.T1 — Hard-delete aged deprecations

For every entry in the PAST list from discovery B, the agent:

1. Runs `rg <symbol>` in repo (excluding the file holding the DEPRECATED definition itself).
2. If 0 hits: delete the symbol. ONE DELETION = ONE COMMIT for clean revert.
3. If hits exist: investigate. The DEPRECATED tag was supposed to coincide with all callers being updated. If callers remain, either:
   - The earlier wave forgot to update some callers — file a follow-up commit in W6 to update them, then delete.
   - The deprecation was incorrect and the symbol is still needed — remove the DEPRECATED tag with a justification comment.

The PR shows the `rg <symbol>` proof for each deletion.

Examples expected from W2 and W5 deprecations:

- `hmi/backend/services/intent_resolution.py:411 _hydrate_draw_workplane` (W2 tagged, W5 deleted live in T6 of W5; W6 verifies the file is gone)
- Legacy CARTESIAN_PATH-emit code path in `drawing_geometry.py` (W2 tagged behind `drawing.use_blended_sequence` flag; W6 deletes the dead branch and the flag)
- Legacy `/llm_intent` single-shot handler in `llm_gateway_node.py` (W3 tagged; W6 deletes if 0 references)

If a deprecation in W4 perception was made (e.g. a `<NOT_CALIBRATED>` placeholder branch), W6 audits and deletes if dead.

### W6.T2 — Duplication audit

`jscpd` output from discovery D enumerates duplicate blocks ≥20 LOC.

For each block:

1. The agent classifies it: **(a) consolidate now**, **(b) consolidate later (file as a TODO, justify why now is wrong)**, **(c) accept (justified by domain — e.g. test fixtures)**.
2. Class (a) blocks: extract to a shared module. Same wave.
3. Class (b) and (c): documented in `MIGRATION-W6.md` with rationale.

Common targets to expect (anti-bloat priorities):

- `_wrap_to_pi`, `_rpy_to_quaternion`, `_pose_to_matrix` — math utilities reimplemented per package. Consolidate into a small `gp4_common` package OR into `src/interfaces/utils/`. Decide path with the human's input; do not pick silently.
- Frame conversion helpers across `safety/`, `motion_core/`, `llm_gateway/`.

If creating a new package is needed, that is a sub-decision. The agent presents the choice; the human picks before W6 proceeds.

CI threshold tightening: after W6, jscpd runs with `--min-lines 30` (was 30 already) AND `--threshold 0` (no duplication permitted), with explicit allow-list entries for accepted (c) cases.

### W6.T3 — Dead-code purge

`vulture` output from discovery E.

For each high-confidence dead-code report:

1. Verify it is actually dead (vulture has false positives for ROS callbacks, decorators, etc.). Use `rg <symbol>` to corroborate.
2. If genuinely dead and predates the rebuild waves (i.e. not introduced by W0–W5), tag with `# DEPRECATED: removal_date=<today+28d>`. Will be deleted in the next cleanup wave.
3. If introduced by W0–W5 by mistake, delete now (it's our own mess per Karpathy's clean-up-only-your-own-mess principle).

### W6.T4 — File-size budget enforcement

`tools/lint/file_size_budget.sh` from W0 currently passes because of the exception list. Each entry in `tools/lint/file_size_exceptions.txt` has a target wave. W6 audits:

- Has the target wave landed?
- Is the file still over budget?
- If yes to both: split now.

Expected splits for files originating in earlier waves:

| File | LOC | Originating wave | W6 action |
|---|---|---|---|
| `src/llm_gateway/llm_gateway/draw_router.py` | 889 | W2's target | Split: `draw_router.py`, `draw_circle_compiler.py`, `draw_text_compiler.py`, `draw_polyline_compiler.py`. Each ≤ 350 LOC. |
| `src/llm_gateway/llm_gateway/llm_gateway_node.py` | 869 | W3's target | Split: `llm_gateway_node.py` (entry), `intent_route_handler.py` (with ReAct dispatch), `raw_command_route_handler.py` (legacy fallback). |
| `src/llm_gateway/llm_gateway/drawing_geometry.py` | 729 | W2's target | Split: `drawing_geometry.py` (geometry math), `drawing_command_emitter.py` (BLENDED_SEQUENCE construction). |
| `src/motion_core/src/primitive_router_dispatch.cpp` | 939 | W1's target (silent fallback removed) | Now smaller after W1; revisit. If still ≥ 700 LOC, split: dispatch logic, planner-specific routing, error handling. |
| `src/jog_pendant/src/servo_bridge_node.cpp` | 842 | W6's own | Split if needed; this is jog logic, separable into command-receiving vs streaming. |

Splits MUST preserve test coverage. Run `colcon test` before and after each split; line counts may change but pass count should not.

For each split: ONE PR or ONE COMMIT per file. Avoid coupling unrelated splits.

### W6.T5 — Branch hygiene

After W0, four branches survive: `main`, `super-fix`, `hmi-pro`, `ws-deep-rebuild-3526`.

W6 audits:

- `super-fix`: how stale? If no commits in 30 days and contains nothing that hasn't been ported, delete (with backup tag).
- `hmi-pro`: same.
- `ws-deep-rebuild-3526`: ready to merge to main? If yes, plan the merge in a separate PR.

The agent does not delete branches without explicit human approval. Reports the audit, recommends.

### W6.T6 — CI check tightening

Now that W1–W5 have stabilized, the CI checks marked as stubs or "TODO" can be tightened:

- `safety-chain` job: was real from W1. W6 verifies it still passes and includes the perception extrinsics check from W4.
- `no-fallback-guard`: real from W1. W6 verifies.
- `duplication`: was lenient (`--min-lines 30 --threshold 0`). W6 confirms still passes after T2 consolidation.
- `dead-code`: vulture baseline updated to post-W6 numbers; future PRs cannot increase.
- `file-size-budget`: exception list shrunk per T4.
- New CI job `aged-deprecation`: runs the discovery B Python script, fails if any deprecation past `removal_date` is still in the tree. Forces cleanup waves to actually clean up.

### W6.T7 — Documentation refresh

- Update `AGENTS.md` if any section has become stale (likely "Hard never" rules — check whether new patterns to forbid have emerged from W1–W5 retrospective).
- Update `SUMMARY.md` (the rebuild plan v3 summary) with completion status of each wave.
- Update `docs/operation/` files if any procedure changed because of W4 perception or W5 HMI rerouting.

### W6.T8 — Schedule the next cleanup wave

In the PR description, propose the next cleanup wave date (today + 14 days). Add to a recurring calendar or issue tracker. The five-mechanism design depends on cleanup waves running, not on intentions.

---

## Verification

| # | Check | Pass criterion |
|---|---|---|
| 1 | `rg "DEPRECATED" --type py --type cpp src/ hmi/backend/` | All remaining tags have future `removal_date` |
| 2 | `python tools/aged_deprecation_check.py` (from T6) | Exit 0 |
| 3 | `jscpd src/ hmi/backend/ --min-lines 30 --threshold 0` | Exit 0 |
| 4 | `vulture src/ hmi/backend/ --min-confidence 90` | Output count ≤ baseline before W6 |
| 5 | `bash tools/lint/file_size_budget.sh` | Exit 0; exception list shrunk |
| 6 | `colcon build --symlink-install` | Green; same package count |
| 7 | `colcon test` | Same pass count as before W6 (no behavioural change) |
| 8 | CI all jobs | Green |
| 9 | `git log --oneline ws-deep-rebuild-3526..main` | Empty (we are not behind main; if behind, sync first) |
| 10 | New `aged-deprecation` CI job | Active and required |
| 11 | Test for split files | Each split file's tests still pass |
| 12 | Branch audit | Recommendation in PR; surviving branch list confirmed |

---

## DON'T

- Do not introduce new behaviour. W6 is cleanup only.
- Do not delete a symbol without `rg <symbol>` proof of zero hits.
- Do not bundle file splits with refactors that change semantics. Split is structural; refactor is behavioural; they are separate commits even if in the same PR.
- Do not delete branches without backup tags AND human approval.
- Do not "fix" code style issues outside files you are already touching for cleanup. Each touch must trace to a deletion or split decision.
- Do not skip the next-cleanup-wave scheduling step. It is the linchpin of the recurring cadence.
- Do not increase the duplication threshold to make CI green; make CI green by removing duplicates.
- Do not consolidate utilities into `motion_core` or `llm_gateway` "because they are nearby". Shared utilities belong in a shared package; otherwise import direction creates loops.

---

## Output artefacts

- One commit per deletion (T1)
- One commit per file split (T4)
- `MIGRATION-W6.md`: list of every deletion, every consolidation, every split, with `rg` proof per deletion
- `tools/lint/aged_deprecation_check.py` — new
- `.github/workflows/ci.yml` — diff: new `aged-deprecation` job; tightened `duplication` and `dead-code`
- `tools/lint/file_size_exceptions.txt` — diff: removed entries for files now in budget
- `AGENTS.md`, `SUMMARY.md`, `docs/operation/*` — diffs as needed
- Branch audit report in PR description

---

## Rollback procedure

```bash
# Per-deletion rollback
git revert <single deletion commit>

# Per-split rollback
git revert <single split commit>

# CI tightening rollback (if a new CI job is over-eager)
# Edit .github/workflows/ci.yml: comment out or weaken the over-eager job
# Open a follow-up to fix the underlying issue
```

Cleanup waves are the safest waves to revert because each commit is small and self-contained.

---

## Risk notes

- **Test coverage gaps revealed by deletion**: deleting a symbol that has hidden callers will surface as a build or test failure. The R5 `rg` check catches most; runtime/dynamic callers may not be visible to grep. Mitigation: T1 deletions go through CI.
- **File split breaks imports in downstream packages**: if a public symbol moves between files, downstream packages break. Mitigation: re-export from the original module path (`from .new_file import OldName as OldName  # re-export`) and tag the old path DEPRECATED for the next cleanup wave.
- **Vulture false positives**: ROS callbacks, decorator-registered functions, and dynamically dispatched methods are flagged as dead. T3's verification step (`rg <symbol>`) catches these, but the agent must not blindly trust vulture's output.
- **Dead-code baseline drift**: if the vulture baseline keeps shrinking, future cleanups have less to do; if it grows, the rebuild generated debt that cleanup must address. Either is informative.
- **Branch audit is conservative by design**: the agent recommends, the human approves. Aggressive branch deletion is not a W6 default.

---

## Stop signal

End of W6. The next cleanup wave is scheduled (state the date in the PR). The recurring cadence begins.

State explicitly: `End of W6. Cleanup wave complete. Next cleanup scheduled for <date>.`

---

**Reliability tag:** `[VERIFIED]` for the cleanup procedure — the actions are mechanical (delete after `rg` proof, split after coverage check). `[NEEDS-VALIDATION]` for the specific list of files to split, since W1–W5 may produce or remove files that change the inventory; the agent runs T4 against the actual exception list at the time of W6. `[NEEDS-VALIDATION]` for the consolidation target (new package vs `interfaces/utils/`); human picks at T2.
