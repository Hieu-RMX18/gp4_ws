# W0 — Governance, Branch Consolidation, Perception Review

**Wave class:** Foundation
**Risk:** Zero runtime change
**Estimated effort:** 3–4 working days
**Depends on:** Nothing (this is the first wave)
**Unblocks:** All subsequent waves

---

## Goal

Install the five-mechanism anti-bloat enforcement, finalize the branch topology so the rebuild has a single working branch, and produce a written review of the existing perception code at commit `36520035` so W4 has reference material without copy-paste pressure.

This wave does NOT change any runtime behaviour. It does NOT touch `safety/`, `motion_core/`, `llm_gateway/`, `primitives/`, `hw_adapter/`, or `supervisor/`. Its outputs are: governance files, branch topology, perception review document, and CI/pre-commit configs.

If the agent finds itself editing `.py` or `.cpp` files inside `src/` for any reason other than adding `# DEPRECATED:` tags or fixing pre-commit-introduced formatting, stop. That work belongs in a later wave.

---

## Discovery (run before writing anything)

The agent must execute and paste the raw output of every command below. If shell access is unavailable, stop and ask the human to run them.

```bash
# A. Confirm current branch and working tree state
git status
git branch -a
git log --oneline -10

# B. Re-verify the package map (must match SUMMARY)
find src/ -maxdepth 2 -name "package.xml" -exec dirname {} \;
ls -la hmi/

# C. Confirm bloat evidence
rg -n "_hydrate_draw_workplane|hydrate_draw_workplane" --type py
rg -n -e "kDefaultVelocityScaling" --type cpp
rg -n -e "computeCartesianPath" --type cpp

# D. Find existing pre-commit / CI / lint configs (do NOT overwrite without listing first)
ls -la .pre-commit-config.yaml .gitlab-ci.yml .github/ pyproject.toml setup.cfg .ruff.toml .mypy.ini 2>&1

# E. Vision package commit — read-only review
git log --all --oneline -- "src/gp4_perception/" | head -20
git show --stat 36520035

# F. Tools available locally
which ruff black mypy clang-format clang-tidy jscpd vulture yamllint detect-secrets pre-commit
```

If any command in B contradicts the SUMMARY's verified-facts table, stop and report. The wave plan assumes those facts hold.

---

## Tasks

### W0.T1 — Branch consolidation

Per decision D2.

```bash
# 1. Tag every branch we are about to abandon, so we can recover
git tag backup/rebuild-again-v2-20260503 rebuild-again-v2
git tag backup/rebuild-core-v2-20260503  rebuild-core-v2
# Any other doomed branch gets a tag in the form backup/<name>-<YYYYMMDD>

# 2. Push tags to origin so the backup is durable
git push origin --tags

# 3. Rename the current working branch
git branch -m chore/workspace-deep-clean-2026-04-26 ws-deep-rebuild-3526
git push origin -u ws-deep-rebuild-3526
git push origin --delete chore/workspace-deep-clean-2026-04-26

# 4. Delete the doomed branches LOCAL ONLY
git branch -D rebuild-again-v2
git branch -D rebuild-core-v2

# 5. Delete the corresponding remotes ONLY AFTER the human confirms
#    Do NOT execute these without explicit approval:
# git push origin --delete rebuild-again-v2
# git push origin --delete fixing
# git push origin --delete feature/gp4-agentic-stack
```

The agent must NOT delete remotes without printing the list and asking for approval. Step 5 is a separate confirmation gate.

The branches that survive: `main`, `super-fix`, `hmi-pro`, `ws-deep-rebuild-3526`. That is four branches. Anything else either gets a `backup/` tag and is deleted, or is justified in writing.

### W0.T2 — Install `AGENTS.md` at the repo root

Path: `AGENTS.md` (repository root, next to `src/` and `hmi/`).

Content (60–80 lines, no more):

```markdown
# AGENTS.md — gp4_ws

## Discover before you write
1. Run `find src/ -name "*<keyword>*"` and `rg -e "<symbol>"` BEFORE creating
   anything. Paste output in your reply. If shell unavailable, STOP and ask.
2. Reuse code in the same domain. New file when an equivalent exists = REJECTED.

## Write surgically
3. Diff-only output. Max 200 LOC changed per file per change.
4. Match existing style. Do NOT "improve" adjacent code.
5. All numeric constants for safety / motion / perception → YAML SSOT.
   Zero magic numbers in .py / .cpp source.

## Remove safely
6. Tag `# DEPRECATED: removal_date=<today+28d>, reason=<why>`.
   Hard delete only in cleanup waves AFTER `rg <symbol>` returns 0.

## Hard never
- Never publish directly to /joint_trajectory_controller/*
- Never bypass the safety chain "for testing"
- Never fabricate ros2 / rg / find output. If no shell, STOP and ask.
- Never combine refactor + behavior change in one commit
- Never silently fallback to computeCartesianPath in motion code (banned W1)
- Never reimplement ROS-side logic in hmi/backend/ — call the ROS service
- Never change a HIGH-sensitivity ROS surface (per docs/hmi/HMI_ROS_INTERFACES.md) without paired hmi/ patch in the same PR
- Never use `park_safe` as a named state (deleted per U1, W1.T0). Use only states listed in docs/audit/NAMED_STATE_AUDIT.md status=`active`.
- Never silently downsample a trajectory before QualityGate. Multi-stage placement is mandatory (W1.T4).

## Branch hygiene
- Working branch is ws-deep-rebuild-3526. State branch at start of every reply.
- Do not checkout a new branch to escape complexity. Refactor in place.
- Surviving branches: main, super-fix, hmi-pro, ws-deep-rebuild-3526.

## Python deps for ROS 2 Humble
- requirements.txt with exact pins. pip install --user. Never sudo pip.
- After install: python3 -c 'import rclpy; print("OK")' must pass.
- Do not activate venv inside a sourced ROS workspace.

## Response contract
Every reply contains, in order:
  1. Current branch and wave name
  2. Discovery output pasted (commands echoed as `$ <cmd>`)
  3. Plan: files changed, files NOT changed, rationale
  4. Diffs (unified diff format)
  5. Verification commands actually run
  6. Risks / safety notes
  7. [VERIFIED] / [NEEDS-VALIDATION] / [KNOWN-GAP] tag
  8. End-of-wave stop signal
```

If a top-level `AGENTS.md` already exists with different content, the agent must report the conflict and ask for resolution. Do not silently overwrite.

### W0.T3 — Install `.pre-commit-config.yaml`

Path: `.pre-commit-config.yaml` (repo root).

Required hooks:

```yaml
repos:
  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: v0.6.9
    hooks:
      - id: ruff
        args: [--fix]
      - id: ruff-format
  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: v1.11.2
    hooks:
      - id: mypy
        args: [--strict, --ignore-missing-imports]
        files: ^(src/llm_gateway|src/safety|hmi/backend)/.*\.py$
  - repo: https://github.com/pre-commit/mirrors-clang-format
    rev: v18.1.8
    hooks:
      - id: clang-format
        files: \.(cpp|hpp|h|c)$
  - repo: https://github.com/adrienverge/yamllint
    rev: v1.35.1
    hooks:
      - id: yamllint
  - repo: https://github.com/Yelp/detect-secrets
    rev: v1.5.0
    hooks:
      - id: detect-secrets
  - repo: local
    hooks:
      - id: jscpd
        name: jscpd duplicate detection
        entry: bash -c 'jscpd src/ hmi/backend/ --min-lines 30 --min-tokens 70 --threshold 0 --silent'
        language: system
        pass_filenames: false
      - id: no-magic-motion-numbers
        name: No hardcoded velocity / accel / joint limits
        entry: tools/lint/no_magic_motion_numbers.sh
        language: script
        pass_filenames: false
      - id: deprecated-must-have-removal-date
        name: Every DEPRECATED tag has removal_date
        entry: bash -c '! rg -n "DEPRECATED" --type py --type cpp src/ hmi/backend/ | rg -v "removal_date"'
        language: system
        pass_filenames: false
      - id: file-size-budget
        name: File size budget
        entry: tools/lint/file_size_budget.sh
        language: script
        pass_filenames: false
```

Create the local hook scripts:

`tools/lint/no_magic_motion_numbers.sh` — fail if a velocity/accel/joint-limit literal appears in production source outside a YAML/config/test path. Implementation note: use `rg` with whitelist exclusions (matches the improved R2 self-check). Exit 1 with the offending file:line on hit.

`tools/lint/file_size_budget.sh` — Python files in `src/` over 500 LOC, C++ files over 700 LOC: fail unless listed in a `tools/lint/file_size_exceptions.txt` file with a justification line. Today's known offenders go into the exceptions list with their current line counts and a `# TODO(W6): split` note.

### W0.T4 — Install CI workflow

Path: `.github/workflows/ci.yml` (assuming GitHub; if the project uses GitLab, agent must propose `.gitlab-ci.yml` instead and ask for confirmation).

Required jobs (each a separate job, all required for merge):

1. `lint` — pre-commit run --all-files
2. `typecheck-python` — mypy --strict on `src/llm_gateway`, `src/safety`, `hmi/backend`
3. `typecheck-cpp` — clang-tidy on `src/motion_core`, `src/primitives`, `src/hw_adapter`, `src/supervisor`
4. `build` — `colcon build --symlink-install` against ROS 2 Humble
5. `test-ros` — `colcon test && colcon test-result --verbose`
6. `test-hmi-backend` — `pytest hmi/backend/`
7. `test-hmi-frontend` — `npm test --prefix hmi/frontend/`
8. `duplication` — jscpd, fail on any block ≥30 LOC
9. `dead-code` — vulture, fail if dead code count increases vs main branch baseline
10. `safety-chain` — `python tools/validate_safety_chain.py` (the script does not exist yet; W1 creates it. Until then, this job is allowed to be a stub that exits 0 with a `TODO(W1)` log line)
11. `no-fallback-guard` — `! rg -n 'computeCartesianPath' src/motion_core/src/primitive_router_dispatch.cpp` (W1 introduces this guard properly; until then, this job is also a `TODO(W1)` stub)

The two stub jobs must remain visible in CI status from W0 onward, even as no-ops, so that W1 only has to fill in the implementation, not invent the contract.

### W0.T5 — Install PR template

Path: `.github/PULL_REQUEST_TEMPLATE.md` (or repository equivalent).

```markdown
## Wave / Task
Wave: <W0|W1|...>  Section: <T1|T2|...>

## What I searched (paste rg / find output)
```

```
$ rg -e "..." --type py
<paste raw output>
```

```markdown
## What I reused (file:line of existing code I called)
- ...

## What I deprecated
- file:line — DEPRECATED tag added with removal_date=YYYY-MM-DD
- (or "none")

## File size budget impact (LOC delta per file changed)
- src/foo/bar.py: +X / -Y (within budget / requires exception)

## HMI compatibility
- ROS service / topic / action schema changes: <yes / no>
- HMI backend touched: <yes / no — paths>
- HMI frontend touched: <yes / no — paths>

## Branch hygiene
- Source branch: <name>
- Days since branched from main: <N>
- Plan to merge: <date>

## Risks / safety notes
- <one-line risk per row>

## Verification
- Pre-commit: <pass / fail>
- CI: <green / yellow>
- Sim test: <run / not run / N/A>
- Hardware test: <run / not run / N/A>
```

PRs that submit this template empty are rejected.

### W0.T6 — Perception review document (read-only)

Per decision D1, W4 will rebuild perception fresh, but it should not start from a blank page. The agent reads commit `36520035` and writes a review document at `docs/perception/REVIEW_OF_36520035.md`.

```bash
# Read the perception code from the commit without checking it out
git show 36520035 --stat -- "src/gp4_perception/"
git show 36520035:src/gp4_perception/CMakeLists.txt
git show 36520035:src/gp4_perception/package.xml
git show 36520035:src/gp4_perception/README.md
git show 36520035:src/gp4_perception/config/d435i.yaml
git show 36520035:src/gp4_perception/config/camera_extrinsics.yaml
git show 36520035:src/gp4_perception/config/fiducials.yaml
git show 36520035:src/gp4_perception/config/vision_pick_planner.yaml
git show 36520035:src/gp4_perception/gp4_perception/calibration_recorder_node.py
git show 36520035:src/gp4_perception/gp4_perception/fiducial_detector_node.py
git show 36520035:src/gp4_perception/gp4_perception/realsense_health_node.py
git show 36520035:src/gp4_perception/gp4_perception/vision_pick_planner.py
git show 36520035:src/gp4_perception/launch/realsense_d435i.launch.py
git show 36520035:src/gp4_perception/launch/calibration_collection.launch.py
```

The review document must answer, for each file:

- What does this file do?
- Does it depend on packages we still have, or on packages that have been removed/renamed?
- What is its calibration approach? Frame conventions? Hand-eye solver?
- What ROS topics, services, actions does it publish/subscribe?
- What QoS does it use? Does it match RealSense publishers (`SensorDataQoS`)?
- Is `calibration_date` filled at runtime or hardcoded?
- Does it have a depth-noise / freshness / reprojection guard?
- Are there obvious bugs that would explain "not-finished-rebuild"?
- For W4, what is keep, what is discard, what needs redesign?

The review document produces a numbered list of decisions for W4. W4 reads this document and implements fresh code informed by it.

### W0.T7 — File-size exception list

Path: `tools/lint/file_size_exceptions.txt`. Pre-populate with current known offenders so CI is green from day one. Each row needs a justification:

```
# Format: <path> <current_loc> <reason> <target_wave_for_split>
src/llm_gateway/llm_gateway/draw_router.py 889 emits CARTESIAN_PATH; will be split when rewired in W2 W2
src/llm_gateway/llm_gateway/llm_gateway_node.py 869 god-node; will be split when ReAct lands W3
src/llm_gateway/llm_gateway/drawing_geometry.py 729 will be split alongside draw_router rewire W2
src/motion_core/src/primitive_router_dispatch.cpp 939 contains the silent fallback being removed in W1 W1
src/jog_pendant/src/servo_bridge_node.cpp 842 separate domain; revisit in W6 W6
src/primitives/src/primitive_blended_sequence.cpp 827 dispatcher complexity; revisit after W2 wires it W6
src/motion_core/src/motion_core_node.cpp 784 god-node; revisit in W6 W6
src/hw_adapter/src/hw_adapter_node.cpp 666 boundary file; revisit in W6 W6
src/hw_adapter/test/test_hw_adapter_node.cpp 650 mirrors hw_adapter_node W6
src/hw_adapter/src/trajectory_executor.cpp 624 revisit W6
src/supervisor/src/execution_monitor.cpp 594 revisit W6
src/motion_core/src/dispatch_trajectory_executor.cpp 590 revisit W6
```

Every wave that splits a file removes the entry. By W6, the list should be empty or each entry has a hard justification.


##Patch 1.1 — ADD task W0.T8 Named-state audit (J5 ±90° prep)
Insert immediately after current W0.T7 in W0_governance_branch_perception_review.md.
markdown### W0.T8 — Named-state audit (precondition for W1's J5 ±90°)

**Why:** W1 enforces operational_joint_limits including `joint_5_b: {min: -1.571, max: 1.571}` (±90°, derated from hardware ±123°). If any named pose, fixture, or HMI quick-action references J5 outside ±90°, W1 will silently start rejecting that pose — including `park_safe` if it stows the arm with J5 < -1.571 rad. We discover those references now so W1 deletes or relocates them as a single coordinated change.

**Mandatory grep matrix.** Run each command and paste output verbatim into `docs/audit/NAMED_STATE_AUDIT.md`:

```bash
# A. SRDF / MoveIt named states
rg -n -e 'group_state name=' src/gp4_moveit_config/
rg -n -e 'park_safe|park-safe|park_pose|park_position' src/ hmi/ docs/ 2>/dev/null

# B. Python / C++ symbol references
rg -n -e 'park_safe|park-safe|PARK_SAFE' --type py --type cpp --type yaml

# C. HMI quick actions / fixtures
rg -n -e 'park_safe|park-safe|"park"' hmi/

# D. Other named poses that may collide with J5 ±90°
rg -n -e 'home_pose|home_position|named_pose|stored_pose|stowed' src/ hmi/

# E. URDF/SRDF joint values for inspection
rg -n -e 'joint_5_b' src/gp4_moveit_config/
```

**Output deliverable:** `docs/audit/NAMED_STATE_AUDIT.md` with one table:

| Pose name | Defined in | J1 | J2 | J3 | J4 | J5 | J6 | Conflicts with W1 limits? | Owner |
|---|---|---|---|---|---|---|---|---|---|

For each conflict (per U1: J5 outside ±1.571 rad, J4 outside ±2.443 rad, J6 outside ±3.142 rad), the row's "Owner" column states one of:
- `delete` — the pose is unused, deprecate it now (use the standard tag from W0)
- `relocate` — the pose is needed, propose new joint values inside operational limits
- `escalate` — operator must decide; do NOT auto-pick

`park_safe` specifically: per U1, the user has chosen `delete`. The audit row for `park_safe` is filled with `delete` regardless of where it is found, and W1.T0 (Patch 2.0) executes the deletion.

**Stop signal:** W0 PR includes `NAMED_STATE_AUDIT.md`. Human reviews each row before merging W0.
Patch 1.2 — ADD task W0.T9 HMI ROS interface inventory
Insert immediately after Patch 1.1's W0.T8 in W0_governance_branch_perception_review.md.
markdown### W0.T9 — HMI ROS interface inventory

**Why:** HMI under `hmi/backend/` and `hmi/frontend/` consumes ROS topics, services, and actions. Verified consumers from earlier discovery include `/validate_command`, `/execute_motion`, `/llm_text_input`, `/llm_command`, `/llm_debug`, `/gateway_status`, `/get_current_pose`. Any later wave that changes a ROS surface (especially W2 schema additions, W3 new ReAct route, W4 perception services, W5 aggressive consolidation) breaks HMI silently if we do not have an inventory locked in.

**Discovery commands** (paste raw output):

```bash
rg -n -e 'rclpy|rosbridge|websocket|rosjs|ROSLIB' hmi/ 2>/dev/null
rg -n -e '/llm_intent|/llm_raw_command|/llm_text_input|/llm_command|/llm_debug' hmi/
rg -n -e '/validate_command|/execute_motion|/get_current_pose|/gateway_status' hmi/
rg -n -e 'topic_name|service_name|action_name' hmi/backend/
rg -n -e 'subscribe|publish|create_client|create_action_client' hmi/backend/ hmi/frontend/
cat hmi/HMI_V2_COMMAND_INGRESS.md 2>/dev/null | head -80
```

**Output deliverable:** `docs/hmi/HMI_ROS_INTERFACES.md` with one table:

| ROS surface | Type | Used by (file:line) | Direction | Pinned message version | Change sensitivity |
|---|---|---|---|---|---|

`Change sensitivity` is one of `LOW` (HMI tolerates additions), `MEDIUM` (HMI must update on schema change), `HIGH` (HMI breaks on any rename or field rename).

**Re-verification rule:** every wave from W2 onward that touches a ROS surface must:
1. Re-run the inventory grep matrix and diff against this file.
2. Update the `Change sensitivity` column.
3. List the breaking changes (if any) in that wave's MIGRATION-W<N>.md.

**Stop signal:** W0 PR includes `HMI_ROS_INTERFACES.md`. The "Re-verification rule" is added to W0.T2's AGENTS.md as a hard rule under "Hard never": `Never change a HIGH-sensitivity ROS surface without updating HMI_ROS_INTERFACES.md and a paired hmi/ patch.`
---

## Verification

The agent must run each of the following and paste output, and the human must inspect.

| # | Check | Pass criterion |
|---|---|---|
| 1 | `git branch -a` | Local: `ws-deep-rebuild-3526` (current), `main`, `super-fix`, `hmi-pro`. Backup tags exist for deleted branches. |
| 2 | `cat AGENTS.md \| wc -l` | Between 60 and 90 lines. Contains "Hard never", "Branch hygiene", and "Response contract" sections. |
| 3 | `pre-commit run --all-files` | Exit 0 (or fails only on existing offenders that the file-size exception list legitimizes; agent reports each failure with classification "expected" or "unexpected") |
| 4 | `cat .github/workflows/ci.yml \| grep -c '^  [a-z-]*:$'` | At least 11 jobs |
| 5 | CI run on the W0 PR | Green; the two stub jobs print `TODO(W1)` and exit 0. |
| 6 | `cat docs/perception/REVIEW_OF_36520035.md \| wc -l` | At least 200 lines, structured per W0.T6 spec, with explicit keep/discard/redesign list. |
| 7 | `cat tools/lint/file_size_exceptions.txt` | All current offenders listed with target wave. |
| 8 | `colcon build --symlink-install` | Still green (no source changed) |
| 9 | `colcon test` | Same pass count as before W0 (no source changed) |
| 10| `git log --oneline ws-deep-rebuild-3526..main` | Empty (we are ahead of main, not behind) |
| 11|     cat docs/audit/NAMED_STATE_AUDIT.md |All conflicts have delete / relocate / escalate filled. park_safe row exists with delete.|
| 12|      cat docs/hmi/HMI_ROS_INTERFACES.md |All current HMI ROS consumers listed. Change sensitivity filled per row.|
| 13|      AGENTS.md content |Contains the three new "Hard never" bullets from Patch 1.3|
---

## DON'T

- Do not edit any file under `src/<package>/src/`, `src/<package>/include/`, or `src/<package>/<package>/` (Python source). The whole point of W0 is to be a no-op runtime-wise.
- Do not delete remote branches in step W0.T1.5 without explicit human approval each time.
- Do not write the perception review by paraphrasing file contents. Read each file, then state in your own words what it does. The review's value is the agent's analytical reading, not transcription.
- Do not enable the `safety-chain` or `no-fallback-guard` CI jobs as real tests in this wave. They are stubs by design — W1 fills them.
- Do not configure `mypy --strict` on packages that are known to fail today (e.g. `motion_core` C++ surface). Restrict mypy to the listed Python packages.
- Do not "fix" any pre-commit failure that is unrelated to W0 outputs. If `ruff` finds 200 issues in `draw_router.py`, that is W2's problem, not W0's. Add the file to a `# noqa` allow-list temporarily, with `# W2 cleanup target` comment.
- Do not add any new ROS package, Python module, or C++ class.

---

## Output artefacts

W0 PR contains exactly these files:

1. `AGENTS.md` (root)
2. `.pre-commit-config.yaml` (root)
3. `.github/workflows/ci.yml`
4. `.github/PULL_REQUEST_TEMPLATE.md`
5. `tools/lint/no_magic_motion_numbers.sh`
6. `tools/lint/file_size_budget.sh`
7. `tools/lint/file_size_exceptions.txt`
8. `docs/perception/REVIEW_OF_36520035.md`
9. `docs/waves/SUMMARY.md` (this plan, copied in)
10. `docs/waves/W0_governance_branch_perception_review.md` (this file, copied in)
11. `MIGRATION-W0.md` summarising what changed and how to roll back
12. `docs/audit/NAMED_STATE_AUDIT.md`
13. `docs/hmi/HMI_ROS_INTERFACES.md`

Plus branch operations as printed git commands.

---

## Rollback procedure

If anything goes wrong:

```bash
# Restore original branch name
git branch -m ws-deep-rebuild-3526 chore/workspace-deep-clean-2026-04-26

# Restore deleted branches from backup tags
git branch rebuild-again-v2 backup/rebuild-again-v2-20260503
git branch rebuild-core-v2  backup/rebuild-core-v2-20260503

# Revert the W0 PR
git revert -m 1 <W0 merge commit hash>

# Drop the CI workflow if it's blocking work
git rm .github/workflows/ci.yml
git commit -m "rollback: drop W0 CI workflow"
```

If pre-commit becomes a productivity blocker, the human can disable individual hooks by commenting them in `.pre-commit-config.yaml`. The CI version remains authoritative until the human chooses to weaken it.

---

## Stop signal

End of W0. Do not proceed to W1 until:

- W0 PR is reviewed and merged.
- The branch topology decision (T1.5 remote deletions) is executed.
- The perception review document is read by the human and approved as W4 input.

State explicitly: `End of W0. Awaiting review before W1.`

---

**Reliability tag:** `[VERIFIED]` — every artefact in this wave is a configuration file or a documentation document, no runtime change. The discovery commands are syntactically correct (lessons from earlier rounds applied — no `\|` escapes). `[NEEDS-VALIDATION]` only if the local toolchain is missing one of `ruff`, `clang-format`, `jscpd`, etc.; in that case the agent reports and the human installs.
