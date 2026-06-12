# LLM Gateway R1 — factory_task.py Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the 4331-line `factory_task.py` god-file into 6 focused modules so it keeps only FactoryTask dataclasses + WorldModel + TaskCompiler + PolicyEngine + StationSceneGraph, per spec §3 — without changing any behavior.

**Architecture:** Pure relocation guarded by the existing 442-test suite. `factory_task.py` is already a re-export hub (its `__all__` re-exports symbols from `llm_payload_parser`, `task_runtime`, `drawing_geometry`). We extend that pattern: move a cluster of symbols to a new module, then re-import them back into `factory_task.py` so every existing `from llm_gateway.factory_task import X` caller keeps working. Each module move is one green commit.

**Tech Stack:** Python 3.10, pytest, colcon (ROS 2 Humble), llm_gateway package (egg-link install).

This is phase R1 of the remediation spec `docs/superpowers/specs/2026-06-12-llm-gateway-remediation-design.md`. R2 (closed-loop actuate), R3 (task_events), R4 (thin node) get their own plans after R1 lands.

---

## Key facts (verified 2026-06-12)

- **External callers only use** `from llm_gateway.factory_task import <symbol>` (node, task_planner, task_runtime, supervisor_validation.py, ~10 test files). None reach into internals. So re-export = zero caller edits.
- **`factory_task.py` already re-exports** from other modules — follow that established pattern, do not invent a new one.
- **Relocation has no new tests.** The invariant is: the 442-test suite stays green and the build stays green after every move. There is no red-green-refactor here (that belongs to R2).
- **Circular-import guard:** `factory_task` and `task_runtime` already import each other lazily (function-level / TYPE_CHECKING). Keep new cross-module imports at module top-level only when acyclic; otherwise import inside the function, matching the existing style.
- **Watch for duplicate module globals:** `_LOGGER` is defined 3× and `_FRAME_BASE_LINK` 2× in the current file. When moving a cluster, give each new module its own `_LOGGER = logging.getLogger(__name__)` and its own `_FRAME_BASE_LINK = "base_link"` constant — do not import these across modules.

## Module target layout

| New module | Symbols moved (classes + their module-level helpers/constants) |
|-----------|----------------------------------------------------------------|
| `validation.py` | `SchemaValidator`, `SemanticValidator`, `SequenceValidator`, `SequenceValidationResult`, `SequenceValidationError`, `_default_schema_path`, `_load_schema`, `_load_safety_rules`, `_FAILSAFE_MOTION_LIMITS`, `_QUERY_PRIMITIVES`, `_FRAME_REQUIRED_PRIMITIVES`, `_SUPPORTED_SEQUENCE_FRAMES` |
| `normalization.py` | `Normalizer`, `normalize_pose`, `normalize_joints`, `_to_float`, `_convert_linear`, `_convert_angular`, `_is_likely_mm`, `_is_likely_degrees`, `_wrap_to_pi`, `_normalize_single_joint_angle`, `_rpy_to_quaternion`, `_normalize_orientation`, `_import_geometry_msgs`, `_VALID_LINEAR_UNITS`, `_VALID_ANGULAR_UNITS`, `_COMPAT_UNIT_HEURISTIC` |
| `goal_mapper.py` | `GoalMapper`, `_SchemaValidatorLike`, `prepare_execution_command`, `command_from_sanitized_json` |
| `drawing_router.py` | `DrawRouterMixin`, `hydrate_draw_workplane`, `_build_route_result`, `_ORIENTATION_PRESETS` |
| `intent_router.py` | `IntentRouter`, `RouteResult`, `prepare_semantic_ir_for_routing`, `_joint_alias_key`, `resolve_gp4_joint_index`, `_current_joint_positions_for_delta`, `_default_macro_policy_path`, `_default_named_pose_srdf_path`, `load_srdf_named_poses`, `_NAMED_POSE_ALIASES`, `canonicalize_named_pose`, `load_macro_policy`, `_copy_semantic_ir` |
| `composite_tools.py` | `_CompositeTool`, `EmitSequenceTool`, `RefreshSceneTool`, `PickObjectTool`, `ApproachObjectTool`, `PlaceObjectTool`, `VerifyPostconditionTool`, `VerifyGraspTool`, `ToolResult`, `CandidatePoseRequest`, `CandidatePoseResult`, `VerificationResult`, `PostconditionVerifier`, `mtc_select`, `generate_candidate_poses` |

**`factory_task.py` keeps:** `FactoryTaskError`, `ResolveResult`, `StationSceneGraph`, `SkillCall`, `TaskNode`, `FactoryTask`, `PolicyDecision`, `CompiledTask`, `WorldModel`, `PolicyEngine`, `TaskCompiler`, `compile_goal`, `parse_factory_task`, `is_factory_task`, `count_task_nodes`, `load_station_semantic_map`, `map_contains_verify_config`, `VERIFY_CONFIG`, `_parse_node` + parse helpers, the FACTORY_TASK node-type frozensets, and the full `__all__` re-export block (unchanged).

> Symbol placement is verified empirically: after each move, the full suite must pass. If the suite surfaces a symbol that belongs with a different cluster (e.g. a private helper used only by the moved class), move it too and re-run. The table is the starting assignment, the suite is the arbiter.

---

## Task 0: Baseline green

**Files:** none (verification only)

- [ ] **Step 1: Run the full suite, record the count**

Run:
```bash
cd /home/hieu2/gp4_ws/src/llm_gateway && python -m pytest tests/ -q
```
Expected: all pass (≈442). Record the exact number printed — it must not drop in any later task.

- [ ] **Step 2: Confirm the package builds**

Run:
```bash
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
```
Expected: `Finished <<< llm_gateway`, no errors.

- [ ] **Step 3: Record factory_task.py baseline size**

Run:
```bash
wc -l /home/hieu2/gp4_ws/src/llm_gateway/llm_gateway/factory_task.py
```
Expected: 4331. Target after R1: ≤ ~900 (core only; exact number is informational, not a gate).

---

## Task 1: Extract `validation.py`

**Files:**
- Create: `src/llm_gateway/llm_gateway/validation.py`
- Modify: `src/llm_gateway/llm_gateway/factory_task.py`

- [ ] **Step 1: Run impact analysis before editing**

Run (report blast radius; these classes have many callers — expect HIGH, which is fine for relocation):
```bash
# via GitNexus MCP tools in-session:
#   gitnexus_impact({target: "SemanticValidator", direction: "upstream"})
#   gitnexus_impact({target: "SchemaValidator", direction: "upstream"})
#   gitnexus_impact({target: "SequenceValidator", direction: "upstream"})
```
Expected: callers limited to factory_task internals + tests. If any UNEXPECTED non-test caller appears, stop and note it.

- [ ] **Step 2: Create the new module**

Create `src/llm_gateway/llm_gateway/validation.py` with this header, then paste the moved symbols (cut the class/helper bodies from `factory_task.py`):
```python
"""Schema + semantic + sequence validation for llm_gateway commands.

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import jsonschema
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

_LOGGER = logging.getLogger(__name__)

_QUERY_PRIMITIVES = {"GET_POSE"}
_FRAME_REQUIRED_PRIMITIVES = {"PTP", "LIN", "MOVE_REL", "CARTESIAN_PATH"}
_SUPPORTED_SEQUENCE_FRAMES = {"base_link"}

# ... paste: _FAILSAFE_MOTION_LIMITS, _load_safety_rules, _default_schema_path,
#     _load_schema, SchemaValidator, SemanticValidator, SequenceValidationResult,
#     SequenceValidationError, SequenceValidator (cut verbatim from factory_task.py)
```
Cut every symbol listed for `validation.py` in the layout table out of `factory_task.py`. Keep their bodies byte-for-byte; only the file they live in changes.

- [ ] **Step 3: Re-export from factory_task.py**

In `factory_task.py`, add near the other `from llm_gateway.<module> import ...` lines:
```python
from llm_gateway.validation import (
    SchemaValidator,
    SemanticValidator,
    SequenceValidator,
    SequenceValidationResult,
    SequenceValidationError,
)
```
Leave `__all__` unchanged (it already lists these names). If `TaskCompiler` (kept in factory_task) referenced any moved symbol by bare module-global name, that reference now resolves through this import — verify in the next step.

- [ ] **Step 4: Run the full suite**

Run:
```bash
cd /home/hieu2/gp4_ws/src/llm_gateway && python -m pytest tests/ -q
```
Expected: same count as Task 0, all pass. If an `ImportError` or `NameError` appears, a helper the moved classes use was left behind (or vice versa) — move it to the side that uses it and re-run.

- [ ] **Step 5: Build**

Run:
```bash
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
```
Expected: green.

- [ ] **Step 6: Commit**

```bash
cd /home/hieu2/gp4_ws
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/validation.py src/llm_gateway/llm_gateway/factory_task.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): extract validation.py from factory_task (R1)"
npx gitnexus analyze
```

---

## Task 2: Extract `normalization.py`

**Files:**
- Create: `src/llm_gateway/llm_gateway/normalization.py`
- Modify: `src/llm_gateway/llm_gateway/factory_task.py`

- [ ] **Step 1: Impact analysis**

Run: `gitnexus_impact({target: "Normalizer", direction: "upstream"})` and the same for `normalize_pose`, `normalize_joints`. Expected: factory_task internals + tests + `supervisor_validation.py` (which imports `Normalizer`). The supervisor import is `from llm_gateway.factory_task import ... Normalizer` — re-export keeps it working.

- [ ] **Step 2: Create the module**

Create `src/llm_gateway/llm_gateway/normalization.py`:
```python
"""Unit/pose/joint normalization for llm_gateway commands.

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

import logging
import math
from typing import Any, List, Optional

import numpy as np

_LOGGER = logging.getLogger(__name__)

_VALID_LINEAR_UNITS = {"m", "cm", "mm"}
_VALID_ANGULAR_UNITS = {"rad", "deg"}
_COMPAT_UNIT_HEURISTIC = False

# ... paste: _import_geometry_msgs, _to_float, _convert_linear, _convert_angular,
#     _is_likely_mm, _is_likely_degrees, _wrap_to_pi, _normalize_single_joint_angle,
#     _rpy_to_quaternion, _normalize_orientation, normalize_pose, normalize_joints,
#     Normalizer (cut verbatim from factory_task.py)
```
Cut every symbol listed for `normalization.py` out of `factory_task.py`.

- [ ] **Step 3: Re-export from factory_task.py**

Add to `factory_task.py`:
```python
from llm_gateway.normalization import (
    Normalizer,
    normalize_pose,
    normalize_joints,
)
```
`__all__` unchanged.

- [ ] **Step 4: Run the full suite**

Run: `cd /home/hieu2/gp4_ws/src/llm_gateway && python -m pytest tests/ -q`
Expected: same count, all pass. Note `Normalizer` uses `_import_geometry_msgs`/`_rpy_to_quaternion` — confirm they moved with it.

- [ ] **Step 5: Build** — `colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash`. Expected: green.

- [ ] **Step 6: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/normalization.py src/llm_gateway/llm_gateway/factory_task.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): extract normalization.py from factory_task (R1)"
npx gitnexus analyze
```

---

## Task 3: Extract `goal_mapper.py`

**Files:**
- Create: `src/llm_gateway/llm_gateway/goal_mapper.py`
- Modify: `src/llm_gateway/llm_gateway/factory_task.py`

- [ ] **Step 1: Impact analysis**

Run: `gitnexus_impact({target: "GoalMapper", direction: "upstream"})`. Expected callers: `llm_gateway_node.py` (`self._goal_mapper`), tests. Re-export keeps node import working.

- [ ] **Step 2: Create the module**

Create `src/llm_gateway/llm_gateway/goal_mapper.py`:
```python
"""ExecuteMotion goal construction + command payload mapping.

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

import json
import logging
from typing import Any, Dict, Protocol

_LOGGER = logging.getLogger(__name__)

# ... paste: _SchemaValidatorLike, prepare_execution_command,
#     command_from_sanitized_json, GoalMapper (cut verbatim from factory_task.py)
```
`GoalMapper.to_execute_motion_goal` imports `from interfaces.action import ExecuteMotion` inside the method — keep that lazy import as-is.

- [ ] **Step 3: Re-export from factory_task.py**

```python
from llm_gateway.goal_mapper import (
    GoalMapper,
    prepare_execution_command,
    command_from_sanitized_json,
)
```
`__all__` unchanged.

- [ ] **Step 4: Run the full suite** — `python -m pytest tests/ -q`. Expected: same count, all pass.

- [ ] **Step 5: Build** — `colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash`. Expected: green.

- [ ] **Step 6: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/goal_mapper.py src/llm_gateway/llm_gateway/factory_task.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): extract goal_mapper.py from factory_task (R1)"
npx gitnexus analyze
```

---

## Task 4: Extract `drawing_router.py`

**Files:**
- Create: `src/llm_gateway/llm_gateway/drawing_router.py`
- Modify: `src/llm_gateway/llm_gateway/factory_task.py`

- [ ] **Step 1: Impact analysis**

Run: `gitnexus_impact({target: "DrawRouterMixin", direction: "upstream"})`. Expected: `IntentRouter` (which subclasses it — moves in Task 5), `hydrate_draw_workplane` callers, tests. Since `IntentRouter(DrawRouterMixin)` still lives in factory_task until Task 5, the re-export must be in place before Task 5 runs.

- [ ] **Step 2: Create the module**

Create `src/llm_gateway/llm_gateway/drawing_router.py`:
```python
"""Drawing/geometry intent routing mixin (text, shapes, workplane hydration).

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

import logging
from typing import Any, Dict

from llm_gateway.drawing_geometry import (
    DrawingGeometryError,
    compile_strokes_to_commands,
    generate_arc_path,
    generate_circle_path,
    generate_polygon_path,
    generate_polyline_path,
    generate_rectangle_path,
    generate_square_path,
    generate_text_stroke_segments,
    generate_triangle_path,
    lift_points_to_poses,
    parse_position_dict,
    parse_vector_dict,
    resolve_workplane,
    supported_glyphs,
    to_meters,
    to_radians,
)

_LOGGER = logging.getLogger(__name__)
_FRAME_BASE_LINK = "base_link"
_ORIENTATION_PRESETS: Dict[str, Dict[str, float]] = {
    # ... paste the preset dict verbatim
}

# ... paste: _build_route_result, hydrate_draw_workplane, DrawRouterMixin
#     (cut verbatim from factory_task.py)
```
Move only the `_ORIENTATION_PRESETS` and `_FRAME_BASE_LINK` definitions that the drawing code uses; the intent-router copies move in Task 5.

- [ ] **Step 3: Re-export from factory_task.py**

```python
from llm_gateway.drawing_router import (
    DrawRouterMixin,
    hydrate_draw_workplane,
)
```
`__all__` unchanged.

- [ ] **Step 4: Run the full suite** — `python -m pytest tests/ -q`. Expected: same count, all pass (`test_core_pipeline.py` exercises `hydrate_draw_workplane`).

- [ ] **Step 5: Build** — `colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash`. Expected: green.

- [ ] **Step 6: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/drawing_router.py src/llm_gateway/llm_gateway/factory_task.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): extract drawing_router.py from factory_task (R1)"
npx gitnexus analyze
```

---

## Task 5: Extract `intent_router.py`

**Files:**
- Create: `src/llm_gateway/llm_gateway/intent_router.py`
- Modify: `src/llm_gateway/llm_gateway/factory_task.py`

- [ ] **Step 1: Impact analysis**

Run: `gitnexus_impact({target: "IntentRouter", direction: "upstream"})`. Expected: `llm_gateway_node.py`, `supervisor_validation.py` (`IntentRouter(runtime_mode=...)`), tests. Re-export keeps both working.

- [ ] **Step 2: Create the module**

Create `src/llm_gateway/llm_gateway/intent_router.py`:
```python
"""Top-level intent router: maps structured intents to routed commands.

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List

from llm_gateway.drawing_router import DrawRouterMixin
from llm_gateway.normalization import Normalizer
from llm_gateway.validation import SchemaValidator, SemanticValidator

_LOGGER = logging.getLogger(__name__)
_FRAME_BASE_LINK = "base_link"
_NAMED_POSE_ALIASES: Dict[str, str] = {
    # ... paste verbatim
}

# ... paste: _copy_semantic_ir, _joint_alias_key, resolve_gp4_joint_index,
#     _current_joint_positions_for_delta, prepare_semantic_ir_for_routing,
#     _default_macro_policy_path, _default_named_pose_srdf_path, load_srdf_named_poses,
#     canonicalize_named_pose, load_macro_policy, RouteResult, IntentRouter
#     (cut verbatim from factory_task.py)
```
`IntentRouter` still declares `class IntentRouter(DrawRouterMixin):` — the import above satisfies it. If `IntentRouter` references `GoalMapper`/validators by bare name, add those imports too (the suite will tell you).

- [ ] **Step 3: Re-export from factory_task.py**

```python
from llm_gateway.intent_router import (
    IntentRouter,
    RouteResult,
    prepare_semantic_ir_for_routing,
    resolve_gp4_joint_index,
    canonicalize_named_pose,
    load_srdf_named_poses,
    load_macro_policy,
)
```
`__all__` unchanged.

- [ ] **Step 4: Run the full suite** — `python -m pytest tests/ -q`. Expected: same count, all pass (`test_intent_router.py`, `test_contracts.py` exercise `IntentRouter`).

- [ ] **Step 5: Build** — `colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash`. Expected: green.

- [ ] **Step 6: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/intent_router.py src/llm_gateway/llm_gateway/factory_task.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): extract intent_router.py from factory_task (R1)"
npx gitnexus analyze
```

---

## Task 6: Extract `composite_tools.py`

**Files:**
- Create: `src/llm_gateway/llm_gateway/composite_tools.py`
- Modify: `src/llm_gateway/llm_gateway/factory_task.py`

- [ ] **Step 1: Impact analysis**

Run: `gitnexus_impact({target: "PickObjectTool", direction: "upstream"})` and `gitnexus_impact({target: "VerifyGraspTool", direction: "upstream"})`. Expected: tests (`test_scene_cache.py`), the runtime skill executor (R2, not yet wired), node. Re-export keeps test imports working.

- [ ] **Step 2: Create the module**

Create `src/llm_gateway/llm_gateway/composite_tools.py`:
```python
"""Composite fail-closed skill tools (pick/approach/place/verify) + pose candidates.

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, ClassVar, Dict

from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract

_LOGGER = logging.getLogger(__name__)

# ... paste: CandidatePoseRequest, CandidatePoseResult, generate_candidate_poses,
#     ToolResult, _CompositeTool, EmitSequenceTool, RefreshSceneTool, PickObjectTool,
#     VerificationResult, PostconditionVerifier, ApproachObjectTool, PlaceObjectTool,
#     VerifyPostconditionTool, mtc_select, VerifyGraspTool
#     (cut verbatim from factory_task.py)
```

- [ ] **Step 3: Re-export from factory_task.py**

```python
from llm_gateway.composite_tools import (
    CandidatePoseRequest,
    CandidatePoseResult,
    ToolResult,
    PostconditionVerifier,
    VerificationResult,
    EmitSequenceTool,
    RefreshSceneTool,
    PickObjectTool,
    ApproachObjectTool,
    PlaceObjectTool,
    VerifyPostconditionTool,
    VerifyGraspTool,
    mtc_select,
    generate_candidate_poses,
)
```
`__all__` unchanged.

- [ ] **Step 4: Run the full suite** — `python -m pytest tests/ -q`. Expected: same count, all pass.

- [ ] **Step 5: Build** — `colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash`. Expected: green.

- [ ] **Step 6: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/composite_tools.py src/llm_gateway/llm_gateway/factory_task.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): extract composite_tools.py from factory_task (R1)"
npx gitnexus analyze
```

---

## Task 7: Verify shrink + final green + docs

**Files:**
- Modify (docs only): `CLAUDE.md` if the package table or pipeline description needs the new module names (per `.claude/rules/when-to-update-claude-docs.md`).

- [ ] **Step 1: Confirm factory_task.py shrank**

Run: `wc -l src/llm_gateway/llm_gateway/factory_task.py`
Expected: ≤ ~900 lines (core dataclasses + WorldModel + TaskCompiler + PolicyEngine + StationSceneGraph + parse helpers + re-export block). If still > 1200, a cluster did not fully move — re-check Tasks 1–6.

- [ ] **Step 2: Confirm no symbol left the public surface**

Run:
```bash
cd /home/hieu2/gp4_ws/src/llm_gateway
python -c "import llm_gateway.factory_task as f; import sys; missing=[n for n in f.__all__ if not hasattr(f,n)]; print('MISSING:', missing); sys.exit(1 if missing else 0)"
```
Expected: `MISSING: []` and exit 0. Every name in `__all__` still resolves through the re-exports.

- [ ] **Step 3: Full suite + build one more time**

Run:
```bash
cd /home/hieu2/gp4_ws/src/llm_gateway && python -m pytest tests/ -q
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
```
Expected: same test count as Task 0, build green.

- [ ] **Step 4: Detect-changes scope check before committing docs**

Run: `gitnexus_detect_changes()`. Confirm the only affected symbols are the moved classes/functions and `factory_task` — no unexpected execution-flow changes.

- [ ] **Step 5: Update CLAUDE.md if needed**

If the root `CLAUDE.md` llm_gateway description names internal modules, add the six new module names. If it only describes the package at a high level, no change needed — note that in the commit.

- [ ] **Step 6: Commit**

```bash
cd /home/hieu2/gp4_ws
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add -A
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): finish R1 factory_task split, factory_task now core-only"
npx gitnexus analyze
```

---

## Done criteria for R1

- [ ] `factory_task.py` ≤ ~900 lines, contains only FactoryTask dataclasses, WorldModel, TaskCompiler, PolicyEngine, StationSceneGraph, parse helpers, and the unchanged `__all__` re-export block.
- [ ] Six new modules exist: `validation.py`, `normalization.py`, `goal_mapper.py`, `drawing_router.py`, `intent_router.py`, `composite_tools.py`.
- [ ] Full suite green at the same count as Task 0 after every task.
- [ ] `__all__` membership check (Task 7 Step 2) passes — zero caller breakage.
- [ ] Build green; GitNexus reindexed.

After R1 lands, write the R2 plan (closed-loop actuate) against the remediation spec §4.
