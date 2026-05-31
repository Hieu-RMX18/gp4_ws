# README Update — Surgical Patch Design

**Date:** 2026-05-31
**Scope:** Root `README.md` only — fix drift between docs and current codebase state.
**Approach:** Option A (surgical patch) — keep existing structure, fix specific outdated sections.
**Audience:** Author (thesis/research) + lab supervisor.

---

## Changes

### 1. Core Features — remove CI/CD bullet

**Why:** `.github/` directory was deleted. The "CI/CD Validation" bullet references GitHub Actions jobs that no longer exist.

**Action:** Delete the "CI/CD Validation" bullet from the Core Features list.

---

### 2. Example Commands — clean up gp4_cmd reference

**Why:** `gp4_cmd` is no longer an entry point in `src/llm_gateway/setup.py`. README currently has a sentence explaining it was "removed during W8 cleanup" — this is historical noise.

**Action:** Remove the paragraph that mentions the legacy `gp4_cmd` CLI. Keep HMI + sim launch instructions.

---

### 3. Getting Started — Camera Perception section

**Why:** Section is ~90 lines covering 5 sub-steps with verbose verification commands. After the RGB+depth pipeline rewrite, the detailed bring-up runbook lives in `src/gp4_perception/README.md`. Root README should be a concise pointer.

**Action:** Replace with ~15 lines:
- Two launch commands (camera-only and perception_full)
- Updated node list: `realsense2_camera_node`, `scene_processor`, `detection_visualizer`, `preprocessing_visualizer`, `tf_publisher`/`calibration_service`
- Updated topic list reflecting current pipeline
- Link to `src/gp4_perception/README.md` for full runbook

**Topic corrections:**
- `/perception/detections` (Detection3DArray) — now published by `detection_visualizer`, not `scene_processor`
- `/perception/annotated_image` — RGB image with bounding boxes (detection_visualizer)
- `/perception/debug_dashboard_image` — composite debug view (detection_visualizer)
- `/perception/zoom_roi_image` — zoomed ROI (detection_visualizer)
- `/perception/debug_mask/blue_border` — HSV mask debug (detection_visualizer)
- `/perception/debug_mask/white_workpiece` — HSV mask debug (detection_visualizer)
- `/perception/status` — pipeline status (scene_processor)

---

### 4. Implementation History — remove entirely

**Why:** ~50 lines of wave-by-wave history (W1–W8). Belongs in git log, not README. Audience is author + supervisor who have git access.

**Action:** Delete the entire "Implementation History" section.

---

## What Does NOT Change

- Safety limits table
- Primitives table (13 public primitives)
- Architecture diagram and package table
- HMI section
- Configuration table
- Testing & Validation section
- Prerequisites and Installation sections

---

## Success Criteria

- No references to `gp4_cmd` CLI or GitHub Actions CI/CD
- Camera Perception section ≤ 20 lines in root README
- Perception topic list matches actual publishers in current codebase
- `preprocessing_visualizer` node appears in node list
- Implementation History section absent
- All remaining content still accurate against current codebase
