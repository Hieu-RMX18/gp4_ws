# Unified Visualizer — Design Spec
**Date:** 2026-06-02  
**Status:** Approved  

---

## 1. Goal

Replace the two separate OpenCV windows (`detection_visualizer` → "GP4 Perception" and `preprocessing_visualizer` → "GP4 Preprocessing Pipeline") with a single unified node and window: **"GP4 Perception"**.

---

## 2. Architecture

| Item | Value |
|------|-------|
| New file | `src/gp4_perception/gp4_perception/unified_visualizer.py` |
| Node name | `unified_visualizer` |
| Window name | `"GP4 Perception"` |
| Old files | `detection_visualizer.py`, `preprocessing_visualizer.py` — kept for backward compat, not deleted |

**Subscriptions** (same as current `detection_visualizer`):
- `/camera/color/image_raw` (Image)
- `/camera/aligned_depth_to_color/image_raw` (Image, 16UC1)
- `/camera/color/camera_info` (CameraInfo)

**Publications** (all existing topics preserved):
- `/perception/detections` (Detection3DArray)
- `/perception/annotated_image` (Image)
- `/perception/debug_dashboard_image` (Image)
- `/perception/preprocessing_debug` (Image)
- `/perception/zoom_roi_image` (Image)
- `/perception/debug_mask/blue_border` (Image)
- `/perception/debug_mask/white_workpiece` (Image)

**Code strategy:** Copy all helpers from both files into `unified_visualizer.py`. No cross-imports between old files.

---

## 3. Window Layout

### 3.1 Tab Bar — 11 tabs (full merge)

```
[Detection] [ALL] [Original] [Blur] [Gray] [CLAHE] [Resize] [Threshold] [Canny] [Contours] [Red Mask]
```

- Rendered as OpenCV-drawn clickable tab bar (same technique as `_TabBar` in `preprocessing_visualizer.py`)
- Click any tab → switches active view
- Light theme: white/light-gray background (#F7F8FA), active tab underlined in blue (#1A73E8)

### 3.2 Trackbars — always visible, 7 total

| Trackbar | Range | Affects |
|----------|-------|---------|
| BBox Filter | 0–1 | Detection: calls `scene_processor/set_parameters` |
| Canny Low | 0–255 | Canny stage |
| Canny High | 0–255 | Canny stage |
| H_Lo | 0–180 | Red mask HSV |
| H_Hi | 0–180 | Red mask HSV |
| S_Min | 0–255 | Red mask HSV |
| V_Min | 0–255 | Red mask HSV |

OpenCV trackbars cannot be hidden per-tab — all 7 always rendered above the tab bar.

### 3.3 Detection Tab Content

```
┌─────────────────────────────────────────┐
│  [trackbars row]                        │
│  [tab bar: Detection* | ALL | ...]      │
├─────────────────┬───────────────────────┤
│  RGB annotated  │  Depth (JET colormap) │  ← 310px height
│  (bbox overlay) │                       │
│  [popup if sel] │                       │
├─────────────────┴───────────────────────┤
│  [keybinding status bar]                │
├─────────────────────────────────────────┤
│  Dashboard text list (scrollable)       │  ← ~150px
└─────────────────────────────────────────┘
```

### 3.4 Preprocessing Tabs (ALL, Original, … Red Mask)

Identical behavior to current `preprocessing_visualizer.py`. The `_compute_stages()` pipeline runs on the RGB frame whenever a preprocessing tab is active.

---

## 4. Annotation System

### 4.1 Click → Floating Popup (Metadata)

- Mouse left-click inside a detection bbox → `_selected_idx` set
- Popup drawn on image (white bg, blue border, system-like font via cv2)
- Shows: `class_id`, `confidence`, `distance`, `cam_xyz`, `base_xyz`, `bbox_px`, `circularity`, `solidity`
- Dashboard row for selected detection highlighted (blue background)
- Click outside all bboxes OR press `Esc` → deselect

### 4.2 Note Input (keyboard)

- With a detection selected, press `n` → enter note mode (`_note_mode = True`)
- `waitKey` captures chars: printable → append to `_note_buffer`; Backspace → delete last; Enter → confirm save to `_notes[idx]`; Esc → cancel
- Note displayed in popup + dashboard row

### 4.3 ROI Drawing

- Press `r` → toggle draw mode (`_draw_roi = True`)
- `EVENT_LBUTTONDOWN` → record start point
- `EVENT_MOUSEMOVE` (with button held) → update end point (live preview as dashed yellow rect)
- `EVENT_LBUTTONUP` → finalize ROI, append to `_rois: list[tuple]`
- Press `c` → clear all ROIs
- ROIs rendered as dashed yellow rectangles on Detection tab only

### 4.4 Export Snapshot

- Press `s` → save to `/tmp/gp4_snapshot_<timestamp>/`
  - `frame.png` — current annotated frame (RGB+Depth hstack)
  - `detections.json` — all detection data + notes + ROI list
- Log path to ROS logger

---

## 5. Mouse Callback Unified

Single `cv2.setMouseCallback("GP4 Perception", _on_mouse)` handles:
1. Tab bar hit-test (y < tab_bar_bottom)
2. ROI draw (if `_draw_roi` mode)
3. Detection bbox click (Detection tab only)

Priority: tab click > ROI draw > detection select.

---

## 6. Launch File Update

`perception_full.launch.py` — add parameter `use_unified_gui` (default `True`):
- `True` → launch `unified_visualizer` node
- `False` → launch `detection_visualizer` + `preprocessing_visualizer` (legacy)

---

## 7. Light Theme (OpenCV rendering)

All UI panels (tab bar, dashboard, popup, key bar) drawn on **light** numpy backgrounds:
- Panel bg: `(247, 248, 250)` BGR
- Border: `(208, 213, 221)` BGR  
- Text: `(26, 26, 26)` BGR
- Active/highlight: `(30, 115, 26)` BGR ← OpenCV BGR for #1A73E8
- Note accent: `(0, 115, 232)` BGR

Font: `cv2.FONT_HERSHEY_SIMPLEX` (OpenCV limitation — no system fonts in cv2.putText)

---

## 8. Out of Scope

- No changes to detection logic (`detect_color_objects`, NMS, depth deprojection)
- No changes to existing ROS2 topic names or message types
- No changes to `detection_visualizer.py` or `preprocessing_visualizer.py`
- No web UI or external dependencies beyond what's already used
