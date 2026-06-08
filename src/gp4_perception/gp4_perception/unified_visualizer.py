"""Unified perception visualizer — single-window debug tool.

Merges detection_visualizer.py and preprocessing_visualizer.py into
one ROS 2 node with a tabbed OpenCV interface and light-theme UI.

Subscribes to:
    /camera/color/image_raw (Image)
    /camera/aligned_depth_to_color/image_raw (Image, 16UC1)
    /camera/color/camera_info (CameraInfo)

Publishes:
    /perception/detections (Detection3DArray)
    /perception/annotated_image (Image)
    /perception/debug_dashboard_image (Image)
    /perception/preprocessing_debug (Image)
    /perception/zoom_roi_image (Image)
    /perception/debug_mask/blue_border (Image)
    /perception/debug_mask/white_workpiece (Image)

Entry point: ros2 run gp4_perception unified_visualizer
"""

from __future__ import annotations

import collections
import json
import logging
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

from gp4_perception.latest_frame import MonotonicRateGate
from gp4_perception.visual_tracking import VisualDetectionTracker

_LOGGER = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════
# Light-theme palette (BGR for OpenCV)
# ═══════════════════════════════════════════════════════════════════════
_PANEL_BG = (250, 248, 247)       # #F7F8FA
_PANEL_BORDER = (221, 213, 208)   # #D0D5DD
_TEXT_DARK = (26, 26, 26)         # #1A1A1A
_TEXT_MUTED = (140, 140, 140)     # #8C8C8C
_TAB_BG_COLOR = (245, 245, 245)  # #F5F5F5
_TAB_ACTIVE_BG = (232, 115, 26)  # #1A73E8 BGR
_TAB_HOVER_BG = (230, 230, 230)  # #E6E6E6
_TAB_TEXT_COLOR = (100, 100, 100) # #646464
_TAB_ACTIVE_TEXT = (255, 255, 255)
_TAB_BORDER_COLOR = (208, 208, 208)
_ACCENT_BLUE = (232, 115, 26)    # #1A73E8 BGR
_NOTE_ACCENT = (232, 115, 0)     # #0073E8 BGR
_STATUS_OK = (80, 180, 80)
_STATUS_BAR_BG = (235, 235, 235)
_POPUP_BG = (255, 255, 255)
_POPUP_BORDER = (232, 115, 26)   # blue border

# ═══════════════════════════════════════════════════════════════════════
# Tab definitions
# ═══════════════════════════════════════════════════════════════════════
TAB_DETECTION = 0
TAB_ALL = 1
TAB_ORIGINAL = 2
TAB_BLUR = 3
TAB_GRAY = 4
TAB_CLAHE = 5
TAB_RESIZED = 6
TAB_THRESHOLD = 7
TAB_CANNY = 8
TAB_CONTOURS = 9
TAB_COLOUR_MASK = 10

_TAB_LABELS: list[tuple[int, str]] = [
    (TAB_DETECTION, "Detection"),
    (TAB_ALL, "ALL"),
    (TAB_ORIGINAL, "Original"),
    (TAB_BLUR, "Blur"),
    (TAB_GRAY, "Gray"),
    (TAB_CLAHE, "CLAHE"),
    (TAB_RESIZED, "Resize"),
    (TAB_THRESHOLD, "Threshold"),
    (TAB_CANNY, "Canny"),
    (TAB_CONTOURS, "Contours"),
    (TAB_COLOUR_MASK, "Colour Mask"),
]

_TAB_H = 36
_TAB_FONT = cv2.FONT_HERSHEY_SIMPLEX
_TAB_FONT_SCALE = 0.42
_TAB_FONT_THICK = 1
_STD_W, _STD_H = 320, 240
_WINDOW_NAME = "GP4 Perception"

# Colour Mask presets: (name, bgr_display, h_min, h_max, s_min, v_min, wraps_hue)
_COLOR_PRESETS: list[tuple[str, tuple, int, int, int, int, bool]] = [
    ("Red",    (0, 0, 220),   10, 170, 80,  60,  True),
    ("Blue",   (220, 100, 0), 100, 130, 80,  60,  False),
    ("Green",  (0, 180, 0),   35,  85,  80,  60,  False),
    ("Yellow", (0, 230, 230), 20,  35,  80,  60,  False),
    ("Orange", (0, 140, 240), 10,  25,  100, 100, False),
    ("White",  (230, 230, 230), 0, 180, 0,   200, False),
]


# Color palette for different class labels (BGR for OpenCV).
_COLORS = {
    "red": (0, 0, 255),
    "orange": (0, 140, 255),
    "yellow": (0, 255, 255),
    "green": (0, 200, 0),
    "blue": (255, 100, 0),
    "purple": (200, 0, 200),
    "white": (255, 255, 255),
    "gray": (160, 160, 160),
    "black": (80, 80, 80),
}
_DEFAULT_COLOR = (0, 255, 128)  # bright green


def _color_for_class(class_id: str) -> tuple[int, int, int]:
    """Pick a display color based on the class_id prefix (color name)."""
    parts = class_id.split("_") if class_id else []
    if parts:
        color_name = parts[0].lower()
        if color_name in _COLORS:
            return _COLORS[color_name]
    return _DEFAULT_COLOR


def _deproject_pixel(
    u: float, v: float, z_m: float, fx: float, fy: float, cx: float, cy: float
) -> tuple[float, float, float]:
    """Pinhole deprojection of one pixel to camera_color_optical_frame metres."""
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return (x, y, z_m)


def _median_depth_m(
    depth_raw: np.ndarray | None,
    mask: np.ndarray | None,
    depth_scale: float = 0.001,
) -> float | None:
    """Median depth in metres over masked, nonzero pixels.

    Returns None when no valid (masked & nonzero) pixel exists.
    """
    if depth_raw is None or mask is None:
        return None
    valid = (mask > 0) & (depth_raw > 0)
    sel = depth_raw[valid]
    if sel.size == 0:
        return None
    return float(np.median(sel)) * depth_scale


def _bbox_size_m(
    w_px: float, h_px: float, z_m: float, fx: float, fy: float
) -> tuple[float, float]:
    """Estimate metric bbox width/height from pixel extent at depth z."""
    return (w_px * z_m / fx, h_px * z_m / fy)


def _downscale_to_max_width(image: np.ndarray, max_width: int | None) -> np.ndarray:
    """Return a width-limited image while preserving aspect ratio."""
    if max_width is None or max_width <= 0:
        return image
    h, w = image.shape[:2]
    if w <= max_width:
        return image
    scale = float(max_width) / float(w)
    return cv2.resize(image, (int(max_width), max(1, int(h * scale))), interpolation=cv2.INTER_AREA)


# ---------------------------------------------------------------------------
# Geometric helpers for NMS / suppression / shape filtering
# ---------------------------------------------------------------------------


def _bbox_iou(a: tuple, b: tuple) -> float:
    """Compute IoU between two (x, y, w, h) bounding boxes."""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = aw * ah
    area_b = bw * bh
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _bbox_center_inside(inner_bbox: tuple, outer_bbox: tuple) -> bool:
    """Check if center of inner_bbox is inside outer_bbox."""
    ix, iy, iw, ih = inner_bbox
    ox, oy, ow, oh = outer_bbox
    cx = ix + iw / 2
    cy = iy + ih / 2
    return ox <= cx <= ox + ow and oy <= cy <= oy + oh


def _bbox_fully_inside(inner: tuple, outer: tuple) -> bool:
    """Check if inner bbox is fully contained within outer bbox."""
    ix, iy, iw, ih = inner
    ox, oy, ow, oh = outer
    return ix >= ox and iy >= oy and ix + iw <= ox + ow and iy + ih <= oy + oh


def _compute_shape_metrics(contour) -> dict:
    """Compute 2D shape metrics from a contour.

    Returns dict with: area, perimeter, circularity, solidity,
    aspect_ratio, rotated_aspect_ratio.
    """
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    circularity = (4.0 * np.pi * area) / (perimeter * perimeter) if perimeter > 1e-6 else 0.0
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = max(w, h) / max(1, min(w, h))
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area > 1e-6 else 0.0
    # Rotated bounding rect aspect ratio (handles tilted objects).
    rot_rect = cv2.minAreaRect(contour)
    rw, rh = rot_rect[1]
    if rw > 1e-6 and rh > 1e-6:
        rotated_aspect_ratio = max(rw, rh) / min(rw, rh)
    else:
        rotated_aspect_ratio = aspect_ratio
    return {
        "area": float(area),
        "perimeter": float(perimeter),
        "circularity": float(circularity),
        "solidity": float(solidity),
        "aspect_ratio": float(aspect_ratio),
        "rotated_aspect_ratio": float(rotated_aspect_ratio),
    }


def _apply_shape_filter(shape_metrics: dict, shape_cfg: dict | None) -> bool:
    """Check if shape metrics pass the per-class shape filter config.

    Returns True if candidate passes (or no filter configured).
    """
    if not shape_cfg:
        return True
    circ = shape_metrics.get("circularity", 0.0)
    sol = shape_metrics.get("solidity", 0.0)
    ar = shape_metrics.get("aspect_ratio", 1.0)

    if "min_circularity" in shape_cfg and circ < shape_cfg["min_circularity"]:
        return False
    if "min_solidity" in shape_cfg and sol < shape_cfg["min_solidity"]:
        return False
    if "min_aspect_ratio" in shape_cfg and ar < shape_cfg["min_aspect_ratio"]:
        return False
    if "max_aspect_ratio" in shape_cfg and ar > shape_cfg["max_aspect_ratio"]:
        return False
    return True


def _format_distance(distance_m: float, unit: str = "mm") -> str:
    """Format distance for display. Returns e.g. '545mm' or '0.545m'."""
    if unit == "mm":
        return f"{int(round(distance_m * 1000.0))}mm"
    return f"{distance_m:.3f}m"


def _cross_class_nms(
    candidates: list[dict],
    postprocess_cfg: dict,
) -> list[dict]:
    """Apply cross-class NMS and pair-specific suppression.

    Order:
    1. Sort by class priority then confidence.
    2. Same-class NMS: deduplicate overlapping same-class candidates.
    3. Cross-class duplicate suppression: high IoU across classes → keep priority.
    4. Pair-specific suppression: apple/orange inside red_box, etc.
    """
    if not candidates:
        return []

    priority_list = postprocess_cfg.get("class_priority", [])
    priority_map = {cls: i for i, cls in enumerate(priority_list)}
    same_iou = float(postprocess_cfg.get("same_class_nms_iou", 0.50))
    cross_iou = float(postprocess_cfg.get("duplicate_cross_class_iou", 0.70))
    pair_cfg = postprocess_cfg.get("pair_suppression", {})

    def _priority(c):
        return priority_map.get(c.get("class_id", ""), len(priority_list))

    # Sort: lower priority number = higher importance, then higher confidence.
    sorted_cands = sorted(candidates, key=lambda c: (_priority(c), -c.get("confidence", 0.0)))

    kept: list[dict] = []
    suppressed: set[int] = set()

    for i, cand in enumerate(sorted_cands):
        if i in suppressed:
            continue
        for j in range(i + 1, len(sorted_cands)):
            if j in suppressed:
                continue
            other = sorted_cands[j]
            iou = _bbox_iou(cand["bbox"], other["bbox"])

            # Same-class NMS.
            if cand["class_id"] == other["class_id"] and iou > same_iou:
                suppressed.add(j)
                continue

            # Cross-class duplicate suppression (very high overlap).
            if cand["class_id"] != other["class_id"] and iou > cross_iou:
                suppressed.add(j)
                continue

        kept.append(cand)

    # Pair-specific suppression.
    final: list[dict] = []
    for cand in kept:
        cid = cand["class_id"]
        if cid not in pair_cfg:
            final.append(cand)
            continue
        suppress = False
        for suppressor_class, rules in pair_cfg[cid].items():
            for other in kept:
                if other["class_id"] != suppressor_class:
                    continue
                if other is cand:
                    continue
                # Center-inside check.
                if rules.get("center_inside", False):
                    if _bbox_center_inside(cand["bbox"], other["bbox"]):
                        suppress = True
                        break
                # IoU threshold check.
                iou_thresh = rules.get("iou_gt")
                if iou_thresh is not None:
                    if _bbox_iou(cand["bbox"], other["bbox"]) > float(iou_thresh):
                        suppress = True
                        break
                # Fully-inside check.
                if rules.get("fully_inside_only", False):
                    if _bbox_fully_inside(cand["bbox"], other["bbox"]):
                        suppress = True
                        break
            if suppress:
                break
        if not suppress:
            final.append(cand)

    return final


# ---------------------------------------------------------------------------
# Blue-border white-fill detector for white_workpiece
# ---------------------------------------------------------------------------


def _detect_blue_border_white_fill(
    rgb_image: np.ndarray,
    hsv_image: np.ndarray,
    cfg: dict,
) -> list[dict]:
    """Detect white_workpiece by finding blue border contours first,
    then validating white interior fill ratio.

    Returns list of candidate dicts matching detect_color_objects format.
    """
    blue_hsv = cfg.get("blue_border_hsv", [95, 50, 40, 135, 255, 255])
    white_hsv = cfg.get("white_fill_hsv", [0, 0, 135, 179, 90, 255])
    min_area = cfg.get("min_area_px", 500)
    max_area = cfg.get("max_area_px", 30000)
    inner_white_min = float(cfg.get("inner_white_min_ratio", 0.45))
    min_blue_ratio = float(cfg.get("min_blue_border_ratio", 0.04))
    morph_kern = cfg.get("morph_kernel", 5)
    shape_cfg = cfg.get("shape_filter", {})

    # Blue border mask.
    blue_lo = np.array(blue_hsv[:3], dtype=np.uint8)
    blue_hi = np.array(blue_hsv[3:], dtype=np.uint8)
    blue_mask = cv2.inRange(hsv_image, blue_lo, blue_hi)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kern, morph_kern))
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=1)

    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results: list[dict] = []

    # White fill mask.
    white_lo = np.array(white_hsv[:3], dtype=np.uint8)
    white_hi = np.array(white_hsv[3:], dtype=np.uint8)
    white_mask = cv2.inRange(hsv_image, white_lo, white_hi)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        # Shape filter.
        metrics = _compute_shape_metrics(cnt)
        if not _apply_shape_filter(metrics, shape_cfg):
            continue

        # Blue border ratio: blue pixels within outer bbox.
        roi_blue = blue_mask[y:y + h, x:x + w]
        blue_count = cv2.countNonZero(roi_blue)
        total_px = w * h
        blue_ratio = blue_count / total_px if total_px > 0 else 0.0
        if blue_ratio < min_blue_ratio:
            continue

        # Inner ROI: shrink bbox by border thickness to check white interior.
        border_px = max(5, int(min(w, h) * 0.15))
        ix1 = x + border_px
        iy1 = y + border_px
        ix2 = x + w - border_px
        iy2 = y + h - border_px
        if ix2 <= ix1 or iy2 <= iy1:
            continue

        inner_white = white_mask[iy1:iy2, ix1:ix2]
        inner_total = (ix2 - ix1) * (iy2 - iy1)
        white_count = cv2.countNonZero(inner_white)
        white_ratio = white_count / inner_total if inner_total > 0 else 0.0
        if white_ratio < inner_white_min:
            continue

        # Build object mask and candidate.
        obj_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        cv2.drawContours(obj_mask, [cnt], -1, 255, -1)
        # Fill the interior white region too.
        obj_mask[iy1:iy2, ix1:ix2] = np.maximum(
            obj_mask[iy1:iy2, ix1:ix2], white_mask[iy1:iy2, ix1:ix2]
        )

        cx = x + w // 2
        cy_val = y + h // 2
        confidence = float(min(white_ratio, blue_ratio * 5.0, metrics["solidity"]))
        results.append({
            "class_id": "white_workpiece",
            "bbox": (x, y, w, h),
            "mask": obj_mask,
            "contour": cnt,
            "center_uv": (cx, cy_val),
            "confidence": float(confidence),
            "shape_metrics": metrics,
            "source": "blue_border_white_fill",
            "reject_if_inside_class": cfg.get("reject_if_inside_class"),
            "_blue_mask": blue_mask,
            "_white_mask": obj_mask,
        })

    return results


# ---------------------------------------------------------------------------
# HSV multi-range contour detection
# ---------------------------------------------------------------------------

def detect_color_objects(
    rgb_image: np.ndarray,
    color_classes: list[dict],
    postprocess_cfg: dict | None = None,
) -> list[dict]:
    """Detect objects via HSV masking per configured color class.

    Pipeline:
    1. Per-class HSV contour detection (or special detector dispatch).
    2. Shape metrics computation + per-class shape filter.
    3. Cross-class NMS and pair suppression.

    Args:
        rgb_image: H×W×3 uint8 RGB image.
        color_classes: List of class configs from perception.yaml color_classes.
        postprocess_cfg: Postprocess config with NMS/suppression rules.

    Returns:
        List of dicts with keys: class_id, bbox (x,y,w,h), mask, contour,
        center_uv, confidence, shape_metrics.
    """
    raw_candidates: list[dict] = []
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

    for cls in color_classes:
        if not cls.get("enabled", True):
            continue
        class_id = cls["class_id"]
        detector_type = cls.get("detector_type", "hsv_contour")

        # Special detector dispatch.
        if detector_type == "blue_border_white_fill":
            special = _detect_blue_border_white_fill(rgb_image, hsv, cls)
            raw_candidates.extend(special)
            continue

        # Standard HSV contour path.
        ranges = cls.get("hsv_ranges", [])
        min_area = cls.get("min_area_px", 300)
        max_area = cls.get("max_area_px", 999999)
        kern_sz = cls.get("morph_kernel", 3)
        shape_cfg = cls.get("shape_filter")

        # Union of all HSV ranges for this class.
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for rng in ranges:
            lo = np.array(rng[:3], dtype=np.uint8)
            hi = np.array(rng[3:], dtype=np.uint8)
            combined_mask |= cv2.inRange(hsv, lo, hi)

        # Morphology cleanup.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kern_sz, kern_sz))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)

            # Shape metrics and filter.
            metrics = _compute_shape_metrics(cnt)
            if not _apply_shape_filter(metrics, shape_cfg):
                continue

            # Build per-object mask.
            obj_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(obj_mask, [cnt], -1, 255, -1)
            # Border-hue gate.
            if cls.get("require_border"):
                if not validate_border_hue(
                    rgb_image, obj_mask, (x, y, w, h), cls["require_border"]
                ):
                    continue
            # Confidence from solidity.
            confidence = float(metrics["solidity"])
            cx = x + w // 2
            cy = y + h // 2
            raw_candidates.append({
                "class_id": class_id,
                "bbox": (x, y, w, h),
                "mask": obj_mask,
                "contour": cnt,
                "center_uv": (cx, cy),
                "confidence": float(confidence),
                "shape_metrics": metrics,
                "source": "hsv_contour",
                "reject_if_inside_class": cls.get("reject_if_inside_class"),
            })

    # Cross-class NMS and pair suppression.
    if postprocess_cfg:
        return _cross_class_nms(raw_candidates, postprocess_cfg)

    return raw_candidates


# ---------------------------------------------------------------------------
# 2D border-hue validation (Task 4)
# ---------------------------------------------------------------------------

# HSV ranges for border colour validation (OpenCV HSV: H 0-179).
_BORDER_HSV_RANGES: dict[str, list[tuple[int, ...]]] = {
    "blue": [(100, 80, 40, 130, 255, 255)],
    "red": [(0, 80, 50, 10, 255, 255), (170, 80, 50, 179, 255, 255)],
    "green": [(35, 60, 40, 85, 255, 255)],
}


def validate_border_hue(
    rgb_image: np.ndarray,
    mask: np.ndarray,
    bbox: tuple[int, int, int, int],
    required_border: str | None,
) -> bool:
    """Check if the border pixels of a detected object match the required hue.

    Uses morphological erosion to isolate a border ring, then HSV-votes
    the border pixels for the required colour.

    Args:
        rgb_image: Full H×W×3 uint8 RGB image.
        mask: H×W uint8 object mask (255 = object).
        bbox: (x, y, w, h) bounding box.
        required_border: Colour name ("blue", "red", "green") or None.

    Returns:
        True if no border required, or if ≥25% of border pixels match.
    """
    if required_border is None:
        return True

    x, y, w, h = bbox
    roi_mask = mask[y:y + h, x:x + w]
    if roi_mask.sum() == 0:
        return False

    # Erode to get interior, subtract to get border ring.
    # borderValue=0 ensures edge pixels are excluded from interior.
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    interior = cv2.erode(
        roi_mask, kern, iterations=2,
        borderType=cv2.BORDER_CONSTANT, borderValue=0,
    )
    border_ring = cv2.subtract(roi_mask, interior)
    if cv2.countNonZero(border_ring) < 10:
        return False

    roi_rgb = rgb_image[y:y + h, x:x + w]
    border_pixels = roi_rgb[border_ring > 0]

    ranges = _BORDER_HSV_RANGES.get(required_border.lower(), [])
    if not ranges:
        return False

    hsv_pixels = cv2.cvtColor(
        border_pixels.reshape(-1, 1, 3), cv2.COLOR_RGB2HSV
    ).reshape(-1, 3)
    match_count = 0
    for rng in ranges:
        lo = np.array(rng[:3], dtype=np.uint8)
        hi = np.array(rng[3:], dtype=np.uint8)
        match_count += int(np.count_nonzero(
            cv2.inRange(hsv_pixels.reshape(-1, 1, 3), lo, hi)
        ))
    ratio = match_count / len(hsv_pixels)
    return ratio >= 0.25


def _draw_detection_callout(
    image: np.ndarray,
    bbox: tuple[int, int, int, int],
    lines: list[str],
    color_bgr: tuple[int, int, int],
    font_cfg: dict,
    tag_fallback: bool = False,
    tag_text: str = "",
) -> None:
    """Draw bounding box label with background, clamping, and leader line logic."""
    x, y, w, h = bbox
    scale = float(font_cfg.get("scale", 0.4))
    thickness = int(font_cfg.get("thickness", 1))
    pad = int(font_cfg.get("padding_px", 4))
    line_spacing = int(font_cfg.get("line_spacing_px", 14))
    alpha = float(font_cfg.get("background_alpha", 0.6))
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    if not lines:
        return
        
    def _get_size(text_lines):
        tw, th = 0, 0
        line_heights = []
        for line in text_lines:
            (lw, lh), _ = cv2.getTextSize(line, font, scale, thickness)
            tw = max(tw, lw)
            line_heights.append(lh)
            th += lh + line_spacing
        th -= line_spacing
        return tw, th, line_heights
        
    text_w, text_h, line_heights = _get_size(lines)
    box_w, box_h = text_w + 2 * pad, text_h + 2 * pad
    img_h, img_w = image.shape[:2]
    
    # Try above
    px, py = x, y - box_h - pad
    leader_start = (x + w//2, y)
    leader_end = (px + box_w//2, py + box_h)
    fits = True
    
    if py < 0 or px + box_w > img_w or px < 0:
        # Try below
        px, py = x, y + h + pad
        leader_start = (x + w//2, y + h)
        leader_end = (px + box_w//2, py)
        if py + box_h > img_h or px + box_w > img_w or px < 0:
            # Try right
            px, py = x + w + pad, max(0, y)
            leader_start = (x + w, y + h//2)
            leader_end = (px, py + box_h//2)
            if px + box_w > img_w or py + box_h > img_h:
                fits = False
                
    lines_to_draw = lines
    if not fits:
        if tag_fallback and tag_text:
            lines_to_draw = [tag_text]
            text_w, text_h, line_heights = _get_size(lines_to_draw)
            box_w, box_h = text_w + 2 * pad, text_h + 2 * pad
        # Clamp to image bounds
        px = max(0, min(px, img_w - box_w))
        py = max(0, min(py, img_h - box_h))
        # Update leader end after clamping
        leader_end = (px + box_w//2, py + box_h//2)

    overlay = image.copy()
    cv2.rectangle(overlay, (px, py), (px + box_w, py + box_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)
    
    curr_y = py + pad + line_heights[0]
    for i, line in enumerate(lines_to_draw):
        cv2.putText(image, line, (px + pad, curr_y), font, scale, (255, 255, 255), thickness, lineType=cv2.LINE_AA)
        if i + 1 < len(line_heights):
            curr_y += line_heights[i+1] + line_spacing
            
    cv2.line(image, leader_start, leader_end, color_bgr, 1)
# ---------------------------------------------------------------------------
# Zoom ROI grid builder
# ---------------------------------------------------------------------------


def _build_zoom_grid(
    rgb_image: np.ndarray,
    enriched_objects: list[dict],
    zoom_cfg: dict,
    font_cfg: dict,
) -> np.ndarray | None:
    """Build a zoom grid image showing cropped detections.

    Supports modes: single, grid, selected_class, selected_id.
    """
    mode = zoom_cfg.get("mode", "grid")
    top_n = int(zoom_cfg.get("top_n", 6))
    grid_cols = int(zoom_cfg.get("grid_cols", 3))
    cell_size = int(zoom_cfg.get("grid_cell_size", 220))
    margin = int(zoom_cfg.get("margin_px", 20))
    sort_by = zoom_cfg.get("sort_by", "priority_then_score")

    if not enriched_objects:
        return None

    # Filter by mode.
    candidates = list(enriched_objects)
    if mode == "selected_class":
        sel_class = zoom_cfg.get("selected_class", "")
        if sel_class:
            candidates = [o for o in candidates if o["class_id"] == sel_class]
    elif mode == "selected_id":
        sel_id = int(zoom_cfg.get("selected_id", -1))
        if sel_id >= 0:
            candidates = [o for o in candidates if o.get("obj_id") == f"#{sel_id}"]

    if not candidates:
        return None

    # Sort.
    if sort_by == "priority_then_score":
        # Use natural order from NMS (already priority-sorted), then score.
        candidates = sorted(candidates, key=lambda o: -o.get("confidence", 0.0))

    # Single mode: just one crop.
    if mode == "single":
        candidates = candidates[:1]
    else:
        candidates = candidates[:top_n]

    # Build crops.
    crops: list[np.ndarray] = []
    h_img, w_img = rgb_image.shape[:2]

    for obj in candidates:
        x, y, w, h = obj["bbox"]
        y1 = max(0, y - margin)
        y2 = min(h_img, y + h + margin)
        x1 = max(0, x - margin)
        x2 = min(w_img, x + w + margin)

        crop = rgb_image[y1:y2, x1:x2].copy()
        if crop.size == 0:
            continue

        # Resize to cell_size maintaining aspect ratio.
        ch, cw = crop.shape[:2]
        scale = min(cell_size / cw, cell_size / ch)
        new_w = int(cw * scale)
        new_h = int(ch * scale)
        resized = cv2.resize(crop, (new_w, new_h))

        # Pad to cell_size × cell_size.
        cell = np.zeros((cell_size, cell_size, 3), dtype=np.uint8)
        y_off = (cell_size - new_h) // 2
        x_off = (cell_size - new_w) // 2
        cell[y_off:y_off + new_h, x_off:x_off + new_w] = resized

        # Draw bbox on resized crop.
        bx = int((x - x1) * scale) + x_off
        by = int((y - y1) * scale) + y_off
        bw = int(w * scale)
        bh = int(h * scale)
        color_bgr = _color_for_class(obj["class_id"])
        cv2.rectangle(cell, (bx, by), (bx + bw, by + bh), color_bgr, 2)

        # Compact label: #id class score dist
        obj_id = obj.get("obj_id", "")
        dist_str = obj.get("dist_str", "")
        conf = obj.get("confidence", 0.0)
        label = f"{obj_id} {obj['class_id']} {conf:.2f} {dist_str}"
        cv2.putText(cell, label, (4, 16), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (255, 255, 255), 1, lineType=cv2.LINE_AA)

        crops.append(cell)

    if not crops:
        return None

    # Arrange into grid.
    n = len(crops)
    cols = min(grid_cols, n)
    rows = (n + cols - 1) // cols
    # Pad with empty cells.
    while len(crops) < rows * cols:
        crops.append(np.zeros((cell_size, cell_size, 3), dtype=np.uint8))

    grid_rows = []
    for r in range(rows):
        row_cells = crops[r * cols:(r + 1) * cols]
        grid_rows.append(np.hstack(row_cells))
    return np.vstack(grid_rows)
# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def _put_label(img: np.ndarray, text: str) -> np.ndarray:
    """Draw a label with dark background at top-left corner."""
    out = img.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.50
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    cv2.rectangle(out, (0, 0), (tw + 12, th + baseline + 12), (0, 0, 0), -1)
    cv2.putText(out, text, (6, th + 6), font, scale, (0, 255, 200), thickness, cv2.LINE_AA)
    return out


def _ensure_bgr(img: np.ndarray) -> np.ndarray:
    """Ensure image is 3-channel BGR for display / tiling."""
    if len(img.shape) == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img



# ═══════════════════════════════════════════════════════════════════════
# Dashed rectangle helper (for ROI drawing)
# ═══════════════════════════════════════════════════════════════════════

def _draw_dashed_rect(
    img: np.ndarray, pt1: tuple, pt2: tuple,
    color: tuple = (0, 255, 255), thickness: int = 2, dash_len: int = 10,
) -> None:
    """Draw a dashed rectangle on *img* (in-place)."""
    x1, y1 = pt1
    x2, y2 = pt2
    edges = [
        ((x1, y1), (x2, y1)),
        ((x2, y1), (x2, y2)),
        ((x2, y2), (x1, y2)),
        ((x1, y2), (x1, y1)),
    ]
    for (sx, sy), (ex, ey) in edges:
        length = max(abs(ex - sx), abs(ey - sy))
        if length == 0:
            continue
        dx = (ex - sx) / length
        dy = (ey - sy) / length
        i = 0
        while i < length:
            seg_end = min(i + dash_len, length)
            p1 = (int(sx + dx * i), int(sy + dy * i))
            p2 = (int(sx + dx * seg_end), int(sy + dy * seg_end))
            cv2.line(img, p1, p2, color, thickness)
            i += dash_len * 2


# ═══════════════════════════════════════════════════════════════════════
# Tab bar (light theme)
# ═══════════════════════════════════════════════════════════════════════

class _TabBar:
    """Row of clickable tabs — light theme."""

    def __init__(self) -> None:
        self._rects: list[tuple[int, int, int, int, int]] = []
        self._hover_mode: int | None = None

    def render(self, width: int, active_mode: int) -> np.ndarray:
        bar = np.full((_TAB_H, width, 3), _TAB_BG_COLOR, dtype=np.uint8)
        self._rects.clear()
        n = len(_TAB_LABELS)
        pad_x = 3
        total_pad = pad_x * (n + 1)
        tab_w = (width - total_pad) // n
        x = pad_x
        y_top, y_bot = 4, _TAB_H - 4

        for mode, label in _TAB_LABELS:
            x2 = x + tab_w
            if mode == active_mode:
                fill, txt_col = _TAB_ACTIVE_BG, _TAB_ACTIVE_TEXT
            elif mode == self._hover_mode:
                fill, txt_col = _TAB_HOVER_BG, _TAB_TEXT_COLOR
            else:
                fill, txt_col = _TAB_BG_COLOR, _TAB_TEXT_COLOR

            cv2.rectangle(bar, (x, y_top), (x2, y_bot), fill, -1)
            cv2.rectangle(bar, (x, y_top), (x2, y_bot), _TAB_BORDER_COLOR, 1)
            if mode == active_mode:
                cv2.line(bar, (x, y_bot), (x2, y_bot), _ACCENT_BLUE, 2)
            (tw, th), _ = cv2.getTextSize(label, _TAB_FONT, _TAB_FONT_SCALE, _TAB_FONT_THICK)
            tx = x + (tab_w - tw) // 2
            ty = y_top + (y_bot - y_top + th) // 2
            cv2.putText(bar, label, (tx, ty), _TAB_FONT, _TAB_FONT_SCALE,
                        txt_col, _TAB_FONT_THICK, cv2.LINE_AA)
            self._rects.append((x, y_top, x2, y_bot, mode))
            x = x2 + pad_x
        return bar

    def hit_test(self, mx: int, my: int) -> int | None:
        for x1, y1, x2, y2, mode in self._rects:
            if x1 <= mx <= x2 and y1 <= my <= y2:
                return mode
        return None

    def set_hover(self, mx: int, my: int) -> None:
        self._hover_mode = self.hit_test(mx, my)



# ═══════════════════════════════════════════════════════════════════════
# ROS 2 Node
# ═══════════════════════════════════════════════════════════════════════

class UnifiedVisualizer(Node):
    """Unified perception debug window with tabbed light-theme UI."""

    def __init__(self) -> None:
        super().__init__("unified_visualizer")

        # ---- Config ----
        try:
            from pathlib import Path
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory("gp4_perception")) / "config"
        except Exception:
            from pathlib import Path
            share = Path(__file__).resolve().parents[1] / "config"

        import yaml
        with open(share / "perception.yaml") as f:
            cfg = yaml.safe_load(f) or {}
        pcfg = cfg.get("perception", {})
        self._color_classes: list[dict] = pcfg.get("color_classes", [])
        self._postprocess_cfg: dict = pcfg.get("postprocess", {})
        visual_tracking_cfg = pcfg.get("visual_tracking", {})
        self._visual_tracking_enabled = bool(visual_tracking_cfg.get("enabled", True))
        self._visual_tracker = VisualDetectionTracker(
            min_hits=int(visual_tracking_cfg.get("min_hits", 2)),
            miss_tolerance_frames=int(visual_tracking_cfg.get("miss_tolerance_frames", 2)),
            max_centroid_distance_px=float(visual_tracking_cfg.get("max_centroid_distance_px", 45.0)),
            min_iou=float(visual_tracking_cfg.get("min_iou", 0.15)),
            ema_alpha=float(visual_tracking_cfg.get("ema_alpha", 0.35)),
        )
        rgb_cfg = pcfg.get("rgb_detector", {})
        viz_cfg = pcfg.get("visualization", {})

        self._depth_scale = float(rgb_cfg.get("depth_scale_m", 0.001))
        self._base_frame = str(rgb_cfg.get("base_frame", "base_link"))
        self._camera_frame = str(rgb_cfg.get("camera_optical_frame",
                                              "camera_color_optical_frame"))
        self._bbox_z = float(rgb_cfg.get("bbox_thickness_z_m", 0.03))
        self._sync_slop = float(rgb_cfg.get("sync_slop_s", 0.05))
        self._sync_queue = int(rgb_cfg.get("sync_queue", 10))
        self._processing_rate_hz = float(rgb_cfg.get("processing_rate_hz", 10.0))
        self._processing_gate = MonotonicRateGate(self._processing_rate_hz)

        # Viz config.
        ann_cfg = viz_cfg.get("annotated_image", {})
        self._layout = str(ann_cfg.get("layout", viz_cfg.get("layout", "side_by_side")))
        self._max_width_px = viz_cfg.get("max_width_px")
        self._max_height_px = viz_cfg.get("max_height_px")
        self._output_width = viz_cfg.get("output_width_px")

        dash_cfg = viz_cfg.get("dashboard", {})
        self._dashboard_enabled = bool(dash_cfg.get("enabled", True))
        self._dashboard_fps = float(dash_cfg.get("rate_hz", 2.0))

        self._zoom_cfg = viz_cfg.get("zoom_roi", {})
        self._zoom_roi_fps = float(self._zoom_cfg.get("rate_hz", 5.0))

        # Label config.
        label_cfg = viz_cfg.get("label", viz_cfg.get("font", {}))
        self._font_cfg = {
            "scale": float(label_cfg.get("font_scale", 0.75)),
            "thickness": int(label_cfg.get("font_thickness", 2)),
            "bbox_thickness": int(label_cfg.get("bbox_thickness", 3)),
            "padding_px": int(label_cfg.get("padding_px", 5)),
            "line_spacing_px": int(label_cfg.get("line_spacing_px", 14)),
            "background_alpha": float(label_cfg.get("background_alpha", 0.85)),
        }
        self._distance_unit = str(label_cfg.get("distance_unit", "mm"))
        self._score_decimals = int(label_cfg.get("score_decimals", 2))
        self._xyz_decimals = int(label_cfg.get("decimals_xyz", 3))

        debug_mask_cfg = viz_cfg.get("debug_masks", {})
        self._debug_masks_enabled = bool(debug_mask_cfg.get("enabled", True))

        color_topic = str(rgb_cfg.get("color_topic", "/camera/color/image_raw"))
        depth_topic = str(rgb_cfg.get("depth_topic",
                                       "/camera/aligned_depth_to_color/image_raw"))
        info_topic = str(rgb_cfg.get("camera_info_topic",
                                      "/camera/color/camera_info"))

        # ---- ROS plumbing ----
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_tf = None

        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, qos)

        from message_filters import ApproximateTimeSynchronizer, Subscriber
        self._color_sub = Subscriber(self, Image, color_topic, qos_profile=qos)
        self._depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)
        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=self._sync_queue, slop=self._sync_slop,
        )
        self._sync.registerCallback(self._on_synced_rgbd)

        # Publishers.
        self._det_pub = self.create_publisher(Detection3DArray, "/perception/detections", 10)
        self._annotated_pub = self.create_publisher(Image, "/perception/annotated_image", 10)

        qos_cfg = viz_cfg.get("qos", {}).get("debug_images", {})
        rel_str = qos_cfg.get("reliability", "reliable").lower()
        debug_rel = ReliabilityPolicy.RELIABLE if rel_str == "reliable" else ReliabilityPolicy.BEST_EFFORT
        debug_qos = QoSProfile(
            reliability=debug_rel, history=HistoryPolicy.KEEP_LAST,
            depth=int(qos_cfg.get("depth", 1)), durability=DurabilityPolicy.VOLATILE,
        )
        self._dashboard_pub = self.create_publisher(Image, "/perception/debug_dashboard_image", debug_qos)
        self._zoom_pub = self.create_publisher(Image, "/perception/zoom_roi_image", debug_qos)
        self._preprocess_pub = self.create_publisher(Image, "/perception/preprocessing_debug", debug_qos)
        self._blue_border_mask_pub = self.create_publisher(Image, "/perception/debug_mask/blue_border", debug_qos)
        self._white_wp_mask_pub = self.create_publisher(Image, "/perception/debug_mask/white_workpiece", debug_qos)

        self._last_dashboard_time = time.time()
        self._last_zoom_time = time.time()

        # ---- UI state ----
        self._active_tab: int = TAB_DETECTION
        self._tab_bar = _TabBar()
        # Canny params.
        self._canny_low: int = 50
        self._canny_high: int = 150
        # Colour Mask params.
        self._hsv_h_min: int = 10
        self._hsv_h_max: int = 170
        self._hsv_s_min: int = 80
        self._hsv_v_min: int = 60
        self._hsv_wraps: bool = True
        self._color_preset_idx: int = 0  # Red
        # Blur / CLAHE / Resize / Threshold / Contours params.
        self._blur_kernel: int = 5
        self._clahe_clip: float = 2.0
        self._resize_scale: int = 50   # percent
        self._thresh_val: int = 0      # 0 = Otsu auto
        self._min_contour_area: int = 500

        # Annotation state.
        self._selected_idx: int | None = None
        self._current_objects: list[dict] = []
        self._dashboard_data: list[dict] = []
        self._notes: dict[int, str] = {}
        self._note_mode: bool = False
        self._note_buffer: str = ""
        self._draw_roi: bool = False
        self._roi_start: tuple | None = None
        self._roi_end: tuple | None = None
        self._rois: list[tuple] = []
        self._content_offset_y: int = _TAB_H
        self._bbox_filter_on: bool = True
        self._ctrl_buttons: list[tuple] = []  # (x1,y1,x2,y2, attr, delta)
        self._ctrl_value_boxes: list[tuple] = []  # (x1,y1,x2,y2, attr, label)
        self._editing_param: str | None = None  # attr being keyboard-edited
        self._edit_buffer: str = ""
        
        # FPS Tracking
        self._fps_queue: collections.deque = collections.deque(maxlen=30)
        self._last_frame_time: float = time.time()

        # ---- OpenCV window (no trackbars — saves vertical space) ----
        cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(_WINDOW_NAME, 1280, 720)
        cv2.setMouseCallback(_WINDOW_NAME, self._on_mouse)

        self._param_client = self.create_client(
            SetParameters, "/scene_processor/set_parameters"
        )

        self.get_logger().info(
            f"UnifiedVisualizer started — {len(self._color_classes)} color classes, "
            f"window: {_WINDOW_NAME}  Keys: b=BBox n=Note r=ROI c=Clear s=Save"
        )

    # ── BBox filter toggle (via keyboard 'b') ───────────────────────
    def _toggle_bbox_filter(self) -> None:
        self._bbox_filter_on = not self._bbox_filter_on
        if not self._param_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("scene_processor param service not available")
            return
        req = SetParameters.Request()
        p = ParameterMsg()
        p.name = "enable_bbox_filter"
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=self._bbox_filter_on)
        req.parameters = [p]
        self._param_client.call_async(req)
        self.get_logger().info(f"BBox filter {'ON' if self._bbox_filter_on else 'OFF'}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = msg.k
        self._fx, self._fy = float(k[0]), float(k[4])
        self._cx, self._cy = float(k[2]), float(k[5])

    # ── Mouse callback ──────────────────────────────────────────────
    def _on_mouse(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            hit_value_box = False
            if self._ctrl_value_boxes:
                for bx1, by1, bx2, by2, attr, label in self._ctrl_value_boxes:
                    if bx1 <= x <= bx2 and by1 <= y <= by2:
                        self._editing_param = attr
                        if attr == "_clahe_clip_x10":
                            self._edit_buffer = str(int(self._clahe_clip * 10))
                        else:
                            self._edit_buffer = str(getattr(self, attr, 0))
                        hit_value_box = True
                        break
            if hit_value_box:
                return
            else:
                self._editing_param = None

        # 1. Tab bar hit test.
        if event == cv2.EVENT_LBUTTONDOWN and y < _TAB_H:
            hit = self._tab_bar.hit_test(x, y)
            if hit is not None:
                self._active_tab = hit
                self._selected_idx = None
                self._note_mode = False
                self.get_logger().info(f"Tab → {dict(_TAB_LABELS).get(hit, '?')}")
            return
        if event == cv2.EVENT_MOUSEMOVE and y < _TAB_H:
            self._tab_bar.set_hover(x, y)
            return

        # 2. Control bar button clicks.
        if event == cv2.EVENT_LBUTTONDOWN and self._ctrl_buttons:
            for bx1, by1, bx2, by2, attr, delta in self._ctrl_buttons:
                if bx1 <= x <= bx2 and by1 <= y <= by2:
                    self._apply_ctrl_button(attr, delta)
                    return

        # 4. ROI draw mode.
        if self._draw_roi and self._active_tab == TAB_DETECTION:
            cy = y - self._content_offset_y
            if event == cv2.EVENT_LBUTTONDOWN:
                self._roi_start = (x, cy)
                self._roi_end = (x, cy)
            elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON):
                self._roi_end = (x, cy)
            elif event == cv2.EVENT_LBUTTONUP and self._roi_start is not None:
                self._rois.append((self._roi_start, (x, cy)))
                self._roi_start = None
                self._roi_end = None
                self._draw_roi = False
                self.get_logger().info(f"ROI added ({len(self._rois)} total)")
            return

        # 4. Detection bbox click (Detection tab only).
        if event == cv2.EVENT_LBUTTONDOWN and self._active_tab == TAB_DETECTION:
            cy = y - self._content_offset_y
            hit_idx = self._find_detection_at(x, cy)
            if hit_idx is not None:
                self._selected_idx = hit_idx
                self._note_mode = False
            else:
                self._selected_idx = None
                self._note_mode = False
        elif event == cv2.EVENT_MOUSEMOVE:
            self._tab_bar.set_hover(x, y)


    # ── Annotation helpers ──────────────────────────────────────────
    def _find_detection_at(self, x: int, y: int) -> int | None:
        """Return index of detection whose bbox contains (x, y), or None."""
        for i, obj in enumerate(self._current_objects):
            bx, by, bw, bh = obj["bbox"]
            if bx <= x <= bx + bw and by <= y <= by + bh:
                return i
        return None

    def _render_popup(self, image: np.ndarray, obj: dict, idx: int) -> None:
        """Draw floating metadata popup near the selected detection."""
        bx, by, bw, bh = obj["bbox"]
        d = self._xyz_decimals
        sd = self._score_decimals
        lines = [
            f"Class: {obj.get('class_id', '?')}",
            f"Conf:  {obj.get('confidence', 0):.{sd}f}",
            f"Dist:  {obj.get('dist_str', 'N/A')}",
        ]
        cam = obj.get("cam_xyz")
        if cam:
            lines.append(f"Cam:   ({cam[0]:.{d}f},{cam[1]:.{d}f},{cam[2]:.{d}f})")
        base = obj.get("base_xyz_str", "")
        if base and base != "TF_UNAVAILABLE":
            lines.append(f"Base:  {base}")
        lines.append(f"BBox:  [{bx},{by},{bw},{bh}]")
        sm = obj.get("shape_metrics", {})
        if sm:
            lines.append(f"Circ:  {sm.get('circularity', 0):.2f}  Sol: {sm.get('solidity', 0):.2f}")
        note = self._notes.get(idx)
        if note:
            lines.append(f"Note:  {note}")
        if self._note_mode and self._selected_idx == idx:
            lines.append(f"> {self._note_buffer}_")

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale, thick = 0.40, 1
        pad = 8
        max_tw = 0
        line_h_list = []
        for ln in lines:
            (tw, th), _ = cv2.getTextSize(ln, font, scale, thick)
            max_tw = max(max_tw, tw)
            line_h_list.append(th)
        total_h = sum(line_h_list) + (len(lines) - 1) * 6 + 2 * pad
        total_w = max_tw + 2 * pad
        img_h, img_w = image.shape[:2]

        # Position: right of bbox, clamp to image bounds.
        px = min(bx + bw + 10, img_w - total_w - 5)
        py = max(5, by)
        if px < 0:
            px = max(5, bx - total_w - 10)
        if py + total_h > img_h:
            py = img_h - total_h - 5

        # White popup with blue border.
        cv2.rectangle(image, (px, py), (px + total_w, py + total_h), _POPUP_BG, -1)
        cv2.rectangle(image, (px, py), (px + total_w, py + total_h), _POPUP_BORDER, 2)
        cy = py + pad + line_h_list[0]
        for i, ln in enumerate(lines):
            cv2.putText(image, ln, (px + pad, cy), font, scale, _TEXT_DARK, thick, cv2.LINE_AA)
            if i + 1 < len(lines):
                cy += line_h_list[i + 1] + 6

    def _handle_snapshot(self, display: np.ndarray) -> None:
        """Save current frame + detection data to /tmp."""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = f"/tmp/gp4_snapshot_{ts}"
        os.makedirs(out_dir, exist_ok=True)
        cv2.imwrite(os.path.join(out_dir, "frame.png"), display)
        data = {
            "timestamp": ts,
            "detections": [],
            "notes": {str(k): v for k, v in self._notes.items()},
            "rois": [{"start": list(s), "end": list(e)} for s, e in self._rois],
        }
        for i, obj in enumerate(self._current_objects):
            det = {
                "idx": i,
                "class_id": obj.get("class_id", ""),
                "confidence": float(obj.get("confidence", 0)),
                "bbox": list(obj.get("bbox", ())),
                "dist_str": obj.get("dist_str", ""),
            }
            if obj.get("cam_xyz"):
                det["cam_xyz"] = list(obj["cam_xyz"])
            det["base_xyz_str"] = obj.get("base_xyz_str", "")
            data["detections"].append(det)
        json_path = os.path.join(out_dir, "detections.json")
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f"Snapshot saved → {out_dir}")

    # ── Preprocessing pipeline ──────────────────────────────────────
    def _compute_stages(self, bgr: np.ndarray) -> dict[int, np.ndarray]:
        """Run all pre-processing stages and return {tab_mode: image}."""
        stages: dict[int, np.ndarray] = {}
        stages[TAB_ORIGINAL] = _put_label(bgr, "Original")

        # Blur — tunable kernel size.
        k = self._blur_kernel
        blurred = cv2.GaussianBlur(bgr, (k, k), 0)
        stages[TAB_BLUR] = _put_label(blurred, f"Gaussian Blur ({k}x{k})")

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        stages[TAB_GRAY] = _put_label(_ensure_bgr(gray), "Grayscale")

        # CLAHE — tunable clip limit.
        clahe = cv2.createCLAHE(clipLimit=self._clahe_clip, tileGridSize=(8, 8))
        clahe_img = clahe.apply(gray)
        stages[TAB_CLAHE] = _put_label(_ensure_bgr(clahe_img),
                                       f"CLAHE (clip={self._clahe_clip:.1f})")

        # Resize — tunable scale percentage.
        sc = max(10, self._resize_scale)
        rw = max(16, int(bgr.shape[1] * sc / 100))
        rh = max(16, int(bgr.shape[0] * sc / 100))
        resized = cv2.resize(bgr, (rw, rh), interpolation=cv2.INTER_AREA)
        resized_show = cv2.resize(resized, (bgr.shape[1], bgr.shape[0]),
                                  interpolation=cv2.INTER_NEAREST)
        stages[TAB_RESIZED] = _put_label(resized_show, f"Resize {sc}% ({rw}x{rh})")

        # Threshold — tunable (0 = Otsu auto).
        if self._thresh_val <= 0:
            _, binary = cv2.threshold(clahe_img, 0, 255,
                                      cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            t_label = "Threshold (Otsu auto)"
        else:
            _, binary = cv2.threshold(clahe_img, self._thresh_val, 255,
                                      cv2.THRESH_BINARY)
            t_label = f"Threshold ({self._thresh_val})"
        stages[TAB_THRESHOLD] = _put_label(_ensure_bgr(binary), t_label)

        blurred_gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(blurred_gray, self._canny_low, self._canny_high)
        stages[TAB_CANNY] = _put_label(
            _ensure_bgr(canny), f"Canny ({self._canny_low}/{self._canny_high})")

        # Contours — tunable min area.
        contour_vis = bgr.copy()
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours
                 if cv2.contourArea(c) > self._min_contour_area]
        cv2.drawContours(contour_vis, valid, -1, (0, 255, 0), 2)
        for cnt in valid:
            rx, ry, rw2, rh2 = cv2.boundingRect(cnt)
            cv2.rectangle(contour_vis, (rx, ry), (rx + rw2, ry + rh2),
                          (255, 0, 255), 2)
        stages[TAB_CONTOURS] = _put_label(
            contour_vis, f"Contours ({len(valid)}, area>{self._min_contour_area})")

        # Colour Mask — supports wrapping (red) and single-range hues.
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h_min, h_max = self._hsv_h_min, self._hsv_h_max
        s_min, v_min = self._hsv_s_min, self._hsv_v_min
        if self._hsv_wraps:
            m_lo = cv2.inRange(hsv, np.array([0, s_min, v_min]),
                               np.array([h_min, 255, 255]))
            m_hi = cv2.inRange(hsv, np.array([h_max, s_min, v_min]),
                               np.array([180, 255, 255]))
            colour_mask = cv2.bitwise_or(m_lo, m_hi)
            h_str = f"H:[0-{h_min}]+[{h_max}-180]"
        else:
            colour_mask = cv2.inRange(hsv, np.array([h_min, s_min, v_min]),
                                     np.array([h_max, 255, 255]))
            h_str = f"H:[{h_min}-{h_max}]"
        morph_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_CLOSE, morph_k)
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_OPEN, morph_k)
        overlay = bgr.copy()
        overlay[colour_mask > 0] = (255, 255, 255)
        n_px = int(np.count_nonzero(colour_mask))
        total = colour_mask.shape[0] * colour_mask.shape[1]
        pct = 100.0 * n_px / total if total > 0 else 0.0
        preset_name = _COLOR_PRESETS[self._color_preset_idx][0]
        stages[TAB_COLOUR_MASK] = _put_label(
            overlay,
            f"{preset_name} Mask {h_str} S>={s_min} V>={v_min} ({pct:.1f}%)")
        return stages

    def _build_tiled(self, stages: dict[int, np.ndarray]) -> np.ndarray:
        """3x3 grid of 9 preprocessing stages."""
        ref = stages[TAB_ORIGINAL]
        tile_h, tile_w = ref.shape[0] // 3, ref.shape[1] // 3
        tiles = []
        for mode in range(TAB_ORIGINAL, TAB_COLOUR_MASK + 1):
            img = stages.get(mode, np.zeros((tile_h, tile_w, 3), dtype=np.uint8))
            tiles.append(cv2.resize(img, (tile_w, tile_h), interpolation=cv2.INTER_AREA))
        row1 = np.hstack(tiles[0:3])
        row2 = np.hstack(tiles[3:6])
        row3 = np.hstack(tiles[6:9])
        return np.vstack([row1, row2, row3])

    def _publish_debug_masks(self, objects: list[dict], header) -> None:
        for obj in objects:
            if obj.get("source") != "blue_border_white_fill":
                continue
            blue_mask = obj.get("_blue_mask")
            if blue_mask is not None:
                try:
                    m = self._bridge.cv2_to_imgmsg(blue_mask, encoding="mono8")
                    m.header = header
                    self._blue_border_mask_pub.publish(m)
                except Exception:
                    pass
            wp_mask = obj.get("_white_mask")
            if wp_mask is not None:
                try:
                    m = self._bridge.cv2_to_imgmsg(wp_mask, encoding="mono8")
                    m.header = header
                    self._white_wp_mask_pub.publish(m)
                except Exception:
                    pass

    # ── Clickable control bar widget ────────────────────────────────
    def _render_ctrl_bar(
        self, width: int, content_h: int,
        params: list[tuple[str, str, int, int]],
    ) -> tuple[np.ndarray, list[tuple]]:
        """Render a control bar with [-] [value] [+] buttons.

        Args:
            width: bar width in pixels
            content_h: height of content above bar (for y-offset calc)
            params: [(label, attr_name, current_value, step), ...]
        Returns:
            (bar_image, buttons) where buttons are
            [(x1,y1,x2,y2, attr_name, delta), ...] in display coords.
        """
        bar_h = 44
        bar = np.full((bar_h, width, 3), _PANEL_BG, dtype=np.uint8)
        cv2.line(bar, (0, 0), (width, 0), _PANEL_BORDER, 1)
        buttons: list[tuple] = []
        value_boxes: list[tuple] = []
        n = len(params)
        section_w = width // max(n, 1)
        btn_w = 32
        btn_h = 28
        val_w = 90
        by = 8
        font = cv2.FONT_HERSHEY_SIMPLEX

        # y offset in full display: _TAB_H + content_h + by
        y_base = _TAB_H + content_h

        for i, (label, attr, val, step) in enumerate(params):
            sx = i * section_w + 8

            # [-] button
            bx = sx
            cv2.rectangle(bar, (bx, by), (bx + btn_w, by + btn_h), _ACCENT_BLUE, -1)
            cv2.rectangle(bar, (bx, by), (bx + btn_w, by + btn_h), _PANEL_BORDER, 1)
            cv2.putText(bar, "-", (bx + 10, by + 20), font, 0.6,
                        (255, 255, 255), 2, cv2.LINE_AA)
            buttons.append((bx, y_base + by, bx + btn_w, y_base + by + btn_h, attr, -step))

            # [value] box — clickable for direct edit
            vx = bx + btn_w + 4
            is_editing = (self._editing_param == attr)
            box_bg = (220, 240, 255) if is_editing else (255, 255, 255)
            box_border = _ACCENT_BLUE if is_editing else _PANEL_BORDER
            cv2.rectangle(bar, (vx, by), (vx + val_w, by + btn_h), box_bg, -1)
            cv2.rectangle(bar, (vx, by), (vx + val_w, by + btn_h), box_border, 2 if is_editing else 1)
            if is_editing:
                txt = f"{self._edit_buffer}_"
            else:
                txt = f"{label}:{val}"
            (tw, _), _ = cv2.getTextSize(txt, font, 0.50, 1)
            tx = vx + (val_w - tw) // 2
            cv2.putText(bar, txt, (tx, by + 20), font, 0.50,
                        _ACCENT_BLUE if is_editing else _TEXT_DARK, 1, cv2.LINE_AA)
            value_boxes.append((vx, y_base + by, vx + val_w, y_base + by + btn_h, attr, label))

            # [+] button
            px = vx + val_w + 4
            cv2.rectangle(bar, (px, by), (px + btn_w, by + btn_h), _ACCENT_BLUE, -1)
            cv2.rectangle(bar, (px, by), (px + btn_w, by + btn_h), _PANEL_BORDER, 1)
            cv2.putText(bar, "+", (px + 8, by + 20), font, 0.6,
                        (255, 255, 255), 2, cv2.LINE_AA)
            buttons.append((px, y_base + by, px + btn_w, y_base + by + btn_h, attr, step))

        return bar, buttons, value_boxes

    def _render_color_presets(
        self, width: int, content_h: int,
    ) -> tuple[np.ndarray, list[tuple]]:
        """Render a row of clickable color preset squares.

        Returns (bar_image, buttons) where buttons use special
        attr='__preset_N' for preset index N.
        """
        bar_h = 40
        bar = np.full((bar_h, width, 3), _PANEL_BG, dtype=np.uint8)
        cv2.line(bar, (0, 0), (width, 0), _PANEL_BORDER, 1)
        buttons: list[tuple] = []
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_base = _TAB_H + content_h
        n = len(_COLOR_PRESETS)
        sq_w = 80
        sq_h = 26
        gap = 8
        total_w = n * (sq_w + gap)
        sx = max(8, (width - total_w) // 2)
        by = 7

        for i, (name, bgr_col, *_rest) in enumerate(_COLOR_PRESETS):
            x1 = sx + i * (sq_w + gap)
            is_active = (i == self._color_preset_idx)
            # Draw colored square.
            cv2.rectangle(bar, (x1, by), (x1 + sq_w, by + sq_h),
                          bgr_col, -1)
            border = _ACCENT_BLUE if is_active else _PANEL_BORDER
            thick = 3 if is_active else 1
            cv2.rectangle(bar, (x1, by), (x1 + sq_w, by + sq_h),
                          border, thick)
            # Label.
            (tw, _), _ = cv2.getTextSize(name, font, 0.40, 1)
            tx = x1 + (sq_w - tw) // 2
            # Choose text color for contrast.
            brightness = int(bgr_col[0]) * 0.1 + int(bgr_col[1]) * 0.6 + int(bgr_col[2]) * 0.3
            txt_col = (0, 0, 0) if brightness > 128 else (255, 255, 255)
            cv2.putText(bar, name, (tx, by + 18), font, 0.40,
                        txt_col, 1, cv2.LINE_AA)
            # Store as special preset button.
            buttons.append((x1, y_base + by, x1 + sq_w, y_base + by + sq_h,
                           f"__preset_{i}", 0))
        return bar, buttons

    def _apply_ctrl_button(self, attr: str, delta: int) -> None:
        """Handle a control bar button click with special-case logic."""
        # Color preset selection.
        if attr.startswith("__preset_"):
            idx = int(attr.split("_")[-1])
            if 0 <= idx < len(_COLOR_PRESETS):
                self._color_preset_idx = idx
                _, _, h_min, h_max, s_min, v_min, wraps = _COLOR_PRESETS[idx]
                self._hsv_h_min = h_min
                self._hsv_h_max = h_max
                self._hsv_s_min = s_min
                self._hsv_v_min = v_min
                self._hsv_wraps = wraps
            return
        # CLAHE clip: virtual attr _clahe_clip_x10 → real float _clahe_clip.
        if attr == "_clahe_clip_x10":
            new_val = max(5, min(100, int(self._clahe_clip * 10) + delta))
            self._clahe_clip = new_val / 10.0
            return
        # Normal int attribute.
        cur = getattr(self, attr, 0)
        mx = 180 if 'hsv_h_' in attr else 255
        if attr == '_blur_kernel':
            mx = 21
        elif attr == '_resize_scale':
            mx = 200
        elif attr == '_min_contour_area':
            mx = 10000
        new_val = max(0, min(mx, cur + delta))
        # Blur kernel must be odd and >= 3.
        if attr == '_blur_kernel':
            new_val = max(3, new_val)
            if new_val % 2 == 0:
                new_val += (1 if delta > 0 else -1)
                new_val = max(3, min(21, new_val))
        setattr(self, attr, new_val)

    # ── Main RGBD callback ──────────────────────────────────────────
    def _on_synced_rgbd(self, color_msg: Image, depth_msg: Image) -> None:
        if not self._processing_gate.allow(time.monotonic()):
            return
        try:
            rgb = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge color failed: {exc}")
            return
        try:
            depth_raw = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge depth failed: {exc}")
            return

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # ── Always run detection pipeline (publishes regardless of tab) ──
        objects: list[dict] = []
        overlay_rgb = rgb.copy()
        n_published = 0
        dashboard_data: list[dict] = []
        enriched_objects: list[dict] = []

        if self._fx is not None:
            objects = detect_color_objects(rgb, self._color_classes, self._postprocess_cfg)
            # Ensure distance_m field exists before tracker (not yet deprojected).
            for _obj in objects:
                _obj.setdefault("distance_m", 0.0)
            if self._visual_tracking_enabled:
                objects = self._visual_tracker.update(objects)

            # TF lookup.
            tf_cam_to_base = None
            try:
                tf_cam_to_base = self._tf_buffer.lookup_transform(
                    self._base_frame, self._camera_frame, Time(),
                    timeout=Duration(seconds=0.0))
                self._last_tf = tf_cam_to_base
            except Exception:
                if self._last_tf is not None:
                    tf_cam_to_base = self._last_tf

            from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
            from geometry_msgs.msg import PoseWithCovariance
            from vision_msgs.msg import Detection3D, ObjectHypothesis, ObjectHypothesisWithPose
            from std_msgs.msg import Header

            det_arr = Detection3DArray()

            for idx, obj in enumerate(objects):
                held_for_display = bool(obj.get("held_for_display", False))
                obj_id = f"#{idx + 1}"
                class_id = obj["class_id"]
                x, y, w, h = obj["bbox"]
                mask = obj["mask"]
                cx_px, cy_px = obj["center_uv"]
                confidence = obj["confidence"]

                depth_roi = depth_raw[y:y + h, x:x + w]
                mask_roi = mask[y:y + h, x:x + w]
                z_m = _median_depth_m(depth_roi, mask_roi, depth_scale=self._depth_scale)

                color_bgr = _color_for_class(class_id)
                bbox_thick = int(self._font_cfg.get("bbox_thickness", 3))
                cv2.rectangle(overlay_rgb, (x, y), (x + w, y + h), color_bgr, bbox_thick)

                if z_m is None or z_m <= 0.0:
                    lines = [f"{obj_id} {class_id}", "XYZ_INVALID"]
                    _draw_detection_callout(overlay_rgb, (x, y, w, h), lines,
                                           color_bgr, self._font_cfg,
                                           tag_fallback=True, tag_text=obj_id)
                    enriched_objects.append({
                        **obj, "obj_id": obj_id, "distance_m": None,
                        "dist_str": "N/A", "cam_xyz": None, "base_xyz_str": "N/A",
                    })
                    continue

                cam_x, cam_y, cam_z = _deproject_pixel(
                    float(cx_px), float(cy_px), z_m,
                    self._fx, self._fy, self._cx, self._cy)
                distance_m = z_m

                base_xyz_str = "TF_UNAVAILABLE"
                frame_id = self._camera_frame
                det_x, det_y, det_z = cam_x, cam_y, cam_z

                if tf_cam_to_base is not None:
                    try:
                        from scipy.spatial.transform import Rotation
                        t = tf_cam_to_base.transform.translation
                        r = tf_cam_to_base.transform.rotation
                        rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
                        base_pt = rot.apply(np.array([cam_x, cam_y, cam_z])) + \
                                  np.array([t.x, t.y, t.z])
                        det_x, det_y, det_z = float(base_pt[0]), float(base_pt[1]), float(base_pt[2])
                        frame_id = self._base_frame
                        dd = self._xyz_decimals
                        base_xyz_str = f"({det_x:.{dd}f},{det_y:.{dd}f},{det_z:.{dd}f})"
                    except Exception:
                        pass

                det = Detection3D()
                det.header = Header(stamp=color_msg.header.stamp, frame_id=frame_id)
                pose = Pose()
                pose.position = Point(x=det_x, y=det_y, z=det_z)
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                hyp = ObjectHypothesis()
                hyp.class_id = class_id
                hyp.score = float(confidence)
                det.results.append(ObjectHypothesisWithPose(
                    hypothesis=hyp, pose=PoseWithCovariance(pose=pose)))
                sx, sy = _bbox_size_m(float(w), float(h), z_m, self._fx, self._fy)
                det.bbox.size = Vector3(x=sx, y=sy, z=self._bbox_z)
                if not held_for_display:
                    det_arr.detections.append(det)
                    n_published += 1

                dist_str = _format_distance(distance_m, self._distance_unit)
                sd = self._score_decimals
                lines = [f"{obj_id} {class_id} {confidence:.{sd}f} {dist_str}"]
                if held_for_display:
                    lines.append("HELD_DISPLAY_ONLY")
                _draw_detection_callout(overlay_rgb, (x, y, w, h), lines,
                                       color_bgr, self._font_cfg)

                dd = self._xyz_decimals
                dashboard_data.append({
                    "id": obj_id, "class": class_id,
                    "conf": f"{confidence:.{sd}f}", "dist": dist_str,
                    "tf": base_xyz_str,
                    "cam_xyz": f"({cam_x:.{dd}f},{cam_y:.{dd}f},{cam_z:.{dd}f})",
                    "bbox_px": f"[{x},{y},{w},{h}]",
                })
                enriched_objects.append({
                    **obj, "obj_id": obj_id, "distance_m": distance_m,
                    "dist_str": dist_str, "cam_xyz": (cam_x, cam_y, cam_z),
                    "base_xyz_str": base_xyz_str,
                })

            det_arr.header = Header(
                stamp=color_msg.header.stamp,
                frame_id=self._base_frame if tf_cam_to_base else self._camera_frame)
            self._det_pub.publish(det_arr)

            if self._debug_masks_enabled:
                self._publish_debug_masks(objects, color_msg.header)

        self._current_objects = enriched_objects
        self._dashboard_data = dashboard_data

        # ── Render based on active tab ──────────────────────────────
        if self._active_tab == TAB_DETECTION:
            content = self._render_detection_content(
                overlay_rgb, depth_raw, dashboard_data, n_published, color_msg)
        elif self._active_tab >= TAB_ALL:
            stages = self._compute_stages(bgr)
            if self._active_tab == TAB_ALL:
                content = self._build_tiled(stages)
            else:
                content = stages.get(self._active_tab, stages[TAB_ORIGINAL])
            # Publish preprocessing debug.
            try:
                dbg = self._bridge.cv2_to_imgmsg(content, encoding="bgr8")
                dbg.header = color_msg.header
                self._preprocess_pub.publish(dbg)
            except Exception:
                pass
        else:
            content = bgr

        # ── Render clickable control bars for tunable tabs ───────────
        self._ctrl_buttons = []
        self._ctrl_value_boxes = []
        extra_bars: list[np.ndarray] = []
        content_h = content.shape[0]
        w = content.shape[1]

        if self._active_tab == TAB_BLUR:
            bar, btns, vboxes = self._render_ctrl_bar(w, content_h,
                [("Kernel", "_blur_kernel", self._blur_kernel, 2)])
            extra_bars.append(bar)
            self._ctrl_buttons.extend(btns)
            self._ctrl_value_boxes.extend(vboxes)
        elif self._active_tab == TAB_CLAHE:
            # Clip stored as float but control bar expects int display
            clip_x10 = int(self._clahe_clip * 10)
            bar, btns, vboxes = self._render_ctrl_bar(w, content_h,
                [("Clipx10", "_clahe_clip_x10", clip_x10, 5)])
            extra_bars.append(bar)
            self._ctrl_buttons.extend(btns)
            self._ctrl_value_boxes.extend(vboxes)
        elif self._active_tab == TAB_RESIZED:
            bar, btns, vboxes = self._render_ctrl_bar(w, content_h,
                [("Scale%", "_resize_scale", self._resize_scale, 10)])
            extra_bars.append(bar)
            self._ctrl_buttons.extend(btns)
            self._ctrl_value_boxes.extend(vboxes)
        elif self._active_tab == TAB_THRESHOLD:
            bar, btns, vboxes = self._render_ctrl_bar(w, content_h,
                [("Thresh", "_thresh_val", self._thresh_val, 5)])
            extra_bars.append(bar)
            self._ctrl_buttons.extend(btns)
            self._ctrl_value_boxes.extend(vboxes)
        elif self._active_tab == TAB_CANNY:
            bar, btns, vboxes = self._render_ctrl_bar(w, content_h,
                [("Low", "_canny_low", self._canny_low, 5),
                 ("High", "_canny_high", self._canny_high, 5)])
            extra_bars.append(bar)
            self._ctrl_buttons.extend(btns)
            self._ctrl_value_boxes.extend(vboxes)
        elif self._active_tab == TAB_CONTOURS:
            bar, btns, vboxes = self._render_ctrl_bar(w, content_h,
                [("MinArea", "_min_contour_area", self._min_contour_area, 100)])
            extra_bars.append(bar)
            self._ctrl_buttons.extend(btns)
            self._ctrl_value_boxes.extend(vboxes)
        elif self._active_tab == TAB_COLOUR_MASK:
            # Color preset row.
            preset_bar, preset_btns = self._render_color_presets(w, content_h)
            extra_bars.append(preset_bar)
            self._ctrl_buttons.extend(preset_btns)
            # HSV params row.
            content_h2 = content_h + preset_bar.shape[0]
            bar, btns, vboxes = self._render_ctrl_bar(w, content_h2,
                [("H_Min", "_hsv_h_min", self._hsv_h_min, 2),
                 ("H_Max", "_hsv_h_max", self._hsv_h_max, 2),
                 ("S_Min", "_hsv_s_min", self._hsv_s_min, 5),
                 ("V_Min", "_hsv_v_min", self._hsv_v_min, 5)])
            extra_bars.append(bar)
            self._ctrl_buttons.extend(btns)
            self._ctrl_value_boxes.extend(vboxes)

        for eb in extra_bars:
            content = np.vstack([content, eb])

        # ── Compose display ─────────────────────────────────────────
        tab_bar_img = self._tab_bar.render(content.shape[1], self._active_tab)
        display = np.vstack([tab_bar_img, content])
        self._content_offset_y = _TAB_H

        # Calculate and draw FPS inside the image (top right)
        now = time.time()
        dt = now - self._last_frame_time
        self._last_frame_time = now
        if dt > 0:
            self._fps_queue.append(1.0 / dt)
        if self._fps_queue:
            fps_avg = sum(self._fps_queue) / len(self._fps_queue)
            fps_str = f"FPS: {fps_avg:.1f}"
            dw = display.shape[1]
            bx = dw - 95
            by = _TAB_H + 5
            cv2.rectangle(display, (bx, by), (bx + 85, by + 26), (0, 0, 0), -1)
            cv2.putText(display, fps_str, (bx + 8, by + 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow(_WINDOW_NAME, display)
        key = cv2.waitKey(1) & 0xFF
        self._handle_keys(key, display)

        # ── Rate-limited publishers ─────────────────────────────────
        now = time.time()
        if self._zoom_cfg.get("enabled", False) and enriched_objects:
            if (now - self._last_zoom_time) >= (1.0 / self._zoom_roi_fps):
                self._last_zoom_time = now
                try:
                    zoom_img = _build_zoom_grid(rgb, enriched_objects,
                                                self._zoom_cfg, self._font_cfg)
                    if zoom_img is not None and zoom_img.size > 0:
                        z_bgr = cv2.cvtColor(zoom_img, cv2.COLOR_RGB2BGR)
                        zm = self._bridge.cv2_to_imgmsg(z_bgr, encoding="bgr8")
                        zm.header = color_msg.header
                        self._zoom_pub.publish(zm)
                except Exception:
                    pass

        if self._dashboard_enabled and (now - self._last_dashboard_time) >= (1.0 / self._dashboard_fps):
            self._last_dashboard_time = now
            try:
                overlay_bgr = cv2.cvtColor(overlay_rgb, cv2.COLOR_RGB2BGR)
                depth_vis = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_c = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                if depth_c.shape[0] != overlay_bgr.shape[0]:
                    sf = overlay_bgr.shape[0] / depth_c.shape[0]
                    depth_c = cv2.resize(depth_c, (int(depth_c.shape[1] * sf), overlay_bgr.shape[0]))
                top_row = np.hstack([overlay_bgr, depth_c])
                req_h = 130 + len(dashboard_data) * 40 + 20
                panel_h = max(req_h, top_row.shape[0] // 2)
                panel = np.full((panel_h, top_row.shape[1], 3), _PANEL_BG, dtype=np.uint8)
                cv2.putText(panel, "--- Detections Dashboard ---", (15, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, _TEXT_DARK, 2, cv2.LINE_AA)
                cv2.putText(panel, f"Total: {n_published}  TF: {self._base_frame}",
                            (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, _TEXT_MUTED, 2, cv2.LINE_AA)
                y_off = 130
                for row in dashboard_data:
                    if y_off > panel_h - 20:
                        break
                    txt = (f"{row['id']} | {row['class']} | {row['conf']} | "
                           f"{row['dist']} | TF:{row['tf']} | Cam:{row['cam_xyz']}")
                    cv2.putText(panel, txt, (15, y_off), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, _TEXT_DARK, 1, cv2.LINE_AA)
                    y_off += 35
                dash_img = np.vstack([top_row, panel])
                dm = self._bridge.cv2_to_imgmsg(dash_img, encoding="bgr8")
                dm.header = color_msg.header
                self._dashboard_pub.publish(dm)
            except Exception:
                pass


    # ── Detection tab content ───────────────────────────────────────
    def _render_detection_content(
        self, overlay_rgb: np.ndarray, depth_raw: np.ndarray,
        dashboard_data: list[dict], n_published: int, color_msg,
    ) -> np.ndarray:
        """Build Detection tab: RGB+Depth + status bar + inline dashboard."""
        overlay_bgr = cv2.cvtColor(overlay_rgb, cv2.COLOR_RGB2BGR)

        # Depth colormap.
        depth_vis = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        if depth_color.shape[0] != overlay_bgr.shape[0]:
            sf = overlay_bgr.shape[0] / depth_color.shape[0]
            depth_color = cv2.resize(depth_color,
                                     (int(depth_color.shape[1] * sf), overlay_bgr.shape[0]))

        # Draw popup on overlay if selected.
        if self._selected_idx is not None and self._selected_idx < len(self._current_objects):
            self._render_popup(overlay_bgr, self._current_objects[self._selected_idx],
                               self._selected_idx)

        # Draw ROIs.
        for start, end in self._rois:
            _draw_dashed_rect(overlay_bgr, start, end, (0, 255, 255), 2)
        if self._roi_start and self._roi_end:
            _draw_dashed_rect(overlay_bgr, self._roi_start, self._roi_end, (0, 200, 255), 1)

        # Side-by-side.
        layout_mode = self._layout
        if layout_mode == "rgb_only":
            top_row = overlay_bgr
        elif layout_mode == "depth_only":
            top_row = depth_color
        else:
            top_row = np.hstack([overlay_bgr, depth_color])

        # Dynamic resize.
        ch, cw = top_row.shape[:2]
        if self._output_width is not None and self._max_width_px is None:
            self._max_width_px = self._output_width
        scale_f = 1.0
        if self._max_width_px and cw > self._max_width_px:
            scale_f = min(scale_f, self._max_width_px / cw)
        if self._max_height_px and ch > self._max_height_px:
            scale_f = min(scale_f, self._max_height_px / ch)
        if scale_f < 1.0:
            top_row = cv2.resize(top_row, (int(cw * scale_f), int(ch * scale_f)))

        # Publish annotated image.
        try:
            ann_msg = self._bridge.cv2_to_imgmsg(top_row, encoding="bgr8")
            ann_msg.header = color_msg.header
            self._annotated_pub.publish(ann_msg)
        except Exception:
            pass

        # Status bar (light theme).
        bar_w = top_row.shape[1]
        bar_h = 36
        bar = np.full((bar_h, bar_w, 3), _STATUS_BAR_BG, dtype=np.uint8)
        cv2.line(bar, (0, 0), (bar_w, 0), _PANEL_BORDER, 1)
        mode_txt = "[ROI]" if self._draw_roi else ("[NOTE]" if self._note_mode else "")
        bbox_txt = "BBox:ON" if self._bbox_filter_on else "BBox:OFF"
        status = f"Detections: {n_published}  |  {bbox_txt}  |  b:BBox  n:Note  r:ROI  c:Clear  s:Save  {mode_txt}"
        cv2.putText(bar, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, _TEXT_DARK, 1, cv2.LINE_AA)

        # Inline dashboard panel — FIXED height to prevent window flicker.
        _DASH_H = 200
        dash_panel = np.full((_DASH_H, bar_w, 3), _PANEL_BG, dtype=np.uint8)
        cv2.line(dash_panel, (0, 0), (bar_w, 0), _PANEL_BORDER, 1)
        # Header.
        cv2.putText(dash_panel, "ID     Class                Conf     Dist       TF Base",
                    (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, _TEXT_MUTED, 1, cv2.LINE_AA)
        y_off = 58
        for i, row in enumerate(dashboard_data):
            if y_off > _DASH_H - 14:
                break
            is_sel = (self._selected_idx == i)
            if is_sel:
                cv2.rectangle(dash_panel, (0, y_off - 20), (bar_w, y_off + 10),
                              _TAB_ACTIVE_BG, -1)
                tc = _TAB_ACTIVE_TEXT
            else:
                tc = _TEXT_DARK
            txt = f"{row['id']:5s}  {row['class']:20s} {row['conf']:6s}  {row['dist']:8s}  {row['tf']}"
            note = self._notes.get(i)
            if note:
                txt += f"  [{note}]"
            cv2.putText(dash_panel, txt, (12, y_off), cv2.FONT_HERSHEY_SIMPLEX,
                        0.58, tc, 1, cv2.LINE_AA)
            y_off += 34

        return np.vstack([top_row, bar, dash_panel])

    # ── Keyboard handler ────────────────────────────────────────────
    def _handle_keys(self, key: int, display: np.ndarray) -> None:
        if key == 255:
            return

        # Parameter edit mode.
        if self._editing_param:
            if key == 13:  # Enter
                try:
                    val = int(self._edit_buffer)
                    # Use _apply_ctrl_button to enforce bounds, but we need absolute setter here.
                    # We'll set it then call _apply with delta=0 to clamp it.
                    if self._editing_param == "_clahe_clip_x10":
                        self._clahe_clip = val / 10.0
                    else:
                        setattr(self, self._editing_param, val)
                    self._apply_ctrl_button(self._editing_param, 0)
                except ValueError:
                    self.get_logger().warn(f"Invalid input: {self._edit_buffer}")
                self._editing_param = None
                self._edit_buffer = ""
            elif key == 27:  # Esc
                self._editing_param = None
                self._edit_buffer = ""
            elif key == 8:  # Backspace
                self._edit_buffer = self._edit_buffer[:-1]
            elif 48 <= key <= 57 or key == 45:  # 0-9 and minus sign
                self._edit_buffer += chr(key)
            return

        # Note input mode.
        if self._note_mode and self._selected_idx is not None:
            if key == 13:  # Enter
                self._notes[self._selected_idx] = self._note_buffer
                self._note_mode = False
                self._note_buffer = ""
                self.get_logger().info(f"Note saved for detection #{self._selected_idx + 1}")
            elif key == 27:  # Esc
                self._note_mode = False
                self._note_buffer = ""
            elif key == 8:  # Backspace
                self._note_buffer = self._note_buffer[:-1]
            elif 32 <= key < 127:
                self._note_buffer += chr(key)
            return

        if key in (ord("q"), 27):
            if self._selected_idx is not None:
                self._selected_idx = None
                self._note_mode = False
            else:
                self.get_logger().info("Quit requested")
                rclpy.shutdown()
        elif key == ord("n") and self._selected_idx is not None:
            self._note_mode = True
            self._note_buffer = self._notes.get(self._selected_idx, "")
            self.get_logger().info("Note mode ON — type then Enter to save, Esc to cancel")
        elif key == ord("b"):
            self._toggle_bbox_filter()
        elif key == ord("r"):
            self._draw_roi = not self._draw_roi
            self.get_logger().info(f"ROI draw {'ON' if self._draw_roi else 'OFF'}")
        elif key == ord("c"):
            self._rois.clear()
            self.get_logger().info("ROIs cleared")
        elif key == ord("s"):
            self._handle_snapshot(display)
        # Arrow keys: generic param adjustment for active tab.
        elif key in (82, 84, 81, 83):  # Up=82 Down=84 Left=81 Right=83
            # Map: (tab, key) → (attr, delta)
            _ARROW_MAP = {
                TAB_BLUR:        {82: ("_blur_kernel", 2), 84: ("_blur_kernel", -2)},
                TAB_CLAHE:       {82: ("_clahe_clip_x10", 5), 84: ("_clahe_clip_x10", -5)},
                TAB_RESIZED:     {82: ("_resize_scale", 10), 84: ("_resize_scale", -10)},
                TAB_THRESHOLD:   {82: ("_thresh_val", 5), 84: ("_thresh_val", -5)},
                TAB_CANNY:       {82: ("_canny_low", 5), 84: ("_canny_low", -5),
                                  83: ("_canny_high", 5), 81: ("_canny_high", -5)},
                TAB_CONTOURS:    {82: ("_min_contour_area", 100), 84: ("_min_contour_area", -100)},
                TAB_COLOUR_MASK: {82: ("_hsv_h_min", 2), 84: ("_hsv_h_min", -2),
                                  83: ("_hsv_h_max", 2), 81: ("_hsv_h_max", -2)},
            }
            tab_map = _ARROW_MAP.get(self._active_tab, {})
            if key in tab_map:
                attr, delta = tab_map[key]
                self._apply_ctrl_button(attr, delta)
        elif key == ord("+") or key == ord("="):
            if self._active_tab == TAB_COLOUR_MASK:
                self._apply_ctrl_button("_hsv_s_min", 5)
        elif key == ord("-"):
            if self._active_tab == TAB_COLOUR_MASK:
                self._apply_ctrl_button("_hsv_s_min", -5)
        elif key == ord("]"):
            if self._active_tab == TAB_COLOUR_MASK:
                self._apply_ctrl_button("_hsv_v_min", 5)
        elif key == ord("["):
            if self._active_tab == TAB_COLOUR_MASK:
                self._apply_ctrl_button("_hsv_v_min", -5)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = UnifiedVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
