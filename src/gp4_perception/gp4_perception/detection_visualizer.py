"""RGB + aligned-depth object detection and visualization node.

Subscribes to:
    /camera/color/image_raw (Image)
    /camera/aligned_depth_to_color/image_raw (Image, 16UC1)
    /camera/color/camera_info (CameraInfo)

Publishes:
    /perception/detections (Detection3DArray)
    /perception/annotated_image (Image with bounding boxes + labels)

Entry point: ros2 run gp4_perception detection_visualizer
"""

from __future__ import annotations

import logging
import sys

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

_LOGGER = logging.getLogger(__name__)


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


class DetectionVisualizer(Node):
    """RGB + aligned-depth object detection and visualization node.

    Subscribes to:
        /camera/color/image_raw (Image)
        /camera/aligned_depth_to_color/image_raw (Image, 16UC1)
        /camera/color/camera_info (CameraInfo)

    Publishes:
        /perception/detections (Detection3DArray)
        /perception/annotated_image (Image with bounding boxes + labels)
    """

    def __init__(self) -> None:
        super().__init__("detection_visualizer")

        # Load config.
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
        rgb_cfg = pcfg.get("rgb_detector", {})
        viz_cfg = pcfg.get("visualization", {})

        self._depth_scale = float(rgb_cfg.get("depth_scale_m", 0.001))
        self._base_frame = str(rgb_cfg.get("base_frame", "base_link"))
        self._camera_frame = str(rgb_cfg.get("camera_optical_frame",
                                              "camera_color_optical_frame"))
        self._bbox_z = float(rgb_cfg.get("bbox_thickness_z_m", 0.03))
        self._sync_slop = float(rgb_cfg.get("sync_slop_s", 0.05))
        self._sync_queue = int(rgb_cfg.get("sync_queue", 10))

        # Visualization config — new nested structure.
        ann_cfg = viz_cfg.get("annotated_image", {})
        self._layout = str(ann_cfg.get("layout", viz_cfg.get("layout", "side_by_side")))
        self._max_width_px = viz_cfg.get("max_width_px")
        self._max_height_px = viz_cfg.get("max_height_px")
        self._output_width = viz_cfg.get("output_width_px")

        dash_cfg = viz_cfg.get("dashboard", {})
        self._dashboard_enabled = bool(dash_cfg.get("enabled", True))
        self._dashboard_fps = float(dash_cfg.get("rate_hz", viz_cfg.get("dashboard_fps", 2.0)))

        self._zoom_cfg = viz_cfg.get("zoom_roi", {})
        self._zoom_roi_fps = float(self._zoom_cfg.get("rate_hz", viz_cfg.get("zoom_roi_fps", 5.0)))

        # Label config.
        label_cfg = viz_cfg.get("label", viz_cfg.get("font", {}))
        self._font_cfg = {
            "scale": float(label_cfg.get("font_scale", label_cfg.get("scale", 0.75))),
            "thickness": int(label_cfg.get("font_thickness", label_cfg.get("thickness", 2))),
            "bbox_thickness": int(label_cfg.get("bbox_thickness", 3)),
            "padding_px": int(label_cfg.get("padding_px", 5)),
            "line_spacing_px": int(label_cfg.get("line_spacing_px", 14)),
            "background_alpha": float(label_cfg.get("background_alpha", 0.85)),
        }
        self._distance_unit = str(label_cfg.get("distance_unit", "mm"))
        self._score_decimals = int(label_cfg.get("score_decimals", 2))
        self._xyz_decimals = int(label_cfg.get("decimals_xyz", 3))

        # Debug mask config.
        debug_mask_cfg = viz_cfg.get("debug_masks", {})
        self._debug_masks_enabled = bool(debug_mask_cfg.get("enabled", True))

        color_topic = str(rgb_cfg.get("color_topic", "/camera/color/image_raw"))
        depth_topic = str(rgb_cfg.get("depth_topic",
                                       "/camera/aligned_depth_to_color/image_raw"))
        info_topic = str(rgb_cfg.get("camera_info_topic",
                                      "/camera/color/camera_info"))

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_tf = None

        # Camera intrinsics (populated from CameraInfo).
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None

        # Low-latency QoS.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # CameraInfo subscriber — cached intrinsics, not time-synced.
        self.create_subscription(CameraInfo, info_topic,
                                 self._on_camera_info, qos)

        # Synchronized color + depth.
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        self._color_sub = Subscriber(self, Image, color_topic, qos_profile=qos)
        self._depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)
        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=self._sync_queue,
            slop=self._sync_slop,
        )
        self._sync.registerCallback(self._on_synced_rgbd)

        # Publishers.
        self._det_pub = self.create_publisher(
            Detection3DArray, "/perception/detections", 10
        )
        self._annotated_pub = self.create_publisher(
            Image, "/perception/annotated_image", 10
        )

        # QoS for debug images.
        qos_cfg = viz_cfg.get("qos", {}).get("debug_images", viz_cfg.get("debug_qos", {}))
        rel_str = qos_cfg.get("reliability", "reliable").lower()
        debug_reliability = ReliabilityPolicy.RELIABLE if rel_str == "reliable" else ReliabilityPolicy.BEST_EFFORT
        debug_qos = QoSProfile(
            reliability=debug_reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=int(qos_cfg.get("depth", 1)),
            durability=DurabilityPolicy.VOLATILE,
        )

        self._dashboard_pub = self.create_publisher(
            Image, "/perception/debug_dashboard_image", debug_qos
        )
        self._zoom_pub = self.create_publisher(
            Image, "/perception/zoom_roi_image", debug_qos
        )

        # Debug mask publishers.
        self._blue_border_mask_pub = self.create_publisher(
            Image, "/perception/debug_mask/blue_border", debug_qos
        )
        self._white_wp_mask_pub = self.create_publisher(
            Image, "/perception/debug_mask/white_workpiece", debug_qos
        )

        import time
        self._last_dashboard_time = time.time()
        self._last_zoom_time = time.time()

        self.get_logger().info(
            f"DetectionVisualizer node started (RGB+depth path, {len(self._color_classes)} color classes)."
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Cache camera intrinsics from CameraInfo."""
        k = msg.k  # 9-element row-major
        self._fx = float(k[0])
        self._fy = float(k[4])
        self._cx = float(k[2])
        self._cy = float(k[5])

    def _on_synced_rgbd(self, color_msg: Image, depth_msg: Image) -> None:
        """Process synchronized color + aligned-depth frame."""
        if self._fx is None:
            return  # no intrinsics yet

        # Decode images.
        try:
            rgb = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge color conversion failed: {exc}")
            return
        try:
            depth_raw = self._bridge.imgmsg_to_cv2(depth_msg,
                                                    desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge depth conversion failed: {exc}")
            return

        # 2D detection with shape filters + cross-class NMS.
        objects = detect_color_objects(
            rgb, self._color_classes, self._postprocess_cfg
        )

        # TF: camera_color_optical_frame → base_link (non-blocking).
        tf_cam_to_base = None
        try:
            tf_cam_to_base = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._camera_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            self._last_tf = tf_cam_to_base
        except Exception:
            if self._last_tf is not None:
                tf_cam_to_base = self._last_tf

        # Build Detection3DArray.
        from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
        from geometry_msgs.msg import PoseWithCovariance
        from vision_msgs.msg import (
            Detection3D, ObjectHypothesis, ObjectHypothesisWithPose,
        )
        from std_msgs.msg import Header

        det_arr = Detection3DArray()
        overlay_rgb = rgb.copy()  # annotate on a copy
        n_published = 0
        dashboard_data = []
        # Track enriched objects for zoom grid (with distance, IDs).
        enriched_objects: list[dict] = []

        layout_mode = self._layout

        for idx, obj in enumerate(objects):
            obj_id = f"#{idx + 1}"
            class_id = obj["class_id"]
            x, y, w, h = obj["bbox"]
            mask = obj["mask"]
            cx_px, cy_px = obj["center_uv"]
            confidence = obj["confidence"]

            # Median depth over mask within bbox.
            depth_roi = depth_raw[y:y + h, x:x + w]
            mask_roi = mask[y:y + h, x:x + w]
            z_m = _median_depth_m(depth_roi, mask_roi,
                                   depth_scale=self._depth_scale)

            # Overlay colour.
            color_bgr = _color_for_class(class_id)

            # Draw 2D bounding box on overlay.
            bbox_thick = int(self._font_cfg.get("bbox_thickness", 3))
            cv2.rectangle(overlay_rgb, (x, y), (x + w, y + h), color_bgr, bbox_thick)

            if z_m is None or z_m <= 0.0:
                # Depth invalid — draw bbox but don't publish detection.
                lines = [f"{obj_id} {class_id}", "XYZ_INVALID"]
                _draw_detection_callout(
                    overlay_rgb, (x, y, w, h), lines, color_bgr, self._font_cfg,
                    tag_fallback=True, tag_text=obj_id
                )
                continue

            # Deproject center to camera XYZ.
            cam_x, cam_y, cam_z = _deproject_pixel(
                float(cx_px), float(cy_px), z_m,
                self._fx, self._fy, self._cx, self._cy,
            )
            distance_m = z_m

            # Transform to base_link.
            base_xyz_str = "TF_UNAVAILABLE"
            frame_id = self._camera_frame
            det_x, det_y, det_z = cam_x, cam_y, cam_z

            if tf_cam_to_base is not None:
                try:
                    from scipy.spatial.transform import Rotation
                    t = tf_cam_to_base.transform.translation
                    r = tf_cam_to_base.transform.rotation
                    rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
                    cam_pt = np.array([cam_x, cam_y, cam_z])
                    base_pt = rot.apply(cam_pt) + np.array([t.x, t.y, t.z])
                    det_x, det_y, det_z = float(base_pt[0]), float(base_pt[1]), float(base_pt[2])
                    frame_id = self._base_frame
                    d = self._xyz_decimals
                    base_xyz_str = f"({det_x:.{d}f},{det_y:.{d}f},{det_z:.{d}f})"
                except Exception as exc:
                    self.get_logger().debug(f"TF transform failed: {exc}")

            # Build Detection3D.
            det = Detection3D()
            det.header = Header(
                stamp=color_msg.header.stamp,
                frame_id=frame_id,
            )
            pose = Pose()
            pose.position = Point(x=det_x, y=det_y, z=det_z)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            hyp = ObjectHypothesis()
            hyp.class_id = class_id
            hyp.score = float(confidence)
            det.results.append(ObjectHypothesisWithPose(
                hypothesis=hyp,
                pose=PoseWithCovariance(pose=pose),
            ))
            # Bbox size from pixel extent at depth.
            sx, sy = _bbox_size_m(float(w), float(h), z_m,
                                   self._fx, self._fy)
            det.bbox.size = Vector3(x=sx, y=sy, z=self._bbox_z)
            det_arr.detections.append(det)
            n_published += 1

            # Compact overlay label: #id class score distance
            dist_str = _format_distance(distance_m, self._distance_unit)
            sd = self._score_decimals
            lines = [f"{obj_id} {class_id} {confidence:.{sd}f} {dist_str}"]

            d = self._xyz_decimals
            dashboard_data.append({
                "id": obj_id,
                "class": class_id,
                "conf": f"{confidence:.{sd}f}",
                "dist": dist_str,
                "tf": base_xyz_str,
                "cam_xyz": f"({cam_x:.{d}f},{cam_y:.{d}f},{cam_z:.{d}f})",
                "bbox_px": f"[{x},{y},{w},{h}]",
            })

            enriched_objects.append({
                **obj,
                "obj_id": obj_id,
                "distance_m": distance_m,
                "dist_str": dist_str,
            })

            _draw_detection_callout(
                overlay_rgb, (x, y, w, h), lines, color_bgr, self._font_cfg
            )

        # Publish debug masks from white_workpiece candidates.
        if self._debug_masks_enabled:
            self._publish_debug_masks(objects, color_msg.header)

        # Status bar.
        cv2.putText(overlay_rgb, f"Detections: {n_published} Frame: {color_msg.header.frame_id}",
                     (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, lineType=cv2.LINE_AA)

        # Publish Detection3DArray.
        det_arr.header = Header(
            stamp=color_msg.header.stamp,
            frame_id=self._base_frame if tf_cam_to_base else self._camera_frame,
        )
        self._det_pub.publish(det_arr)

        # Process depth colormap.
        depth_vis = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_color_rgb = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        depth_color_rgb = cv2.cvtColor(depth_color_rgb, cv2.COLOR_BGR2RGB)

        if depth_color_rgb.shape[0] != overlay_rgb.shape[0]:
            scale_f = overlay_rgb.shape[0] / depth_color_rgb.shape[0]
            new_w = int(depth_color_rgb.shape[1] * scale_f)
            depth_color_rgb = cv2.resize(depth_color_rgb, (new_w, overlay_rgb.shape[0]))

        # Build main lightweight layout.
        if layout_mode == "rgb_only":
            combined = overlay_rgb
        elif layout_mode == "depth_only":
            combined = depth_color_rgb
        else:  # side_by_side (default)
            combined = np.hstack([overlay_rgb, depth_color_rgb])

        # Dynamic resizing.
        ch, cw = combined.shape[:2]
        if self._output_width is not None and self._max_width_px is None:
            self._max_width_px = self._output_width

        scale_f = 1.0
        if self._max_width_px and cw > self._max_width_px:
            scale_f = min(scale_f, self._max_width_px / cw)
        if self._max_height_px and ch > self._max_height_px:
            scale_f = min(scale_f, self._max_height_px / ch)
        if scale_f < 1.0:
            combined = cv2.resize(combined, (int(cw * scale_f), int(ch * scale_f)))

        # Publish annotated image.
        try:
            bgr = cv2.cvtColor(combined, cv2.COLOR_RGB2BGR)
            annotated_msg = self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            annotated_msg.header = color_msg.header
            self._annotated_pub.publish(annotated_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish annotated image: {exc}")

        # Rate-limited debug images.
        import time
        now = time.time()

        # Zoom ROI — grid or single mode.
        if self._zoom_cfg.get("enabled", False) and enriched_objects:
            if (now - self._last_zoom_time) >= (1.0 / self._zoom_roi_fps):
                self._last_zoom_time = now
                try:
                    zoom_img = _build_zoom_grid(rgb, enriched_objects, self._zoom_cfg, self._font_cfg)
                    if zoom_img is not None and zoom_img.size > 0:
                        z_bgr = cv2.cvtColor(zoom_img, cv2.COLOR_RGB2BGR)
                        zoom_msg = self._bridge.cv2_to_imgmsg(z_bgr, encoding="bgr8")
                        zoom_msg.header = color_msg.header
                        self._zoom_pub.publish(zoom_msg)
                except Exception as exc:
                    self.get_logger().warn(f"Failed to publish zoom image: {exc}")

        # Dashboard.
        if self._dashboard_enabled and (now - self._last_dashboard_time) >= (1.0 / self._dashboard_fps):
            self._last_dashboard_time = now
            try:
                top_row = np.hstack([overlay_rgb, depth_color_rgb])
                required_h = 130 + len(dashboard_data) * 40 + 20
                panel_h = max(required_h, top_row.shape[0] // 2)
                panel_w = top_row.shape[1]
                panel = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)

                cv2.putText(panel, "--- Detections Dashboard ---", (15, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, lineType=cv2.LINE_AA)
                cv2.putText(panel, f"Total: {n_published}  TF Base: {self._base_frame}", (15, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, lineType=cv2.LINE_AA)

                y_offset = 130
                for row in dashboard_data:
                    if y_offset > panel_h - 20:
                        break
                    text = (f"{row['id']} | {row['class']} | Conf: {row['conf']} | "
                            f"Dist: {row['dist']} | TF: {row['tf']} | "
                            f"Cam: {row['cam_xyz']} | Px: {row['bbox_px']}")
                    cv2.putText(panel, text, (15, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, lineType=cv2.LINE_AA)
                    y_offset += 40

                dashboard_combined = np.vstack([top_row, panel])
                dash_bgr = cv2.cvtColor(dashboard_combined, cv2.COLOR_RGB2BGR)
                dash_msg = self._bridge.cv2_to_imgmsg(dash_bgr, encoding="bgr8")
                dash_msg.header = color_msg.header
                self._dashboard_pub.publish(dash_msg)
            except Exception as exc:
                self.get_logger().warn(f"Failed to publish dashboard: {exc}")

    def _publish_debug_masks(self, objects: list[dict], header) -> None:
        """Publish debug masks for white_workpiece detections."""
        for obj in objects:
            if obj.get("source") != "blue_border_white_fill":
                continue
            blue_mask = obj.get("_blue_mask")
            if blue_mask is not None:
                try:
                    mask_msg = self._bridge.cv2_to_imgmsg(blue_mask, encoding="mono8")
                    mask_msg.header = header
                    self._blue_border_mask_pub.publish(mask_msg)
                except Exception:
                    pass
            wp_mask = obj.get("_white_mask")
            if wp_mask is not None:
                try:
                    mask_msg = self._bridge.cv2_to_imgmsg(wp_mask, encoding="mono8")
                    mask_msg.header = header
                    self._white_wp_mask_pub.publish(mask_msg)
                except Exception:
                    pass


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


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = DetectionVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

