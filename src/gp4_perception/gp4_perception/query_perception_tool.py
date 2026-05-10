"""query_perception tool body — fills W3 ReAct stub.

Used by llm_gateway.react_planner.QueryPerceptionTool when gp4_perception is installed.

Enhanced in Phase A3: returns human-readable object descriptions for LLM reasoning.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import yaml

from .safety_guards import check_calibration_freshness

_LOGGER = logging.getLogger(__name__)


def load_extrinsics(config_path: Path | None = None) -> dict:
    """Load extrinsics.yaml from the gp4_perception config share."""
    if config_path is None:
        # Resolve via ament_index or fallback to known install path
        try:
            from ament_index_python.packages import get_package_share_directory

            share = Path(get_package_share_directory("gp4_perception"))
        except Exception:
            share = Path(__file__).resolve().parents[1] / "config"
        config_path = share / "extrinsics.yaml"
    with open(config_path) as f:
        return yaml.safe_load(f) or {}


def _format_detection(detection: dict) -> dict:
    """Convert a raw Detection3D-like dict into a human-readable description.

    Returns a dict with keys: class_id, description, position, size_m.
    """
    results = detection.get("results", [])
    class_id = ""
    position = {"x": 0.0, "y": 0.0, "z": 0.0}
    size = {"x": 0.0, "y": 0.0, "z": 0.0}

    if results:
        first = results[0]
        hyp = first.get("hypothesis", {})
        class_id = hyp.get("class_id", "unknown")
        pose_data = first.get("pose", {}).get("pose", {})
        pos = pose_data.get("position", {})
        position = {
            "x": round(float(pos.get("x", 0.0)), 3),
            "y": round(float(pos.get("y", 0.0)), 3),
            "z": round(float(pos.get("z", 0.0)), 3),
        }

    bbox = detection.get("bbox", {})
    bbox_size = bbox.get("size", {})
    size = {
        "x": round(float(bbox_size.get("x", 0.0)), 4),
        "y": round(float(bbox_size.get("y", 0.0)), 4),
        "z": round(float(bbox_size.get("z", 0.0)), 4),
    }

    # Approximate largest dimension in cm for human readability.
    max_dim_cm = round(max(size["x"], size["y"], size["z"]) * 100, 1)

    # Build human-readable description.
    parts = class_id.split("_") if class_id else ["unknown"]
    if len(parts) == 2:
        color, shape = parts[0], parts[1]
        desc = f"{color} {shape}"
    elif len(parts) == 1:
        desc = parts[0]
    else:
        desc = class_id

    description = (
        f"{desc} at x={position['x']}, y={position['y']}, z={position['z']} "
        f"(~{max_dim_cm} cm)"
    )

    return {
        "class_id": class_id,
        "description": description,
        "position": position,
        "size_m": size,
        "frame_id": "base_link",
    }


def _format_detections_from_ros(detections: list) -> list[dict]:
    """Convert ROS Detection3D messages to human-readable dicts."""
    formatted = []
    for det in detections:
        results = getattr(det, "results", [])
        class_id = ""
        position = {"x": 0.0, "y": 0.0, "z": 0.0}
        size = {"x": 0.0, "y": 0.0, "z": 0.0}

        if results:
            first = results[0]
            hyp = getattr(first, "hypothesis", None)
            class_id = str(getattr(hyp, "class_id", "unknown")) if hyp else "unknown"
            pose_obj = getattr(first, "pose", None)
            if pose_obj:
                inner_pose = getattr(pose_obj, "pose", None)
                if inner_pose:
                    pos = getattr(inner_pose, "position", None)
                    if pos:
                        position = {
                            "x": round(float(getattr(pos, "x", 0.0)), 3),
                            "y": round(float(getattr(pos, "y", 0.0)), 3),
                            "z": round(float(getattr(pos, "z", 0.0)), 3),
                        }

        bbox = getattr(det, "bbox", None)
        if bbox:
            bbox_size = getattr(bbox, "size", None)
            if bbox_size:
                size = {
                    "x": round(float(getattr(bbox_size, "x", 0.0)), 4),
                    "y": round(float(getattr(bbox_size, "y", 0.0)), 4),
                    "z": round(float(getattr(bbox_size, "z", 0.0)), 4),
                }

        max_dim_cm = round(max(size["x"], size["y"], size["z"]) * 100, 1)
        parts = class_id.split("_") if class_id else ["unknown"]
        if len(parts) == 2:
            color, shape = parts[0], parts[1]
            desc = f"{color} {shape}"
        elif len(parts) == 1:
            desc = parts[0]
        else:
            desc = class_id

        description = (
            f"{desc} at x={position['x']}, y={position['y']}, z={position['z']} "
            f"(~{max_dim_cm} cm)"
        )

        formatted.append({
            "class_id": class_id,
            "description": description,
            "position": position,
            "size_m": size,
            "frame_id": "base_link",
        })

    return formatted


def query_perception(
    args: dict[str, Any],
    context_state: dict[str, Any],
    max_age_days: int = 30,
    extrinsics_path: Path | None = None,
) -> dict[str, Any]:
    """Body of the query_perception tool.

    Args:
        args: tool arguments dict; may contain "class_filter".
        context_state: agent context state snapshot; must include "mode".
        max_age_days: calibration freshness threshold.
        extrinsics_path: override path to extrinsics.yaml.

    Returns:
        dict with keys ok, error, payload following ToolResult convention.
        payload.detections contains human-readable object descriptions.
    """
    mode = context_state.get("robot_state", {}).get("mode", "IDLE")
    if mode != "IDLE":
        return {
            "ok": False,
            "error": f"perception_blocked_during_motion (mode={mode})",
            "payload": None,
        }

    extrinsics = load_extrinsics(extrinsics_path)
    ok, reason = check_calibration_freshness(extrinsics, max_age_days)
    if not ok:
        return {"ok": False, "error": f"calibration_invalid: {reason}", "payload": None}

    # Source-only fallback for tests and non-ROS imports. In runtime, the
    # ReAct tool calls LLMGatewayNode._query_perception_detections(), which
    # queries /perception/get_object_positions from scene_processor.
    _LOGGER.warning(
        "query_perception_tool received call but no live detection cache is wired "
        "(expected when running inside the object_query_service node)."
    )
    return {
        "ok": True,
        "error": None,
        "payload": {
            "detections": [],
            "summary": "No objects detected (scene processor not connected).",
        },
    }
