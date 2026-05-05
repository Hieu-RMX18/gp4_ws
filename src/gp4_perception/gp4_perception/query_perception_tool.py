"""query_perception tool body — fills W3 ReAct stub.

Imported by llm_gateway.react.tools.query_perception when gp4_perception is installed.
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

    # In a real runtime, detections are read from a cached subscriber snapshot.
    # The service node (object_query_service.py) has the live subscriber.
    # Here we return a placeholder so the ReAct tool layer can bridge.
    _LOGGER.warning(
        "query_perception_tool received call but no live detection cache is wired "
        "(expected when running inside the object_query_service node)."
    )
    return {
        "ok": True,
        "error": None,
        "payload": {"detections": []},
    }
