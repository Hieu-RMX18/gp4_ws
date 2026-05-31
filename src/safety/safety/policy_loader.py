"""Single-source policy loader for safety_rules.yaml.

All Python consumers that need motion limits, workspace bounds, or
forbidden zones should use ``load_safety_rules()`` from this module
instead of duplicating YAML loading logic.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import yaml

_LOGGER = logging.getLogger(__name__)

# Fail-safe defaults — used only when safety_rules.yaml is completely
# unreachable. These must stay conservative and match the policy YAML.
_FAILSAFE_MOTION_LIMITS: dict[str, float] = {
    "max_velocity_scale": 0.06,
    "max_acceleration_scale": 0.06,
    "max_move_rel_translation": 0.21,
}


def load_safety_rules() -> dict[str, Any]:
    """Load safety_rules.yaml from the safety package share directory.

    Falls back to the workspace source tree if the installed package
    is not found (e.g. during development without colcon install).
    Returns an empty dict on complete failure.
    """
    # Try installed package path first (production).
    try:
        from ament_index_python.packages import get_package_share_directory

        yaml_path = (
            Path(get_package_share_directory("safety")) / "config" / "safety_rules.yaml"
        )
        if yaml_path.exists():
            rules = yaml.safe_load(yaml_path.read_text()) or {}
            if rules:
                return rules
            _LOGGER.warning(
                "policy_loader: safety_rules.yaml exists but is empty: %s", yaml_path
            )
    except Exception as ex:
        _LOGGER.debug("policy_loader: ament_index lookup failed: %s", ex)

    # Fallback: workspace source tree.
    workspace_path = (
        Path(__file__).resolve().parents[1] / "config" / "safety_rules.yaml"
    )
    if workspace_path.exists():
        try:
            rules = yaml.safe_load(workspace_path.read_text()) or {}
            if rules:
                _LOGGER.info(
                    "policy_loader: loaded from workspace source: %s", workspace_path
                )
                return rules
        except Exception as ex:
            _LOGGER.warning("policy_loader: workspace YAML parse failed: %s", ex)

    _LOGGER.warning(
        "policy_loader: no safety_rules.yaml found; using fail-safe defaults."
    )
    return {}


def get_motion_limits(rules: dict[str, Any] | None = None) -> dict[str, float]:
    """Extract motion limits from loaded safety rules.

    Returns a dict with keys: max_velocity_scale, max_acceleration_scale,
    max_move_rel_translation. Falls back to fail-safe constants for any
    missing key.
    """
    if rules is None:
        rules = load_safety_rules()
    motion = rules.get("motion_limits", {})
    legacy = rules.get("joint_limits_override", {})
    return {
        "max_velocity_scale": float(
            motion.get(
                "max_velocity_scale",
                legacy.get(
                    "max_velocity_scale", _FAILSAFE_MOTION_LIMITS["max_velocity_scale"]
                ),
            )
        ),
        "max_acceleration_scale": float(
            motion.get(
                "max_acceleration_scale",
                legacy.get(
                    "max_acceleration_scale",
                    _FAILSAFE_MOTION_LIMITS["max_acceleration_scale"],
                ),
            )
        ),
        "max_move_rel_translation": float(
            motion.get(
                "max_move_rel_translation",
                _FAILSAFE_MOTION_LIMITS["max_move_rel_translation"],
            )
        ),
    }
