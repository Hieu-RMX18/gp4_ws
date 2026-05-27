"""Safety guards for perception calibration and depth quality.

Two guards:
  - reprojection error (≤3 mm)
  - depth noise (range-aware, interpolated from breakpoints)

Calibration age/freshness is NOT enforced — a valid calibration_date is
required to exist but has no expiry.  Re-calibrate when the camera is
physically moved.
"""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

# SSOT schema for extrinsics.yaml — used by validators and safety chain.
EXTRINSICS_SCHEMA = {
    "hand_eye_extrinsics": {
        "parent_frame": "base_link",
        "child_frame": "camera_link",
        "translation": {"x": float, "y": float, "z": float},
        "rotation_quat": {"x": float, "y": float, "z": float, "w": float},
        "calibration_date": str,
        "reprojection_error_mm": float,
        "n_samples": int,
        "solver": str,
        "workspace_distance_m": float,
    }
}


def check_calibration_freshness(
    extrinsics_yaml: dict, max_age_days: int = 0
) -> tuple[bool, str]:
    """Return (ok, reason) — only checks that a calibration_date exists.

    The *max_age_days* parameter is kept in the signature for API
    compatibility but is **ignored**.  No time-based expiry is enforced.
    """
    date_str = extrinsics_yaml.get("hand_eye_extrinsics", {}).get("calibration_date")
    if not date_str or date_str == "<NOT_CALIBRATED>":
        return False, "calibration_date missing — run /perception/calibrate_hand_eye"
    try:
        normalized = date_str.replace("Z", "+00:00")
        cal_date = datetime.fromisoformat(normalized)
        if cal_date.tzinfo is None:
            cal_date = cal_date.replace(tzinfo=timezone.utc)
    except ValueError:
        return False, f"calibration_date '{date_str}' is not a valid ISO 8601 timestamp"
    # No age check — calibration is valid indefinitely until re-calibrated.
    return True, ""


def check_reprojection_error(extrinsics_yaml: dict, max_mm: float) -> tuple[bool, str]:
    """Return (ok, reason) for reprojection error check."""
    err = extrinsics_yaml.get("hand_eye_extrinsics", {}).get("reprojection_error_mm")
    if err is None:
        return False, "reprojection_error_mm missing"
    if err > max_mm:
        return False, f"reprojection_error_mm = {err:.2f} > max {max_mm}"
    return True, ""


def _interpolate_threshold(
    distance_m: float,
    breakpoints: list[dict[str, Any]],
    extrapolation: str = "reject",
) -> float | None:
    """Linear interpolation of noise threshold from breakpoints.

    Args:
        extrapolation: 'reject' returns None for out-of-range distances,
                       'clamp' uses the nearest endpoint threshold.
    """
    if not breakpoints:
        return None
    bp = sorted(breakpoints, key=lambda b: b["distance_m"])
    if distance_m < bp[0]["distance_m"]:
        if extrapolation == "clamp":
            return float(bp[0]["noise_mm_max"])
        return None
    if distance_m > bp[-1]["distance_m"]:
        if extrapolation == "clamp":
            return float(bp[-1]["noise_mm_max"])
        return None
    for i in range(len(bp) - 1):
        d0, n0 = bp[i]["distance_m"], bp[i]["noise_mm_max"]
        d1, n1 = bp[i + 1]["distance_m"], bp[i + 1]["noise_mm_max"]
        if d0 <= distance_m <= d1:
            if d1 == d0:
                return n0
            t = (distance_m - d0) / (d1 - d0)
            return n0 + t * (n1 - n0)
    return None


def check_depth_noise(
    detection_distance_m: float,
    observed_noise_mm: float,
    breakpoints: list[dict[str, Any]],
    extrapolation: str = "reject",
) -> tuple[bool, str]:
    """Return (ok, reason) for depth noise at a given distance."""
    threshold = _interpolate_threshold(detection_distance_m, breakpoints, extrapolation)
    if threshold is None:
        return (
            False,
            f"distance {detection_distance_m:.2f} m outside calibrated breakpoints",
        )
    if observed_noise_mm > threshold:
        return (
            False,
            f"depth_noise {observed_noise_mm:.2f} mm > threshold {threshold:.2f} mm "
            f"at {detection_distance_m:.2f} m",
        )
    return True, ""
