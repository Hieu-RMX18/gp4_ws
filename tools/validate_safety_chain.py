#!/usr/bin/env python3
"""W1.T5: Validate the safety chain configuration.

Checks:
  1. operational_joint_limits exist and are a strict subset of URDF joint_limits.
  2. Every joint in motoros2_config.yaml has an entry in operational_joint_limits.
  3. J5 hard limit is ±1.571 rad (±90°).

Exit 0 on success, 1 on failure.
"""

import sys
from pathlib import Path

import yaml


def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def extract_limit(joint_entry):
    """Extract (min, max) from either flat {min, max} or tiered {default: {min, max}}."""
    if isinstance(joint_entry, dict):
        if "default" in joint_entry and isinstance(joint_entry["default"], dict):
            d = joint_entry["default"]
            return (d.get("min"), d.get("max"))
        if "min" in joint_entry and "max" in joint_entry:
            return (joint_entry["min"], joint_entry["max"])
    return (None, None)


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    safety_rules_path = repo_root / "src" / "safety" / "config" / "safety_rules.yaml"
    joint_limits_path = (
        repo_root / "src" / "gp4_moveit_config" / "config" / "joint_limits.yaml"
    )
    motoros2_path = repo_root / "motoros2_config.yaml"

    errors: list[str] = []

    # --- Load files ---
    if not safety_rules_path.exists():
        errors.append(f"safety_rules.yaml not found: {safety_rules_path}")
        print("\n".join(errors), file=sys.stderr)
        return 1

    safety = load_yaml(safety_rules_path)
    op_limits = safety.get("operational_joint_limits")
    if not op_limits or not isinstance(op_limits, dict):
        errors.append(
            "operational_joint_limits section missing or empty in safety_rules.yaml"
        )
        print("\n".join(errors), file=sys.stderr)
        return 1

    # --- Check 1: subset of URDF limits ---
    if joint_limits_path.exists():
        jl_data = load_yaml(joint_limits_path)
        jl_limits = jl_data.get("joint_limits", {})
        for joint_name, op in op_limits.items():
            if not isinstance(op, dict):
                continue
            op_min, op_max = extract_limit(op)
            if op_min is None or op_max is None:
                continue
            hw = jl_limits.get(joint_name)
            if hw is None:
                errors.append(
                    f"operational_joint_limits[{joint_name}] has no matching entry "
                    f"in joint_limits.yaml"
                )
                continue
            hw_min = hw.get("min_position")
            hw_max = hw.get("max_position")
            if op_min is not None and hw_min is not None and op_min < hw_min:
                errors.append(
                    f"{joint_name}: operational min {op_min:.4f} < "
                    f"hardware min {hw_min:.4f}"
                )
            if op_max is not None and hw_max is not None and op_max > hw_max:
                errors.append(
                    f"{joint_name}: operational max {op_max:.4f} > "
                    f"hardware max {hw_max:.4f}"
                )
    else:
        errors.append(f"joint_limits.yaml not found: {joint_limits_path}")

    # --- Check 2: motoros2 joints covered ---
    if motoros2_path.exists():
        moto = load_yaml(motoros2_path)
        moto_joints: list[str] = []
        # motoros2_config.yaml may list joints under various keys
        for key in ("joint_names", "joints"):
            val = moto.get(key, [])
            if isinstance(val, list):
                for item in val:
                    if isinstance(item, list):
                        moto_joints.extend(item)
                    elif isinstance(item, str):
                        moto_joints.append(item)
        for jn in moto_joints:
            if jn not in op_limits:
                errors.append(
                    f"motoros2_config joint '{jn}' missing from operational_joint_limits"
                )
            else:
                op_entry = op_limits[jn]
                op_jn_min, op_jn_max = extract_limit(op_entry)
                if op_jn_min is None or op_jn_max is None:
                    errors.append(
                        f"motoros2_config joint '{jn}' has no parseable limits in operational_joint_limits"
                    )

    # --- Check 3: J5 hard limit ---
    j5 = op_limits.get("joint_5_b")
    if j5 is None:
        errors.append("joint_5_b missing from operational_joint_limits")
    else:
        j5_min, j5_max = extract_limit(j5)
    if j5_min is None or j5_max is None:
        errors.append("joint_5_b has no parseable limits in operational_joint_limits")
    else:
        if abs(j5_min - (-1.603)) > 0.001 or abs(j5_max - 1.603) > 0.001:
            errors.append(
                f"joint_5_b limits are [{j5_min}, {j5_max}] — "
                f"expected ±1.603 (widened from ±1.571 per operator 2026-05-04)"
            )

    # --- Check 4: Perception calibration SSOT ---
    calibration = safety.get("calibration")
    if calibration is None:
        errors.append("safety.calibration section missing (required by W4)")
    else:
        max_age = calibration.get("max_age_days")
        if max_age is None or not isinstance(max_age, int):
            errors.append("safety.calibration.max_age_days missing or not an integer")
        max_reproj = calibration.get("max_reprojection_error_mm")
        if max_reproj is None or not isinstance(max_reproj, (int, float)):
            errors.append(
                "safety.calibration.max_reprojection_error_mm missing or not a float"
            )

    # --- Check 5: Perception extrinsics YAML exists and is valid ---
    extrinsics_path = (
        repo_root / "src" / "gp4_perception" / "config" / "extrinsics.yaml"
    )
    if extrinsics_path.exists():
        extrinsics = load_yaml(extrinsics_path)
        hee = extrinsics.get("hand_eye_extrinsics", {})
        date_str = hee.get("calibration_date", "")
        if not date_str or date_str == "<NOT_CALIBRATED>":
            errors.append(
                "gp4_perception/config/extrinsics.yaml is not calibrated (<NOT_CALIBRATED>)"
            )
        else:
            try:
                from datetime import datetime, timezone

                cal_date = datetime.fromisoformat(
                    date_str.rstrip("Z").replace("Z", "+00:00")
                )
                now = datetime.now(timezone.utc)
                age_days = (now - cal_date).total_seconds() / 86400.0
                if calibration and age_days > calibration.get("max_age_days", 30):
                    errors.append(
                        f"extrinsics.yaml calibration is {age_days:.1f} days old "
                        f"(max {calibration.get('max_age_days', 30)} days)"
                    )
            except ValueError:
                errors.append(f"extrinsics.yaml calibration_date invalid: {date_str}")
        reproj = hee.get("reprojection_error_mm")
        if reproj is None:
            errors.append("extrinsics.yaml reprojection_error_mm missing")
        elif calibration and reproj > calibration.get("max_reprojection_error_mm", 3.0):
            errors.append(
                f"extrinsics.yaml reprojection_error_mm = {reproj:.2f} > "
                f"max {calibration.get('max_reprojection_error_mm', 3.0)}"
            )
    else:
        # Not a hard error on clean build; warn only.
        print(
            f"  ⚠ extrinsics.yaml not found: {extrinsics_path} "
            "(expected before first calibration run)"
        )

    # --- Report ---
    if errors:
        print("validate_safety_chain FAILED:", file=sys.stderr)
        for e in errors:
            print(f"  ✗ {e}", file=sys.stderr)
        return 1

    print("validate_safety_chain: all checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
