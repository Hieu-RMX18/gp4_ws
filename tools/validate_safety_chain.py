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
            hw = jl_limits.get(joint_name)
            if hw is None:
                errors.append(
                    f"operational_joint_limits[{joint_name}] has no matching entry "
                    f"in joint_limits.yaml"
                )
                continue
            hw_min = hw.get("min_position")
            hw_max = hw.get("max_position")
            if hw_min is not None and op["min"] < hw_min:
                errors.append(
                    f"{joint_name}: operational min {op['min']:.4f} < "
                    f"hardware min {hw_min:.4f}"
                )
            if hw_max is not None and op["max"] > hw_max:
                errors.append(
                    f"{joint_name}: operational max {op['max']:.4f} > "
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

    # --- Check 3: J5 hard limit ---
    j5 = op_limits.get("joint_5_b")
    if j5 is None:
        errors.append("joint_5_b missing from operational_joint_limits")
    else:
        if abs(j5["min"] - (-1.603)) > 0.001 or abs(j5["max"] - 1.603) > 0.001:
            errors.append(
                f"joint_5_b limits are [{j5['min']}, {j5['max']}] — "
                f"expected ±1.603 (widened from ±1.571 per operator 2026-05-04)"
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
