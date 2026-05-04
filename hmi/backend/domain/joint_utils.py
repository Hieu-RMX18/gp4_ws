"""Joint target resolution utilities — extracted from adapter.py."""

from __future__ import annotations

import re
from typing import Any, Sequence


def resolve_joint_target(
    parameters: dict[str, Any],
    joint_names: Sequence[str],
) -> tuple[int | None, str | None]:
    name_to_index = {name: i for i, name in enumerate(joint_names)}
    count = len(joint_names)

    raw_index = parameters.get("jointIndexZeroBased")
    if raw_index is not None:
        try:
            candidate = int(raw_index)
        except (TypeError, ValueError):
            candidate = None
        if candidate is not None and 0 <= candidate < count:
            return candidate, joint_names[candidate]

    raw_name = (
        str(
            parameters.get("jointNameResolved")
            or parameters.get("joint")
            or parameters.get("jointName")
            or ""
        )
        .strip()
        .lower()
    )
    if raw_name:
        canonical_index = name_to_index.get(raw_name)
        if canonical_index is not None:
            return canonical_index, joint_names[canonical_index]
        match = re.fullmatch(r"joint[_\s-]*([1-6])(?:[_\s-].+)?", raw_name)
        if match:
            zero_based = int(match.group(1)) - 1
            return zero_based, joint_names[zero_based]

    raw_index = parameters.get("jointIndex")
    if raw_index is not None:
        try:
            candidate = int(raw_index)
        except (TypeError, ValueError):
            candidate = None
        if candidate is not None:
            if 0 <= candidate < count:
                return candidate, joint_names[candidate]
            if 1 <= candidate <= count:
                zero_based = candidate - 1
                return zero_based, joint_names[zero_based]

    return None, None


def read_joint_position_deg(
    joint_name: str,
    joint_positions: Sequence[Any],
) -> float | None:
    for joint in joint_positions:
        if joint.name == joint_name:
            return joint.position_deg
    return None
