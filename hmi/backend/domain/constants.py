"""Canonical GP4 robot constants — single source of truth."""

from __future__ import annotations

GP4_JOINT_NAMES: tuple[str, ...] = (
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t",
)

GP4_JOINT_COUNT: int = len(GP4_JOINT_NAMES)
