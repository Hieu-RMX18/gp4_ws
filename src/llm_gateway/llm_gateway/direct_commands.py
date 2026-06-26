"""Tầng 1 của pipeline parse: lệnh deterministic an toàn, không cần LLM.

Spec: docs/superpowers/specs/2026-06-10-llm-gateway-factory-pipeline-design.md §3.
Hợp đồng:
  - Chỉ 5 lệnh: stop / go home / get pose / alarm reset / wait N giây.
  - Khớp NGUYÊN CÂU (sau khi fold dấu tiếng Việt, lowercase, gộp khoảng trắng).
  - Trả Semantic IR đơn (dict) khi khớp, None khi không — None nghĩa là
    "chuyển cho task_planner (LLM)", không bao giờ đoán mò.

Mở rộng:
  - parse_factory_task() khớp "đi tới [object]" / "go to [object]" / "nhặt [object]"
    → trả FactoryTask dict (task_type=factory_task), None khi không khớp.
"""

from __future__ import annotations

import re
import unicodedata
from typing import Any, Dict

_WAIT_MAX_SEC = 300.0

_STOP_TEXTS = frozenset({
    "stop", "stop motion", "cancel motion", "halt",
    "dung", "dung lai", "dung ngay",
})
_HOME_TEXTS = frozenset({
    "home", "go home", "move home", "return home",
    "ve nha", "ve home",
})
_GET_POSE_TEXTS = frozenset({
    "get pose", "get_pose", "get current pose", "current pose",
    "where is the robot", "where is robot", "where is tcp",
    "vi tri hien tai", "toa do hien tai", "robot dang o dau",
})
_ALARM_RESET_TEXTS = frozenset({
    "alarm reset", "alarm_reset", "reset alarm", "clear alarm",
    "xoa loi", "reset loi",
})
_WAIT_RE = re.compile(
    r"^(?:wait|cho|doi|cho doi)\s+(\d+(?:\.\d+)?)\s*"
    r"(?:s|sec|secs|second|seconds|giay)?$"
)
_BAT_KHI_NEN_TEXTS = frozenset({
    "bat khi nen", "mo khi nen", "bat khi", "mo khi", "turn on air", "turn on vacuum",
    "bat hut", "mo hut"
})
_TAT_KHI_NEN_TEXTS = frozenset({
    "tat khi nen", "dong khi nen", "tat khi", "dong khi", "turn off air", "turn off vacuum",
    "tat hut", "dong hut"
})

# ── "go to [object]" / "đi tới [object]" ────────────────────────────────
# Matches: "go to red_box", "đi tới yellow box", "đi đến hộp đỏ",
#          "tới hộp xanh", etc.
# After _fold: diacritics removed, đ→d, lowercase.
# NOTE: "move to" is deliberately excluded — it is ambiguous with named
# poses (e.g., "move to ready") and should fall through to the LLM planner.
_MOVE_TO_OBJECT_RE = re.compile(
    r"^(?:go to|di toi|di den|toi|den)\s+(.+)$"
)

# ── "pick [object]" / "nhặt [object]" ───────────────────────────────────
_PICK_OBJECT_RE = re.compile(
    r"^(?:pick|pick up|grab|nhat|cam|lay)\s+(.+)$"
)


def _fold(text: str) -> str:
    """Bỏ dấu tiếng Việt, lowercase, gộp khoảng trắng, cắt dấu câu cuối."""
    folded = unicodedata.normalize("NFD", str(text or ""))
    folded = "".join(ch for ch in folded if unicodedata.category(ch) != "Mn")
    folded = folded.replace("đ", "d").replace("Đ", "D").lower()
    return " ".join(folded.split()).strip(" .!?")


def parse(intent_text: str) -> Dict[str, Any] | None:
    """Parse tầng 1. Trả Semantic IR đơn hoặc None (→ LLM)."""
    folded = _fold(intent_text)
    if not folded:
        return None
    if folded in _STOP_TEXTS:
        return {"intent": "stop"}
    if folded in _HOME_TEXTS:
        return {"intent": "go_home"}
    if folded in _GET_POSE_TEXTS:
        return {"intent": "get_pose", "reference_frame": "base_link"}
    if folded in _ALARM_RESET_TEXTS:
        return {"intent": "alarm_reset"}
    wait_match = _WAIT_RE.match(folded)
    if wait_match is not None:
        duration = float(wait_match.group(1))
        if 0.0 < duration <= _WAIT_MAX_SEC:
            return {"intent": "wait", "wait_duration_sec": duration}
    if folded in _BAT_KHI_NEN_TEXTS:
        return {"intent": "io_set", "io_address": 10017, "io_value": 1}
    if folded in _TAT_KHI_NEN_TEXTS:
        return {"intent": "io_set", "io_address": 10017, "io_value": 0}
    return None


def parse_factory_task(intent_text: str) -> Dict[str, Any] | None:
    """Parse tầng 1.5: lệnh deterministic → FactoryTask, không cần LLM.

    Returns a FactoryTask dict (task_type="factory_task") when matched,
    None otherwise (→ LLM task planner).
    """
    folded = _fold(intent_text)
    if not folded:
        return None

    # Do not intercept compound instructions; let LLM planner handle sequences/fallbacks.
    if any(kw in f" {folded} " for kw in {" then ", " and ", " with ", " otherwise ", " roi ", " va "}):
        return None

    # "go to [object]" / "đi tới [object]"
    match = _MOVE_TO_OBJECT_RE.match(folded)
    if match:
        object_ref = match.group(1).strip()
        # Do not intercept cartesian coordinates; let them fall through to the LLM planner
        if "vi tri" in object_ref or "toa do" in object_ref or re.search(r"[xyz]\s*-?\d+", object_ref):
            return None
        # Do not intercept named poses; let them fall through to the LLM planner
        if object_ref and object_ref not in {
            "a", "b", "pose a", "pose b", "posea", "poseb", 
            "ready", "home", "diem a", "diem b", 
            "first pose", "the first pose"
        }:
            return {
                "task_type": "factory_task",
                "version": "1.0",
                "task_id": f"move-to-{object_ref.replace(' ', '-')}",
                "mode": "supervised_hardware",
                "operator_summary": f"Move to {object_ref}",
                "limits": {"velocity_scale": 0.06, "acceleration_scale": 0.06},
                "replan_policy": {
                    "max_replans": 1,
                    "on_world_change": "replan_before_motion",
                },
                "root": {
                    "type": "sequence",
                    "children": [
                        {
                            "type": "observe",
                            "name": "observe_station",
                            "args": {"region": "station"},
                        },
                        {
                            "type": "skill",
                            "name": "move_to_object",
                            "args": {
                                "object_ref": object_ref,
                                "pose": "approach",
                            },
                        },
                    ],
                },
            }

    # "pick [object]" / "nhặt [object]"
    match = _PICK_OBJECT_RE.match(folded)
    if match:
        object_ref = match.group(1).strip()
        if object_ref:
            return {
                "task_type": "factory_task",
                "version": "1.0",
                "task_id": f"pick-{object_ref.replace(' ', '-')}",
                "mode": "supervised_hardware",
                "operator_summary": f"Pick {object_ref}",
                "limits": {"velocity_scale": 0.06, "acceleration_scale": 0.06},
                "replan_policy": {
                    "max_replans": 1,
                    "on_world_change": "replan_before_motion",
                },
                "root": {
                    "type": "sequence",
                    "children": [
                        {
                            "type": "observe",
                            "name": "observe_station",
                            "args": {"region": "station"},
                        },
                        {
                            "type": "skill",
                            "name": "pick_object",
                            "args": {"object": object_ref},
                        },
                    ],
                },
            }

    return None
