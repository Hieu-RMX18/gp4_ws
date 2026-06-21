"""Tầng 1 của pipeline parse: lệnh deterministic an toàn, không cần LLM.

Spec: docs/superpowers/specs/2026-06-10-llm-gateway-factory-pipeline-design.md §3.
Hợp đồng:
  - Chỉ 5 lệnh: stop / go home / get pose / alarm reset / wait N giây.
  - Khớp NGUYÊN CÂU (sau khi fold dấu tiếng Việt, lowercase, gộp khoảng trắng).
  - Trả Semantic IR đơn (dict) khi khớp, None khi không — None nghĩa là
    "chuyển cho task_planner (LLM)", không bao giờ đoán mò.
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
    return None
