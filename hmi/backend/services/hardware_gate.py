from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any

from ..domain.models import HardwareGateChecklistSnapshot, HardwareGateStatusSnapshot


HARDWARE_GATE_ENV = "HMI_ENABLE_HARDWARE_COMMANDS"
DEFAULT_HARDWARE_GATE_PATH = (
    Path(__file__).resolve().parents[2] / "data" / "hardware_gate.json"
)


class HardwareGateEvaluator:
    def __init__(
        self,
        *,
        env_name: str = HARDWARE_GATE_ENV,
        evidence_path: str | Path = DEFAULT_HARDWARE_GATE_PATH,
    ) -> None:
        self._env_name = env_name
        self._evidence_path = Path(evidence_path)

    def evaluate(self) -> HardwareGateStatusSnapshot:
        flag_enabled = os.getenv(self._env_name, "").strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }
        reasons: list[str] = []
        payload, payload_error = self._load_payload()

        if not flag_enabled:
            reasons.append(f"{self._env_name} is not enabled.")
        if payload_error is not None:
            reasons.append(payload_error)

        approved_by = self._string_or_none(
            payload.get("approvedBy") if payload else None
        )
        approved_at = self._string_or_none(
            payload.get("approvedAt") if payload else None
        )
        report_path = self._string_or_none(
            payload.get("reportPath") if payload else None
        )
        report_sha256 = self._string_or_none(
            payload.get("reportSha256") if payload else None
        )
        checklist = self._parse_checklist(payload.get("checklist") if payload else None)
        report_sha256_match = self._report_sha256_matches(report_path, report_sha256)

        if payload is not None:
            if payload.get("approved") is not True:
                reasons.append("hardware gate evidence is missing approved: true.")
            if not approved_by:
                reasons.append("hardware gate evidence is missing approvedBy.")
            if not approved_at:
                reasons.append("hardware gate evidence is missing approvedAt.")
            if checklist is None:
                reasons.append("hardware gate evidence is missing checklist.")
            elif not self._checklist_complete(checklist):
                reasons.append("hardware gate checklist is incomplete.")
            if not report_path:
                reasons.append("hardware gate evidence is missing reportPath.")
            if not report_sha256:
                reasons.append("hardware gate evidence is missing reportSha256.")
            if report_path and report_sha256 and not report_sha256_match:
                reasons.append(
                    "hardware gate report SHA256 does not match the referenced report."
                )

        return HardwareGateStatusSnapshot(
            unlocked=not reasons,
            reasons=reasons,
            flag_enabled=flag_enabled,
            evidence_path=str(self._evidence_path),
            approved_by=approved_by,
            approved_at=approved_at,
            report_path=report_path,
            report_sha256=report_sha256,
            report_sha256_match=report_sha256_match,
            checklist=checklist,
        )

    def _load_payload(self) -> tuple[dict[str, Any] | None, str | None]:
        try:
            raw = self._evidence_path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return None, f"hardware gate evidence missing at {self._evidence_path}."
        except OSError:
            return (
                None,
                f"hardware gate evidence could not be read at {self._evidence_path}.",
            )
        try:
            loaded = json.loads(raw)
        except json.JSONDecodeError:
            return (
                None,
                f"hardware gate evidence at {self._evidence_path} is not valid JSON.",
            )
        if not isinstance(loaded, dict):
            return (
                None,
                f"hardware gate evidence at {self._evidence_path} must be a JSON object.",
            )
        return loaded, None

    def _parse_checklist(self, payload: Any) -> HardwareGateChecklistSnapshot | None:
        if not isinstance(payload, dict):
            return None
        return HardwareGateChecklistSnapshot(
            timing_jitter=bool(payload.get("timingJitter")),
            disconnect_reconnect=bool(payload.get("disconnectReconnect")),
            robot_status_semantics=bool(payload.get("robotStatusSemantics")),
            joint_source_precedence=bool(payload.get("jointSourcePrecedence")),
            audit_visibility=bool(payload.get("auditVisibility")),
        )

    def _checklist_complete(self, checklist: HardwareGateChecklistSnapshot) -> bool:
        return all(
            (
                checklist.timing_jitter,
                checklist.disconnect_reconnect,
                checklist.robot_status_semantics,
                checklist.joint_source_precedence,
                checklist.audit_visibility,
            )
        )

    def _report_sha256_matches(
        self, report_path: str | None, expected_sha256: str | None
    ) -> bool:
        if not report_path or not expected_sha256:
            return False
        candidate = Path(report_path)
        try:
            digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
        except OSError:
            return False
        return digest == expected_sha256

    def _string_or_none(self, value: Any) -> str | None:
        if value is None:
            return None
        text = str(value).strip()
        return text or None
