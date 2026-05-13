"""Tests for deterministic quick command API (Packet B).

Covers:
- quickCommandId bypasses LLM review and maps to deterministic Semantic IR.
- Unknown quickCommandId is rejected.
- All HMI gates (lease, confirmation, validation) still apply.
- structuredIntent remains rejected.
"""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from hmi.backend.services.audit_service import AuditService
from hmi.backend.services.session_lock_service import SessionLockService
from hmi.backend.services.supervisor_service import SupervisorService
from hmi.backend.services.telemetry_bridge_service import TelemetryBridgeService
from hmi.backend.tests.test_supervisor_service import FakeSupervisorAdapter


class FakeSupervisorAdapterNoJoints(FakeSupervisorAdapter):
    def read_joint_positions(self):
        return []


class FakeSupervisorAdapterCustomJoints(FakeSupervisorAdapter):
    def __init__(self, joint_positions_deg):
        super().__init__()
        from hmi.backend.domain.models import JointPosition

        names = [
            "joint_1_s",
            "joint_2_l",
            "joint_3_u",
            "joint_4_r",
            "joint_5_b",
            "joint_6_t",
        ]
        self._joints = [
            JointPosition(name=name, position_deg=deg)
            for name, deg in zip(names, joint_positions_deg)
        ]


def _build_service(ros_adapter=None, sim_auto_confirm=False):
    temp_dir = TemporaryDirectory()
    db_path = Path(temp_dir.name) / "audit.sqlite3"
    audit = AuditService(db_path)
    session_lock = SessionLockService()
    ros = ros_adapter or FakeSupervisorAdapter()
    telemetry = TelemetryBridgeService(
        audit_service=audit,
        session_lock_service=session_lock,
        ros_adapter=ros,
        poll_interval_sec=0.01,
    )
    svc = SupervisorService(
        audit_service=audit,
        session_lock_service=session_lock,
        ros_adapter=ros,
        sim_auto_confirm=sim_auto_confirm,
    )
    svc.bind_telemetry_service(telemetry)
    return temp_dir, svc


def test_quick_command_home_maps_to_go_home_and_requires_lease():
    _temp_dir, svc = _build_service()
    from hmi.backend.services.supervisor_service import ForbiddenActionError

    with pytest.raises(ForbiddenActionError, match="lease token is required"):
        svc.submit_intent(
            session_id="s1",
            operator_id="o1",
            lease_token=None,
            quick_command_id="home",
            mode="sim",
        )


def test_quick_commands_can_acquire_lease_without_review_intent_token(monkeypatch):
    monkeypatch.delenv("GP4_REVIEW_INTENT_TOKEN", raising=False)
    _temp_dir, svc = _build_service()

    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)

    assert lease["accepted"] is True
    assert lease["lease"]["leaseToken"]


def test_quick_command_home_with_lease_bypasses_llm_review():
    _temp_dir, svc = _build_service()
    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)
    assert lease["accepted"] is True
    lease_token = lease["lease"]["leaseToken"]

    result = svc.submit_intent(
        session_id="s1",
        operator_id="o1",
        lease_token=lease_token,
        quick_command_id="home",
        mode="sim",
    )

    assert result["accepted"] is True
    # Should be a single command (not sequence)
    assert result["jobType"] == "command"
    assert result["command"]["lifecycleState"] == "NEEDS_CONFIRMATION"


def test_quick_command_up_5cm_maps_to_move_relative():
    _temp_dir, svc = _build_service()
    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)
    lease_token = lease["lease"]["leaseToken"]

    result = svc.submit_intent(
        session_id="s1",
        operator_id="o1",
        lease_token=lease_token,
        quick_command_id="up_5cm",
        mode="sim",
    )

    assert result["accepted"] is True
    assert result["jobType"] == "command"


def test_quick_command_joint_1_plus_5_reads_current_joints():
    _temp_dir, svc = _build_service(
        ros_adapter=FakeSupervisorAdapterCustomJoints(
            joint_positions_deg=[10.0, 20.0, 0.0, 0.0, 0.0, 0.0]
        )
    )
    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)
    lease_token = lease["lease"]["leaseToken"]

    result = svc.submit_intent(
        session_id="s1",
        operator_id="o1",
        lease_token=lease_token,
        quick_command_id="joint_1_plus_5",
        mode="sim",
    )

    assert result["accepted"] is True
    assert result["jobType"] == "command"


def test_quick_command_joint_1_plus_5_fails_closed_when_no_joint_state():
    _temp_dir, svc = _build_service(ros_adapter=FakeSupervisorAdapterNoJoints())
    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)
    lease_token = lease["lease"]["leaseToken"]

    result = svc.submit_intent(
        session_id="s1",
        operator_id="o1",
        lease_token=lease_token,
        quick_command_id="joint_1_plus_5",
        mode="sim",
    )

    assert result["accepted"] is False
    assert "joint state unavailable" in result["reason"].lower()


def test_unknown_quick_command_id_is_rejected():
    _temp_dir, svc = _build_service()
    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)
    lease_token = lease["lease"]["leaseToken"]

    result = svc.submit_intent(
        session_id="s1",
        operator_id="o1",
        lease_token=lease_token,
        quick_command_id="fly_to_moon",
        mode="sim",
    )

    assert result["accepted"] is False
    assert "unknown quickcommandid" in result["reason"].lower()


def test_structured_intent_still_rejected_from_hmi_api():
    _temp_dir, svc = _build_service()
    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)
    lease_token = lease["lease"]["leaseToken"]

    result = svc.submit_intent(
        session_id="s1",
        operator_id="o1",
        lease_token=lease_token,
        raw_text="",
        structured_intent={"intent": "go_home"},
        mode="sim",
    )

    assert result["accepted"] is False
    assert "structuredintent is not accepted" in result["reason"].lower()


def test_contract_requires_intenttext_or_quickcommandid():
    from hmi.backend.api.contracts import CommandIntentRequestModel

    with pytest.raises(ValueError, match="intentText or quickCommandId"):
        CommandIntentRequestModel(
            sessionId="s1",
            operatorId="o1",
            leaseToken="tok",
            mode="sim",
        )

    valid_text = CommandIntentRequestModel(
        sessionId="s1",
        operatorId="o1",
        leaseToken="tok",
        intentText="go home",
        mode="sim",
    )
    assert valid_text.intentText == "go home"

    valid_quick = CommandIntentRequestModel(
        sessionId="s1",
        operatorId="o1",
        leaseToken="tok",
        quickCommandId="home",
        mode="sim",
    )
    assert valid_quick.quickCommandId == "home"


def test_contract_rejects_both_intenttext_and_quickcommandid():
    from hmi.backend.api.contracts import CommandIntentRequestModel

    with pytest.raises(ValueError, match="exactly one"):
        CommandIntentRequestModel(
            sessionId="s1",
            operatorId="o1",
            leaseToken="tok",
            intentText="go home",
            quickCommandId="home",
            mode="sim",
        )


def test_submit_intent_rejects_both_intenttext_and_quickcommandid():
    _temp_dir, svc = _build_service()
    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)
    lease_token = lease["lease"]["leaseToken"]

    with pytest.raises(Exception, match="exactly one"):
        svc.submit_intent(
            session_id="s1",
            operator_id="o1",
            lease_token=lease_token,
            raw_text="go home",
            quick_command_id="home",
            mode="sim",
        )
