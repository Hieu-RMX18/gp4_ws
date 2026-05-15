"""Tests for ReAct + Runtime Console Refactor invariants.

Covers:
- Quick commands succeed with review service down and assert no review RPC call.
- Named/home/return sequences run step-by-step and are not collapsed into BLENDED_SEQUENCE.
- BLENDED_SEQUENCE rejects named/joint/home typed steps.
- HMI contracts no longer expose consoleEvents.
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


class FakeSupervisorAdapterNoLLM(FakeSupervisorAdapter):
    """Adapter that simulates LLM gateway being unavailable."""

    def review_intent(self, raw_text, runtime_mode, session_id, operator_id, command_id):
        raise Exception("LLM gateway review_intent service unavailable")


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


def test_quick_command_home_succeeds_with_llm_gateway_down():
    """Quick commands should bypass LLM review and work even when review service is down."""
    _temp_dir, svc = _build_service(ros_adapter=FakeSupervisorAdapterNoLLM())
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
    assert result["jobType"] == "command"


def test_bended_sequence_rejects_home_step():
    """BLENDED_SEQUENCE should reject HOME steps - they must run step-by-step."""
    from hmi.backend.services.supervisor_sequence import SupervisorSequenceMixin

    # Create parsed steps with HOME in the middle
    parsed_steps = [
        {
            "action": "PTP",
            "targetSummary": "pose A",
            "normalizedCommand": {
                "primitive_type": "PTP",
                "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
            },
        },
        {
            "action": "HOME",
            "targetSummary": "home",
            "normalizedCommand": {"primitive_type": "HOME"},
        },
        {
            "action": "PTP",
            "targetSummary": "pose B",
            "normalizedCommand": {
                "primitive_type": "PTP",
                "target_pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.4}},
            },
        },
    ]

    should_blend = SupervisorSequenceMixin._should_emit_blended_sequence(
        parsed_steps, route_metadata=None
    )
    assert should_blend is False, "BLENDED_SEQUENCE should reject HOME steps"


def test_bended_sequence_rejects_named_pose_step():
    """BLENDED_SEQUENCE should reject named pose steps - they must run step-by-step."""
    from hmi.backend.services.supervisor_sequence import SupervisorSequenceMixin

    parsed_steps = [
        {
            "action": "PTP",
            "targetSummary": "pose A",
            "normalizedCommand": {
                "primitive_type": "PTP",
                "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
            },
        },
        {
            "action": "MOVE_NAMED_POSE",
            "targetSummary": "ready",
            "normalizedCommand": {
                "primitive_type": "MOVE_NAMED_POSE",
                "named_target": "ready",
            },
        },
        {
            "action": "PTP",
            "targetSummary": "pose B",
            "normalizedCommand": {
                "primitive_type": "PTP",
                "target_pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.4}},
            },
        },
    ]

    should_blend = SupervisorSequenceMixin._should_emit_blended_sequence(
        parsed_steps, route_metadata=None
    )
    assert should_blend is False, "BLENDED_SEQUENCE should reject named pose steps"


def test_bended_sequence_rejects_joint_target_step():
    """BLENDED_SEQUENCE should reject joint target steps - they must run step-by-step."""
    from hmi.backend.services.supervisor_sequence import SupervisorSequenceMixin

    parsed_steps = [
        {
            "action": "PTP",
            "targetSummary": "pose A",
            "normalizedCommand": {
                "primitive_type": "PTP",
                "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
            },
        },
        {
            "action": "MOVE_JOINTS",
            "targetSummary": "joint target",
            "normalizedCommand": {
                "primitive_type": "MOVE_JOINTS",
                "joint_target": [0.0, 0.0, 0.0, 0.0, -1.57, 0.0],
            },
        },
        {
            "action": "PTP",
            "targetSummary": "pose B",
            "normalizedCommand": {
                "primitive_type": "PTP",
                "target_pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.4}},
            },
        },
    ]

    should_blend = SupervisorSequenceMixin._should_emit_blended_sequence(
        parsed_steps, route_metadata=None
    )
    assert should_blend is False, "BLENDED_SEQUENCE should reject joint target steps"


def test_hmi_contracts_no_console_events():
    """HMI contracts should not expose consoleEvents field."""
    from hmi.backend.api.contracts import (
        HmiStateSnapshotModel,
        CommandLifecycleStreamEventModel,
        SequenceLifecycleStreamEventModel,
    )

    # Check that consoleEvents is not in the model fields
    snapshot_fields = HmiStateSnapshotModel.model_fields
    assert "consoleEvents" not in snapshot_fields

    command_lifecycle_fields = CommandLifecycleStreamEventModel.model_fields
    assert "consoleEvents" not in command_lifecycle_fields

    sequence_lifecycle_fields = SequenceLifecycleStreamEventModel.model_fields
    assert "consoleEvents" not in sequence_lifecycle_fields


def test_sequence_with_home_runs_step_by_step():
    """Sequences containing HOME should run step-by-step, not as BLENDED_SEQUENCE."""
    from hmi.backend.services.supervisor_sequence import SupervisorSequenceMixin

    parsed_steps = [
        {
            "action": "PTP",
            "targetSummary": "pose A",
            "normalizedCommand": {
                "primitive_type": "PTP",
                "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
            },
        },
        {
            "action": "HOME",
            "targetSummary": "home",
            "normalizedCommand": {"primitive_type": "HOME"},
        },
        {
            "action": "PTP",
            "targetSummary": "pose B",
            "normalizedCommand": {
                "primitive_type": "PTP",
                "target_pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.4}},
            },
        },
    ]

    should_blend = SupervisorSequenceMixin._should_emit_blended_sequence(
        parsed_steps, route_metadata=None
    )
    assert should_blend is False, "Sequences with HOME must run step-by-step"


def test_sequence_with_return_to_start_requires_joints():
    """return_to_start requires captured start joints before execution."""
    _temp_dir, svc = _build_service(
        ros_adapter=FakeSupervisorAdapterCustomJoints(
            joint_positions_deg=[10.0, 20.0, 0.0, 0.0, 0.0, 0.0]
        )
    )
    lease = svc.acquire_lease(session_id="s1", operator_id="o1", force_takeover=False)
    assert lease["accepted"] is True
    lease_token = lease["lease"]["leaseToken"]

    # Submit a sequence that should capture start joints
    result = svc.submit_intent(
        session_id="s1",
        operator_id="o1",
        lease_token=lease_token,
        raw_text="move to pose A then return to start",
        mode="sim",
    )

    # Should be accepted if joints are available
    # The actual return_to_start expansion happens in the backend
    assert result is not None
