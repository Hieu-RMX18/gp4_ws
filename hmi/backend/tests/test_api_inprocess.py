from __future__ import annotations

import asyncio
import os
from pathlib import Path
from tempfile import TemporaryDirectory

import httpx
from hmi.backend.api.app import create_app
from hmi.backend.api.contracts import (
    CommandConfirmRequestModel,
    CommandIntentRequestModel,
    LeaseAcquireRequestModel,
    ServoControlRequestModel,
)
from hmi.backend.domain.models import RuntimeMode, SystemRuntimeState
from hmi.backend.services.audit_service import AuditService
from hmi.backend.services.session_lock_service import SessionLockService
from hmi.backend.services.supervisor_service import (
    ForbiddenActionError,
    SupervisorService,
)
from hmi.backend.services.telemetry_bridge_service import TelemetryBridgeService
from hmi.backend.services.jog_service import DEFAULT_JOG_STATUS
from hmi.backend.tests.test_supervisor_service import (
    AlwaysUnlockedHardwareGate,
    FakeSupervisorAdapter,
)
from pydantic import ValidationError


class NoopJogService:
    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_status(self):
        return DEFAULT_JOG_STATUS


def _build_test_services():
    os.environ.setdefault("GP4_REVIEW_INTENT_TOKEN", "test-review-secret")
    temp_dir = TemporaryDirectory()
    audit = AuditService(Path(temp_dir.name) / "audit.sqlite3")
    session_lock = SessionLockService()
    adapter = FakeSupervisorAdapter()
    telemetry = TelemetryBridgeService(
        audit_service=audit,
        session_lock_service=session_lock,
        ros_adapter=adapter,
        poll_interval_sec=0.01,
    )
    supervisor = SupervisorService(
        audit_service=audit,
        session_lock_service=session_lock,
        ros_adapter=adapter,
        confirmation_window_sec=5.0,
    )
    supervisor.bind_telemetry_service(telemetry)
    return temp_dir, adapter, supervisor


def _build_test_app(*, hardware_gate_evaluator=None):
    os.environ.setdefault("GP4_REVIEW_INTENT_TOKEN", "test-review-secret")
    temp_dir = TemporaryDirectory()
    audit = AuditService(Path(temp_dir.name) / "audit.sqlite3")
    session_lock = SessionLockService()
    adapter = FakeSupervisorAdapter()
    telemetry = TelemetryBridgeService(
        audit_service=audit,
        session_lock_service=session_lock,
        ros_adapter=adapter,
        poll_interval_sec=0.01,
    )
    supervisor = SupervisorService(
        audit_service=audit,
        session_lock_service=session_lock,
        ros_adapter=adapter,
        confirmation_window_sec=5.0,
        hardware_gate_evaluator=hardware_gate_evaluator,
    )
    supervisor.bind_telemetry_service(telemetry)
    app = create_app(
        telemetry_service=telemetry,
        supervisor_service=supervisor,
        jog_pendant_service=NoopJogService(),
    )
    app.state.telemetry_service = telemetry
    app.state.supervisor_service = supervisor
    return temp_dir, adapter, supervisor, app


def _route_endpoint(app, path: str):
    for route in app.routes:
        if getattr(route, "path", None) == path:
            return route.endpoint
    raise AssertionError(f"route not found: {path}")


async def _post_json(app, path: str, payload: dict):
    transport = httpx.ASGITransport(app=app)
    async with httpx.AsyncClient(
        transport=transport, base_url="http://testserver"
    ) as client:
        return await client.post(path, json=payload)


async def _post_json_from_client(app, path: str, payload: dict, client_addr: str):
    transport = httpx.ASGITransport(app=app, client=(client_addr, 50000))
    async with httpx.AsyncClient(
        transport=transport, base_url="http://testserver"
    ) as client:
        return await client.post(path, json=payload)


def test_reviewed_named_pose_sequence_api_confirms_without_tcp_socket():
    temp_dir, adapter, supervisor = _build_test_services()
    try:
        adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={
                "intent": "sequence",
                "steps": [
                    {"intent": "move_named_pose", "pose_name": "poseA"},
                    {"intent": "move_named_pose", "pose_name": "poseB"},
                    {"intent": "go_home"},
                ],
            },
        )
        lease_payload = supervisor.acquire_lease(
            session_id="api-sequence-session",
            operator_id="api-sequence-operator",
        )
        assert lease_payload["accepted"] is True
        lease_token = lease_payload["lease"]["leaseToken"]

        submit_payload = supervisor.submit_intent(
            session_id="api-sequence-session",
            operator_id="api-sequence-operator",
            lease_token=lease_token,
            raw_text="move to poseA then poseB then home",
            mode="sim",
        )
        assert submit_payload["accepted"] is True
        assert submit_payload["jobType"] == "sequence"
        assert [
            step["parsedIntent"]["action"]
            for step in submit_payload["sequence"]["steps"]
        ] == [
            "PTP",
            "PTP",
            "HOME",
        ]
        assert adapter.confirm_calls == []

        confirm_payload = supervisor.confirm_sequence(
            session_id="api-sequence-session",
            operator_id="api-sequence-operator",
            lease_token=lease_token,
            sequence_id=submit_payload["sequenceId"],
            plan_fingerprint=submit_payload["sequence"]["planFingerprint"],
        )
        assert confirm_payload["accepted"] is True
        assert confirm_payload["sequence"]["finalState"] == "SUCCEEDED"
        assert [call["parsed_intent"]["action"] for call in adapter.confirm_calls] == [
            "PTP",
            "PTP",
            "HOME",
        ]
    finally:
        temp_dir.cleanup()


def test_command_intent_contract_rejects_structured_intent_ingress():
    try:
        CommandIntentRequestModel(
            sessionId="session-a",
            operatorId="operator-a",
            leaseToken="lease-token",
            mode="sim",
            structuredIntent={"intent": "go_home"},
        )
    except ValidationError as exc:
        assert "structuredIntent" in str(exc)
    else:
        raise AssertionError("structuredIntent should not be accepted by HMI API")


def test_command_intent_contract_rejects_unknown_mode():
    try:
        CommandIntentRequestModel(
            sessionId="session-a",
            operatorId="operator-a",
            leaseToken="lease-token",
            mode="unknown",
            intentText="go home",
        )
    except ValidationError as exc:
        assert "mode" in str(exc)
    else:
        raise AssertionError("unknown mode should not be accepted by HMI command API")


def test_servo_control_contract_rejects_missing_body_fields():
    try:
        ServoControlRequestModel()
    except ValidationError as exc:
        assert "sessionId" in str(exc)
        assert "operatorId" in str(exc)
        assert "leaseToken" in str(exc)
    else:
        raise AssertionError("servo control request should require the HMI lease body")


def test_servo_routes_enforce_controller_lease_before_adapter_call():
    temp_dir, adapter, _supervisor, app = _build_test_app(
        hardware_gate_evaluator=AlwaysUnlockedHardwareGate()
    )
    try:
        adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        servo_start = _route_endpoint(app, "/api/hmi/servo/start")

        try:
            asyncio.run(
                servo_start(
                    ServoControlRequestModel(
                        sessionId="api-servo-session",
                        operatorId="api-servo-operator",
                        leaseToken=None,
                    )
                )
            )
        except ForbiddenActionError:
            pass
        else:
            raise AssertionError("servo start should require a valid controller lease")

        assert adapter.start_traj_mode_calls == 0
    finally:
        temp_dir.cleanup()


def test_servo_routes_dispatch_after_valid_hardware_gate_and_lease():
    temp_dir, adapter, supervisor, app = _build_test_app(
        hardware_gate_evaluator=AlwaysUnlockedHardwareGate()
    )
    try:
        lease_payload = supervisor.acquire_lease(
            session_id="api-servo-session",
            operator_id="api-servo-operator",
        )
        lease_token = lease_payload["lease"]["leaseToken"]
        adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)

        request_model = ServoControlRequestModel(
            sessionId="api-servo-session",
            operatorId="api-servo-operator",
            leaseToken=lease_token,
        )
        servo_start = _route_endpoint(app, "/api/hmi/servo/start")
        servo_stop = _route_endpoint(app, "/api/hmi/servo/stop")
        start_response = asyncio.run(servo_start(request_model))
        stop_response = asyncio.run(servo_stop(request_model))

        assert start_response["accepted"] is True
        assert stop_response["accepted"] is True
        assert adapter.start_traj_mode_calls == 1
        assert adapter.stop_motion_calls == 1
    finally:
        temp_dir.cleanup()


def test_command_intent_http_rejects_structured_intent_ingress():
    temp_dir, _adapter, _supervisor, app = _build_test_app()
    try:
        response = asyncio.run(_post_json(
            app,
            "/api/hmi/commands/intent",
            {
                "sessionId": "session-a",
                "operatorId": "operator-a",
                "leaseToken": "lease-token",
                "mode": "sim",
                "structuredIntent": {"intent": "go_home"},
            },
        ))

        assert response.status_code == 422
        assert "structuredIntent" in response.text
    finally:
        temp_dir.cleanup()


def test_command_intent_http_rejects_unknown_mode():
    temp_dir, _adapter, _supervisor, app = _build_test_app()
    try:
        response = asyncio.run(_post_json(
            app,
            "/api/hmi/commands/intent",
            {
                "sessionId": "session-a",
                "operatorId": "operator-a",
                "leaseToken": "lease-token",
                "mode": "unknown",
                "intentText": "go home",
            },
        ))

        assert response.status_code == 422
        assert "mode" in response.text
    finally:
        temp_dir.cleanup()


def test_command_intent_route_review_and_confirm_flow():
    temp_dir, adapter, _supervisor, app = _build_test_app()
    try:
        acquire_lease = _route_endpoint(app, "/api/hmi/lease/acquire")
        submit_intent = _route_endpoint(app, "/api/hmi/commands/intent")
        lease_payload = acquire_lease(
            LeaseAcquireRequestModel(
                sessionId="api-command-session",
                operatorId="api-command-operator",
                requestedRole="controller",
            )
        )
        lease_token = lease_payload["lease"]["leaseToken"]

        submit_payload = submit_intent(
            CommandIntentRequestModel(
                sessionId="api-command-session",
                operatorId="api-command-operator",
                leaseToken=lease_token,
                mode="sim",
                intentText="go home",
            )
        )
        assert submit_payload["accepted"] is True
        assert submit_payload["command"]["intentSource"] == "text"
        assert submit_payload["command"]["lifecycleState"] == "NEEDS_CONFIRMATION"
        assert adapter.confirm_calls == []

        confirm_command = _route_endpoint(
            app, "/api/hmi/commands/{command_id}/confirm"
        )
        confirm_payload = confirm_command(
            submit_payload["commandId"],
            CommandConfirmRequestModel(
                sessionId="api-command-session",
                operatorId="api-command-operator",
                leaseToken=lease_token,
                planFingerprint=submit_payload["command"]["planFingerprint"],
            )
        )
        assert confirm_payload["accepted"] is True
        assert confirm_payload["command"]["finalState"] == "SUCCEEDED"
        assert [call["parsed_intent"]["action"] for call in adapter.confirm_calls] == [
            "HOME"
        ]
    finally:
        temp_dir.cleanup()


def test_command_intent_route_fails_closed_when_review_intent_rejects():
    temp_dir, adapter, _supervisor, app = _build_test_app()
    try:
        adapter.set_review_result(
            accepted=False,
            adapter="fake-gateway-review",
            error="review_intent service not ready",
        )
        acquire_lease = _route_endpoint(app, "/api/hmi/lease/acquire")
        submit_intent = _route_endpoint(app, "/api/hmi/commands/intent")
        lease_payload = acquire_lease(
            LeaseAcquireRequestModel(
                sessionId="api-review-down-session",
                operatorId="api-review-down-operator",
                requestedRole="controller",
            )
        )
        lease_token = lease_payload["lease"]["leaseToken"]

        submit_payload = submit_intent(
            CommandIntentRequestModel(
                sessionId="api-review-down-session",
                operatorId="api-review-down-operator",
                leaseToken=lease_token,
                mode="sim",
                intentText="go home",
            )
        )
        assert submit_payload["accepted"] is False
        assert submit_payload["command"]["lifecycleState"] == "REJECTED"
        assert "review_intent" in submit_payload["reason"]
        assert adapter.confirm_calls == []
    finally:
        temp_dir.cleanup()


def test_servo_start_http_rejects_missing_lease_body_fields():
    temp_dir, _adapter, _supervisor, app = _build_test_app(
        hardware_gate_evaluator=AlwaysUnlockedHardwareGate()
    )
    try:
        response = asyncio.run(_post_json(app, "/api/hmi/servo/start", {}))

        assert response.status_code == 422
        assert "sessionId" in response.text
        assert "operatorId" in response.text
        assert "leaseToken" in response.text
    finally:
        temp_dir.cleanup()


def test_state_changing_hmi_routes_reject_non_loopback_clients_by_default():
    temp_dir, _adapter, _supervisor, app = _build_test_app()
    try:
        response = asyncio.run(
            _post_json_from_client(
                app,
                "/api/hmi/lease/acquire",
                {
                    "sessionId": "remote-session",
                    "operatorId": "remote-operator",
                    "requestedRole": "controller",
                },
                "10.0.0.25",
            )
        )

        assert response.status_code == 403
        assert "loopback" in response.text
    finally:
        temp_dir.cleanup()


def test_state_changing_hmi_routes_allow_loopback_clients_by_default():
    temp_dir, _adapter, _supervisor, app = _build_test_app()
    try:
        response = asyncio.run(
            _post_json_from_client(
                app,
                "/api/hmi/lease/acquire",
                {
                    "sessionId": "local-session",
                    "operatorId": "local-operator",
                    "requestedRole": "controller",
                },
                "127.0.0.1",
            )
        )

        assert response.status_code == 200
        assert response.json()["accepted"] is True
    finally:
        temp_dir.cleanup()


def test_servo_http_dispatches_after_valid_hardware_gate_and_lease():
    temp_dir, adapter, supervisor, app = _build_test_app(
        hardware_gate_evaluator=AlwaysUnlockedHardwareGate()
    )
    try:
        lease_payload = supervisor.acquire_lease(
            session_id="api-servo-session",
            operator_id="api-servo-operator",
        )
        lease_token = lease_payload["lease"]["leaseToken"]
        adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        start_response = asyncio.run(_post_json(
            app,
            "/api/hmi/servo/start",
            {
                "sessionId": "api-servo-session",
                "operatorId": "api-servo-operator",
                "leaseToken": lease_token,
            },
        ))
        stop_response = asyncio.run(_post_json(
            app,
            "/api/hmi/servo/stop",
            {
                "sessionId": "api-servo-session",
                "operatorId": "api-servo-operator",
                "leaseToken": lease_token,
            },
        ))

        assert start_response.status_code == 200
        assert stop_response.status_code == 200
        assert start_response.json()["accepted"] is True
        assert stop_response.json()["accepted"] is True
        assert adapter.start_traj_mode_calls == 1
        assert adapter.stop_motion_calls == 1
    finally:
        temp_dir.cleanup()
