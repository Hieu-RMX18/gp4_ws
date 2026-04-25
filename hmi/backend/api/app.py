from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
import os
from typing import Any

from fastapi import FastAPI, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from .contracts import (
    CommandCancelRequestModel,
    CommandConfirmRequestModel,
    CommandIntentRequestModel,
    CommandListResponseModel,
    CommandMutationResponseModel,
    CommandViewModel,
    ConnectionStateResponseModel,
    HMI_STREAM_EVENT_ADAPTER,
    HmiStateSnapshotModel,
    JogCommandRequestModel,
    LeaseAcquireRequestModel,
    LeaseMutationResponseModel,
    LeaseReleaseRequestModel,
    LeaseRenewRequestModel,
    LeaseStateResponseModel,
    ReplayDetailModel,
    RuntimeStateResponseModel,
    SequenceViewModel,
)
from ..ros.adapter import WorkspaceRosAdapter
from ..services.audit_service import AuditService
from ..services.jog_pendant_service import JogPendantService
from ..services.session_lock_service import SessionLockService
from ..services.supervisor_service import SupervisorService, SupervisorServiceError
from ..services.telemetry_bridge_service import TelemetryBridgeService


def _env_flag_enabled(name: str) -> bool:
    return os.getenv(name, "").strip().lower() in {"1", "true", "yes", "on"}


def build_default_services() -> tuple[TelemetryBridgeService, SupervisorService]:
    audit_service = AuditService()
    session_lock_service = SessionLockService()
    ros_adapter = WorkspaceRosAdapter()
    telemetry_service = TelemetryBridgeService(
        audit_service=audit_service,
        session_lock_service=session_lock_service,
        ros_adapter=ros_adapter,
    )
    supervisor_service = SupervisorService(
        audit_service=audit_service,
        session_lock_service=session_lock_service,
        ros_adapter=ros_adapter,
        sim_auto_confirm=_env_flag_enabled("HMI_SIM_AUTO_CONFIRM"),
    )
    supervisor_service.bind_telemetry_service(telemetry_service)
    return telemetry_service, supervisor_service



def create_app(
    telemetry_service: TelemetryBridgeService | None = None,
    supervisor_service: SupervisorService | None = None,
    jog_pendant_service: JogPendantService | None = None,
) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        if telemetry_service is None or supervisor_service is None:
            service_pair = build_default_services()
            service = service_pair[0]
            supervisor = service_pair[1]
        else:
            service = telemetry_service
            supervisor = supervisor_service
            supervisor.bind_telemetry_service(service)

        if jog_pendant_service is None:
            jog_svc = JogPendantService()
        else:
            jog_svc = jog_pendant_service

        await service.start()
        jog_svc.start()
        app.state.telemetry_service = service
        app.state.supervisor_service = supervisor
        app.state.jog_pendant_service = jog_svc
        app.state.ros_adapter = supervisor._ros
        try:
            yield
        finally:
            await service.stop()
            jog_svc.stop()

    app = FastAPI(
        title='GP4 HMI Telemetry + Supervisor Bridge',
        version='0.2.0',
        lifespan=lifespan,
    )

    @app.exception_handler(SupervisorServiceError)
    async def supervisor_error_handler(_request, exc: SupervisorServiceError):
        return JSONResponse(status_code=exc.status_code, content={'detail': str(exc)})

    @app.get('/api/hmi/snapshot', response_model=HmiStateSnapshotModel)
    def get_snapshot(
        session_id: str = Query(...),
        operator_id: str = Query(...),
    ) -> dict:
        return app.state.telemetry_service.get_snapshot(session_id, operator_id)

    @app.get('/api/hmi/runtime-state', response_model=RuntimeStateResponseModel)
    def get_runtime_state(
        session_id: str = Query(...),
        operator_id: str = Query(...),
    ) -> dict:
        return app.state.telemetry_service.get_runtime_state(session_id, operator_id)

    @app.get('/api/hmi/connection-state', response_model=ConnectionStateResponseModel)
    def get_connection_state() -> dict:
        return app.state.telemetry_service.get_connection_state()

    @app.get('/api/hmi/lease-state', response_model=LeaseStateResponseModel)
    def get_lease_state(
        session_id: str = Query(...),
        operator_id: str = Query(...),
    ) -> dict:
        return app.state.telemetry_service.get_lease_state(session_id, operator_id)

    @app.post('/api/hmi/lease/acquire', response_model=LeaseMutationResponseModel)
    def acquire_lease(request: LeaseAcquireRequestModel) -> dict:
        return app.state.supervisor_service.acquire_lease(
            session_id=request.sessionId,
            operator_id=request.operatorId,
            force_takeover=request.forceTakeover,
            takeover_reason=request.takeoverReason,
        )

    @app.post('/api/hmi/lease/renew', response_model=LeaseMutationResponseModel)
    def renew_lease(request: LeaseRenewRequestModel) -> dict:
        return app.state.supervisor_service.renew_lease(
            session_id=request.sessionId,
            operator_id=request.operatorId,
            lease_token=request.leaseToken,
        )

    @app.post('/api/hmi/lease/release', response_model=LeaseMutationResponseModel)
    def release_lease(request: LeaseReleaseRequestModel) -> dict:
        return app.state.supervisor_service.release_lease(
            session_id=request.sessionId,
            operator_id=request.operatorId,
            lease_token=request.leaseToken,
        )

    @app.post('/api/hmi/commands/intent', response_model=CommandMutationResponseModel)
    def submit_intent(request: CommandIntentRequestModel) -> dict:
        return app.state.supervisor_service.submit_intent(
            session_id=request.sessionId,
            operator_id=request.operatorId,
            lease_token=request.leaseToken,
            raw_text=request.intentText,
            structured_intent=request.structuredIntent,
            mode=request.mode,
        )

    @app.get('/api/hmi/commands', response_model=CommandListResponseModel)
    def list_commands(
        session_id: str | None = Query(None),
        operator_id: str | None = Query(None),
        final_state: str | None = Query(None),
        from_timestamp: str | None = Query(None, alias='from'),
        to_timestamp: str | None = Query(None, alias='to'),
        limit: int = Query(25, ge=1, le=200),
    ) -> dict:
        return app.state.supervisor_service.list_commands(
            session_id=session_id,
            operator_id=operator_id,
            final_state=final_state,
            created_from=from_timestamp,
            created_to=to_timestamp,
            limit=limit,
        )

    @app.get('/api/hmi/commands/{command_id}', response_model=CommandViewModel)
    def get_command(command_id: str) -> dict:
        return app.state.supervisor_service.get_command(command_id)

    @app.post('/api/hmi/commands/{command_id}/confirm', response_model=CommandMutationResponseModel)
    def confirm_command(command_id: str, request: CommandConfirmRequestModel) -> dict:
        return app.state.supervisor_service.confirm_command(
            session_id=request.sessionId,
            operator_id=request.operatorId,
            lease_token=request.leaseToken,
            command_id=command_id,
            plan_fingerprint=request.planFingerprint,
        )

    @app.post('/api/hmi/commands/{command_id}/cancel', response_model=CommandMutationResponseModel)
    def cancel_command(command_id: str, request: CommandCancelRequestModel) -> dict:
        return app.state.supervisor_service.cancel_command(
            session_id=request.sessionId,
            operator_id=request.operatorId,
            lease_token=request.leaseToken,
            command_id=command_id,
            reason=request.reason,
        )

    @app.get('/api/hmi/sequences/{sequence_id}', response_model=SequenceViewModel)
    def get_sequence(sequence_id: str) -> dict:
        return app.state.supervisor_service.get_sequence(sequence_id)

    @app.post('/api/hmi/sequences/{sequence_id}/confirm', response_model=CommandMutationResponseModel)
    def confirm_sequence(sequence_id: str, request: CommandConfirmRequestModel) -> dict:
        return app.state.supervisor_service.confirm_sequence(
            session_id=request.sessionId,
            operator_id=request.operatorId,
            lease_token=request.leaseToken,
            sequence_id=sequence_id,
            plan_fingerprint=request.planFingerprint,
        )

    @app.post('/api/hmi/sequences/{sequence_id}/cancel', response_model=CommandMutationResponseModel)
    def cancel_sequence(sequence_id: str, request: CommandCancelRequestModel) -> dict:
        return app.state.supervisor_service.cancel_sequence(
            session_id=request.sessionId,
            operator_id=request.operatorId,
            lease_token=request.leaseToken,
            sequence_id=sequence_id,
            reason=request.reason,
        )

    @app.get('/api/hmi/replay', response_model=CommandListResponseModel)
    def list_replay(
        session_id: str | None = Query(None),
        operator_id: str | None = Query(None),
        final_state: str | None = Query(None),
        from_timestamp: str | None = Query(None, alias='from'),
        to_timestamp: str | None = Query(None, alias='to'),
        limit: int = Query(25, ge=1, le=200),
    ) -> dict:
        return app.state.supervisor_service.list_replay(
            session_id=session_id,
            operator_id=operator_id,
            final_state=final_state,
            created_from=from_timestamp,
            created_to=to_timestamp,
            limit=limit,
        )

    @app.get('/api/hmi/replay/{command_id}', response_model=ReplayDetailModel)
    def replay_detail(command_id: str) -> dict:
        return app.state.supervisor_service.replay_detail(command_id)

    # ── Jog Pendant Endpoints ───────────────────────────────────────────────

    # ── Servo Control Endpoints ────────────────────────────────────────────

    @app.post('/api/hmi/servo/start')
    def servo_start() -> dict:
        adapter = app.state.ros_adapter
        if adapter is None:
            return {'accepted': False, 'message': 'ROS adapter not available'}
        return adapter.start_traj_mode()

    @app.post('/api/hmi/servo/stop')
    def servo_stop() -> dict:
        adapter = app.state.ros_adapter
        if adapter is None:
            return {'accepted': False, 'message': 'ROS adapter not available'}
        return adapter.stop_motion()

    @app.post('/api/hmi/jog/activate')
    def jog_activate() -> dict:
        svc = app.state.jog_pendant_service
        accepted, message = svc.activate_bridge()
        return {'accepted': accepted, 'message': message}

    @app.post('/api/hmi/jog/deactivate')
    def jog_deactivate() -> dict:
        svc = app.state.jog_pendant_service
        accepted, message = svc.deactivate_bridge()
        return {'accepted': accepted, 'message': message}

    @app.post('/api/hmi/jog/command')
    def jog_command(request: JogCommandRequestModel) -> dict:
        svc = app.state.jog_pendant_service
        ok, reason = svc.send_jog_command(
            joint_index=request.jointIndex,
            direction=request.direction,
            mode=request.mode,
            velocity_scale=request.velocityScale,
            step_degrees=request.stepDegrees,
        )
        return {'accepted': ok, 'message': reason}

    @app.websocket('/api/hmi/stream')
    async def stream_state(
        websocket: WebSocket,
        session_id: str = Query(...),
        operator_id: str = Query(...),
    ) -> None:
        await websocket.accept()
        service = app.state.telemetry_service
        jog_svc = app.state.jog_pendant_service
        telemetry_queue = service.subscribe(session_id=session_id, operator_id=operator_id)
        jog_queue = jog_svc.subscribe()

        try:
            await websocket.send_json(
                HMI_STREAM_EVENT_ADAPTER.validate_python(
                    {
                        'type': 'snapshot',
                        'snapshot': service.get_snapshot(session_id, operator_id),
                    }
                ).model_dump(mode='json')
            )
            # Send initial jog status
            jog_status = jog_svc.get_status()
            await websocket.send_json({
                'type': 'jog_bridge_status',
                'jogBridgeStatus': {
                    'state': jog_status.state.value,
                    'pointsQueued': jog_status.points_queued,
                    'effectiveHz': jog_status.effective_hz,
                    'robotReady': jog_status.robot_ready,
                    'servoActive': jog_status.servo_active,
                    'bridgeActive': jog_status.bridge_active,
                    'lastError': jog_status.last_error,
                    'rejectionReason': jog_status.rejection_reason,
                },
            })

            while True:
                import asyncio as _asyncio
                done, pending = await _asyncio.wait(
                    [
                        _asyncio.create_task(_async_queue_get(telemetry_queue)),
                        _asyncio.create_task(_async_queue_get(jog_queue)),
                    ],
                    timeout=service.heartbeat_interval_sec,
                    return_when=_asyncio.FIRST_COMPLETED,
                )

                for task in pending:
                    task.cancel()

                if not done:
                    payload = service.get_heartbeat_event()
                    await websocket.send_json(
                        HMI_STREAM_EVENT_ADAPTER.validate_python(payload).model_dump(mode='json')
                    )
                else:
                    for task in done:
                        payload = task.result()
                        if payload is not None:
                            await websocket.send_json(
                                HMI_STREAM_EVENT_ADAPTER.validate_python(payload).model_dump(mode='json')
                            )

        except WebSocketDisconnect:
            pass
        finally:
            service.unsubscribe(telemetry_queue)
            jog_svc.unsubscribe(jog_queue)

    return app


async def _async_queue_get(q: Any) -> Any:
    """Get from asyncio.Queue or queue.Queue without blocking the event loop."""
    try:
        if isinstance(q, asyncio.Queue):
            return await asyncio.wait_for(q.get(), timeout=60.0)

        # JogPendantService uses stdlib queue.Queue from a ROS spin thread.
        # Poll non-blocking so WebSocket handling stays responsive.
        import queue as thread_queue
        import time

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            try:
                return q.get_nowait()
            except thread_queue.Empty:
                await asyncio.sleep(0.05)
        return None
    except asyncio.TimeoutError:
        return None


app = create_app()  # noqa: N816 — exported at module level for uvicorn import
