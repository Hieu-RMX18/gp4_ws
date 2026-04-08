from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI, Query, WebSocket, WebSocketDisconnect

from .contracts import (
    ConnectionStateResponseModel,
    HmiStateSnapshotModel,
    LeaseStateResponseModel,
    READ_ONLY_STREAM_EVENT_ADAPTER,
    RuntimeStateResponseModel,
)
from ..ros.adapter import WorkspaceRosAdapter
from ..services.audit_service import AuditService
from ..services.session_lock_service import SessionLockService
from ..services.telemetry_bridge_service import TelemetryBridgeService


def build_default_telemetry_service() -> TelemetryBridgeService:
    return TelemetryBridgeService(
        audit_service=AuditService(),
        session_lock_service=SessionLockService(),
        ros_adapter=WorkspaceRosAdapter(),
    )

def create_app(telemetry_service: TelemetryBridgeService | None = None) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        service = telemetry_service or build_default_telemetry_service()
        await service.start()
        app.state.telemetry_service = service
        try:
            yield
        finally:
            await service.stop()

    app = FastAPI(
        title='GP4 HMI Read-Only Telemetry Bridge',
        version='0.1.0',
        lifespan=lifespan,
    )

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

    @app.websocket('/api/hmi/stream')
    async def stream_state(
        websocket: WebSocket,
        session_id: str = Query(...),
        operator_id: str = Query(...),
    ) -> None:
        await websocket.accept()
        service = app.state.telemetry_service
        queue = service.subscribe(session_id=session_id, operator_id=operator_id)

        try:
            await websocket.send_json(
                READ_ONLY_STREAM_EVENT_ADAPTER.validate_python(
                    {
                        'type': 'snapshot',
                        'snapshot': service.get_snapshot(session_id, operator_id),
                    }
                ).model_dump(mode='json')
            )
            while True:
                try:
                    payload = await asyncio.wait_for(
                        queue.get(),
                        timeout=service.heartbeat_interval_sec,
                    )
                except asyncio.TimeoutError:
                    payload = service.get_heartbeat_event()
                await websocket.send_json(
                    READ_ONLY_STREAM_EVENT_ADAPTER.validate_python(payload).model_dump(mode='json')
                )
        except WebSocketDisconnect:
            pass
        finally:
            service.unsubscribe(queue)

    return app


app = create_app()
