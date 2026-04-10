from __future__ import annotations

import sqlite3
import unittest
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory

from pydantic import ValidationError

from hmi.backend.api.contracts import (
    ConnectionStateResponseModel,
    HMI_STREAM_EVENT_ADAPTER,
    HmiStateSnapshotModel,
    LeaseStateResponseModel,
    RuntimeStateResponseModel,
)
from hmi.backend.domain.models import (
    BridgeConnection,
    ConnectionHealth,
    JointPosition,
    RobotStatusSnapshot,
    TelemetryFreshnessState,
    TelemetrySourceSnapshot,
    RuntimeMode,
    RuntimeSnapshot,
    SystemRuntimeState,
)
from hmi.backend.services.audit_service import AuditService
from hmi.backend.services.session_lock_service import SessionLockService
from hmi.backend.services.telemetry_bridge_service import SNAPSHOT_SCHEMA_VERSION, TelemetryBridgeService


def build_connections(
    *,
    ros: ConnectionHealth,
    moveit: ConnectionHealth,
    llm: ConnectionHealth,
    motoros2: ConnectionHealth,
) -> list[BridgeConnection]:
    return [
        BridgeConnection(name='ros2', label='ROS 2', health=ros),
        BridgeConnection(name='moveit2', label='MoveIt 2', health=moveit),
        BridgeConnection(name='llm', label='LLM', health=llm),
        BridgeConnection(name='motoros2', label='MotoROS2', health=motoros2),
    ]


class FakeRosAdapter:
    def __init__(
        self,
        *,
        runtime: RuntimeSnapshot,
        connections: list[BridgeConnection],
        joints: list[JointPosition] | None = None,
        source_statuses: list[TelemetrySourceSnapshot] | None = None,
    ) -> None:
        self._runtime = runtime
        self._connections = list(connections)
        self._joints = list(joints or [])
        self._source_statuses = list(source_statuses or [])
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def read_runtime_snapshot(self) -> RuntimeSnapshot:
        return deepcopy(self._runtime)

    def read_connections(self) -> list[BridgeConnection]:
        return deepcopy(self._connections)

    def read_joint_positions(self) -> list[JointPosition]:
        return deepcopy(self._joints)

    def read_source_statuses(self) -> list[TelemetrySourceSnapshot]:
        return deepcopy(self._source_statuses)

    def set_runtime(
        self,
        system_state: SystemRuntimeState,
        *,
        blocking: bool,
        status_text: str,
        mode: RuntimeMode = RuntimeMode.UNKNOWN,
    ) -> None:
        self._runtime = RuntimeSnapshot(
            system_state=system_state,
            blocking=blocking,
            status_text=status_text,
            mode=mode,
            robot_status=RobotStatusSnapshot(readiness_message=status_text),
        )

    def set_connections(self, connections: list[BridgeConnection]) -> None:
        self._connections = list(connections)

    def set_source_statuses(self, source_statuses: list[TelemetrySourceSnapshot]) -> None:
        self._source_statuses = list(source_statuses)


class TelemetryBridgeV1Tests(unittest.IsolatedAsyncioTestCase):
    def _build_service(
        self,
        adapter: FakeRosAdapter,
        db_path: Path,
        *,
        poll_interval_sec: float = 0.05,
    ) -> TelemetryBridgeService:
        return TelemetryBridgeService(
            audit_service=AuditService(db_path),
            session_lock_service=SessionLockService(),
            ros_adapter=adapter,
            poll_interval_sec=poll_interval_sec,
        )

    async def test_snapshot_shapes_from_service(self) -> None:
        with TemporaryDirectory() as temp_dir:
            adapter = FakeRosAdapter(
                runtime=RuntimeSnapshot(
                    system_state=SystemRuntimeState.LOST_CONN,
                    blocking=True,
                    status_text='No fresh ROS telemetry received from configured read-only topics.',
                    mode=RuntimeMode.UNKNOWN,
                ),
                connections=build_connections(
                    ros=ConnectionHealth.DOWN,
                    moveit=ConnectionHealth.DOWN,
                    llm=ConnectionHealth.DOWN,
                    motoros2=ConnectionHealth.DOWN,
                ),
                joints=[
                    JointPosition(name='joint_1_s', position_deg=0.0),
                    JointPosition(name='joint_2_l', position_deg=10.0),
                ],
            )
            service = self._build_service(adapter, Path(temp_dir) / 'audit.sqlite3')
            await service.start()
            self.addAsyncCleanup(service.stop)

            snapshot_payload = service.get_snapshot('session-a', 'operator-a')
            runtime_payload = service.get_runtime_state('session-a', 'operator-a')
            connection_payload = service.get_connection_state()
            lease_payload = service.get_lease_state('session-a', 'operator-a')

            self.assertEqual(snapshot_payload['schemaVersion'], SNAPSHOT_SCHEMA_VERSION)
            self.assertEqual(snapshot_payload['runtime']['systemState'], 'LOST_CONN')
            self.assertTrue(snapshot_payload['runtime']['blocking'])
            self.assertTrue(snapshot_payload['capabilities']['readOnly'])
            self.assertEqual(snapshot_payload['telemetryState'], 'unavailable')

            self.assertEqual(runtime_payload['schemaVersion'], SNAPSHOT_SCHEMA_VERSION)
            self.assertEqual(connection_payload['schemaVersion'], SNAPSHOT_SCHEMA_VERSION)
            self.assertEqual(lease_payload['schemaVersion'], SNAPSHOT_SCHEMA_VERSION)
            self.assertEqual(lease_payload['lease']['role'], 'observer')
            self.assertFalse(lease_payload['lease']['ownsControl'])
            self.assertFalse(lease_payload['capabilities']['canSubmitCommands'])

    async def test_contract_models_validate_payloads_and_fail_on_bad_schema(self) -> None:
        with TemporaryDirectory() as temp_dir:
            adapter = FakeRosAdapter(
                runtime=RuntimeSnapshot(
                    system_state=SystemRuntimeState.NORMAL,
                    blocking=False,
                    status_text='Telemetry bridge connected and no blocking state is active.',
                    mode=RuntimeMode.SIM,
                ),
                connections=build_connections(
                    ros=ConnectionHealth.HEALTHY,
                    moveit=ConnectionHealth.DEGRADED,
                    llm=ConnectionHealth.HEALTHY,
                    motoros2=ConnectionHealth.DEGRADED,
                ),
                source_statuses=[
                    TelemetrySourceSnapshot(
                        name='readiness',
                        label='HW readiness',
                        topic='/hw_adapter/ready',
                        last_seen_at=None,
                        freshness_threshold_sec=3.0,
                        freshness_state=TelemetryFreshnessState.STALE,
                    ),
                ],
            )
            service = self._build_service(adapter, Path(temp_dir) / 'audit.sqlite3')
            await service.start()
            self.addAsyncCleanup(service.stop)

            snapshot = service.get_snapshot('session-contract', 'operator-contract')
            runtime = service.get_runtime_state('session-contract', 'operator-contract')
            connection = service.get_connection_state()
            lease = service.get_lease_state('session-contract', 'operator-contract')
            heartbeat = service.get_heartbeat_event()

            HmiStateSnapshotModel.model_validate(snapshot)
            RuntimeStateResponseModel.model_validate(runtime)
            ConnectionStateResponseModel.model_validate(connection)
            LeaseStateResponseModel.model_validate(lease)
            HMI_STREAM_EVENT_ADAPTER.validate_python({'type': 'snapshot', 'snapshot': snapshot})
            HMI_STREAM_EVENT_ADAPTER.validate_python(heartbeat)

            broken_snapshot = deepcopy(snapshot)
            broken_snapshot['schemaVersion'] = 'telemetry.v999'
            with self.assertRaises(ValidationError):
                HmiStateSnapshotModel.model_validate(broken_snapshot)

    async def test_semantic_changes_only_drive_audit_writes(self) -> None:
        with TemporaryDirectory() as temp_dir:
            db_path = Path(temp_dir) / 'audit.sqlite3'
            adapter = FakeRosAdapter(
                runtime=RuntimeSnapshot(
                    system_state=SystemRuntimeState.LOST_CONN,
                    blocking=True,
                    status_text='No fresh ROS telemetry received from configured read-only topics.',
                    mode=RuntimeMode.UNKNOWN,
                ),
                connections=build_connections(
                    ros=ConnectionHealth.DOWN,
                    moveit=ConnectionHealth.DOWN,
                    llm=ConnectionHealth.DOWN,
                    motoros2=ConnectionHealth.DOWN,
                ),
            )
            service = self._build_service(adapter, db_path)
            await service.start()
            self.addAsyncCleanup(service.stop)

            service.get_snapshot('session-a', 'operator-a')
            service.get_runtime_state('session-a', 'operator-a')
            service.get_connection_state()
            service.get_lease_state('session-a', 'operator-a')
            await self._sleep(0.2)

            with sqlite3.connect(db_path) as connection:
                telemetry_count = connection.execute(
                    'SELECT COUNT(*) FROM telemetry_snapshots'
                ).fetchone()[0]
                transition_count = connection.execute(
                    'SELECT COUNT(*) FROM state_transitions'
                ).fetchone()[0]

            self.assertEqual(telemetry_count, 1)
            self.assertEqual(transition_count, 0)

            adapter.set_runtime(
                SystemRuntimeState.SAFETY_BLOCKED,
                blocking=True,
                status_text='Supervisor reported safety blocked.',
            )
            adapter.set_connections(
                build_connections(
                    ros=ConnectionHealth.HEALTHY,
                    moveit=ConnectionHealth.DEGRADED,
                    llm=ConnectionHealth.HEALTHY,
                    motoros2=ConnectionHealth.HEALTHY,
                )
            )
            service.get_snapshot('session-a', 'operator-a')
            await self._sleep(0.2)

            with sqlite3.connect(db_path) as connection:
                telemetry_count_after = connection.execute(
                    'SELECT COUNT(*) FROM telemetry_snapshots'
                ).fetchone()[0]
                transition_count_after = connection.execute(
                    'SELECT COUNT(*) FROM state_transitions'
                ).fetchone()[0]

            self.assertEqual(telemetry_count_after, 2)
            self.assertGreaterEqual(transition_count_after, 2)

    async def test_bursty_updates_keep_latest_queue_payload_bounded(self) -> None:
        with TemporaryDirectory() as temp_dir:
            adapter = FakeRosAdapter(
                runtime=RuntimeSnapshot(
                    system_state=SystemRuntimeState.NORMAL,
                    blocking=False,
                    status_text='steady',
                    mode=RuntimeMode.SIM,
                ),
                connections=build_connections(
                    ros=ConnectionHealth.HEALTHY,
                    moveit=ConnectionHealth.HEALTHY,
                    llm=ConnectionHealth.HEALTHY,
                    motoros2=ConnectionHealth.HEALTHY,
                ),
            )
            service = self._build_service(adapter, Path(temp_dir) / 'audit.sqlite3', poll_interval_sec=0.01)
            await service.start()
            self.addAsyncCleanup(service.stop)
            queue = service.subscribe('session-burst', 'operator-burst')
            self.addCleanup(lambda: service.unsubscribe(queue))
            await self._sleep(0.02)

            for index in range(16):
                next_state = SystemRuntimeState.FAULT if index % 2 else SystemRuntimeState.NORMAL
                adapter.set_runtime(
                    next_state,
                    blocking=next_state != SystemRuntimeState.NORMAL,
                    status_text=f'burst-{index}',
                    mode=RuntimeMode.SIM,
                )
                await self._sleep(0.03)

            self.assertLessEqual(queue.qsize(), 8)
            latest_event = None
            while not queue.empty():
                latest_event = queue.get_nowait()

            self.assertIsNotNone(latest_event)
            assert latest_event is not None
            self.assertEqual(latest_event['type'], 'snapshot')
            self.assertEqual(
                latest_event['snapshot']['runtime']['statusText'],
                'burst-15',
            )

    async def test_runtime_state_blocking_and_lost_conn_transition(self) -> None:
        with TemporaryDirectory() as temp_dir:
            adapter = FakeRosAdapter(
                runtime=RuntimeSnapshot(
                    system_state=SystemRuntimeState.NORMAL,
                    blocking=False,
                    status_text='Telemetry bridge connected and no blocking state is active.',
                    mode=RuntimeMode.SIM,
                ),
                connections=build_connections(
                    ros=ConnectionHealth.HEALTHY,
                    moveit=ConnectionHealth.HEALTHY,
                    llm=ConnectionHealth.HEALTHY,
                    motoros2=ConnectionHealth.DOWN,
                ),
            )
            service = self._build_service(adapter, Path(temp_dir) / 'audit.sqlite3')
            await service.start()
            self.addAsyncCleanup(service.stop)

            baseline = service.get_snapshot('session-b', 'operator-b')
            self.assertEqual(baseline['runtime']['systemState'], 'NORMAL')
            self.assertFalse(baseline['runtime']['blocking'])

            for state, text in (
                (SystemRuntimeState.FAULT, 'Robot controller reports an active fault condition.'),
                (SystemRuntimeState.ESTOP, 'Emergency stop is active according to /yaskawa/robot_status.'),
                (SystemRuntimeState.SAFETY_BLOCKED, 'Supervisor reported safety blocked.'),
            ):
                adapter.set_runtime(state, blocking=True, status_text=text)
                await self._sleep(0.1)
                payload = service.get_runtime_state('session-b', 'operator-b')
                self.assertEqual(payload['runtime']['systemState'], state.value)
                self.assertTrue(payload['runtime']['blocking'])

            adapter.set_runtime(
                SystemRuntimeState.LOST_CONN,
                blocking=True,
                status_text='No fresh ROS telemetry received from configured read-only topics.',
            )
            adapter.set_connections(
                build_connections(
                    ros=ConnectionHealth.DOWN,
                    moveit=ConnectionHealth.DOWN,
                    llm=ConnectionHealth.DOWN,
                    motoros2=ConnectionHealth.DOWN,
                )
            )
            await self._sleep(0.1)
            lost_conn = service.get_snapshot('session-b', 'operator-b')
            self.assertEqual(lost_conn['runtime']['systemState'], 'LOST_CONN')
            self.assertEqual(lost_conn['transportState'], 'disconnected')
            self.assertEqual(lost_conn['lease']['role'], 'observer')
            self.assertFalse(lost_conn['lease']['ownsControl'])

    async def test_stale_subscriber_cleanup(self) -> None:
        with TemporaryDirectory() as temp_dir:
            adapter = FakeRosAdapter(
                runtime=RuntimeSnapshot(
                    system_state=SystemRuntimeState.NORMAL,
                    blocking=False,
                    status_text='ok',
                    mode=RuntimeMode.SIM,
                ),
                connections=build_connections(
                    ros=ConnectionHealth.HEALTHY,
                    moveit=ConnectionHealth.HEALTHY,
                    llm=ConnectionHealth.HEALTHY,
                    motoros2=ConnectionHealth.HEALTHY,
                ),
            )
            service = self._build_service(adapter, Path(temp_dir) / 'audit.sqlite3')
            await service.start()
            self.addAsyncCleanup(service.stop)

            queue = service.subscribe('session-cleanup', 'operator-cleanup')
            self.assertEqual(service.subscriber_count(), 1)
            service.unsubscribe(queue)
            self.assertEqual(service.subscriber_count(), 0)

    async def test_ros_reconnect_while_backend_stays_up(self) -> None:
        with TemporaryDirectory() as temp_dir:
            db_path = Path(temp_dir) / 'audit.sqlite3'
            adapter = FakeRosAdapter(
                runtime=RuntimeSnapshot(
                    system_state=SystemRuntimeState.LOST_CONN,
                    blocking=True,
                    status_text='initial lost',
                    mode=RuntimeMode.UNKNOWN,
                ),
                connections=build_connections(
                    ros=ConnectionHealth.DOWN,
                    moveit=ConnectionHealth.DOWN,
                    llm=ConnectionHealth.DOWN,
                    motoros2=ConnectionHealth.DOWN,
                ),
            )
            service = self._build_service(adapter, db_path)
            await service.start()
            self.addAsyncCleanup(service.stop)

            first = service.get_snapshot('session-reconnect', 'operator-reconnect')
            self.assertEqual(first['runtime']['systemState'], 'LOST_CONN')

            adapter.set_runtime(
                SystemRuntimeState.NORMAL,
                blocking=False,
                status_text='reconnected',
                mode=RuntimeMode.HARDWARE,
            )
            adapter.set_connections(
                build_connections(
                    ros=ConnectionHealth.HEALTHY,
                    moveit=ConnectionHealth.DEGRADED,
                    llm=ConnectionHealth.HEALTHY,
                    motoros2=ConnectionHealth.HEALTHY,
                )
            )
            service.get_snapshot('session-reconnect', 'operator-reconnect')
            await self._sleep(0.1)
            second = service.get_snapshot('session-reconnect', 'operator-reconnect')
            self.assertEqual(second['runtime']['systemState'], 'NORMAL')
            self.assertEqual(second['transportState'], 'connected')

            adapter.set_runtime(
                SystemRuntimeState.LOST_CONN,
                blocking=True,
                status_text='lost again',
                mode=RuntimeMode.UNKNOWN,
            )
            adapter.set_connections(
                build_connections(
                    ros=ConnectionHealth.DOWN,
                    moveit=ConnectionHealth.DOWN,
                    llm=ConnectionHealth.DOWN,
                    motoros2=ConnectionHealth.DOWN,
                )
            )
            service.get_snapshot('session-reconnect', 'operator-reconnect')
            await self._sleep(0.1)
            third = service.get_snapshot('session-reconnect', 'operator-reconnect')
            self.assertEqual(third['runtime']['systemState'], 'LOST_CONN')
            self.assertEqual(third['transportState'], 'disconnected')

            with sqlite3.connect(db_path) as connection:
                runtime_transitions = connection.execute(
                    """
                    SELECT COUNT(*)
                    FROM state_transitions
                    WHERE channel = 'runtime.systemState'
                    """
                ).fetchone()[0]

            self.assertGreaterEqual(runtime_transitions, 2)

    async def _sleep(self, seconds: float) -> None:
        import asyncio

        await asyncio.sleep(seconds)


if __name__ == '__main__':
    unittest.main()
