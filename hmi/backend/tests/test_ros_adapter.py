from __future__ import annotations

from datetime import timedelta
from types import SimpleNamespace
import unittest

from hmi.backend.domain.models import ConnectionHealth, SystemRuntimeState
from hmi.backend.ros.adapter import CONNECTION_FRESHNESS_SEC, WorkspaceRosAdapter


class WorkspaceRosAdapterTests(unittest.TestCase):
    def test_lost_conn_after_startup_grace_without_fresh_topics(self) -> None:
        adapter = WorkspaceRosAdapter()
        adapter._state.start_error = None
        adapter._state.ros_started_at = adapter._now() - timedelta(
            seconds=CONNECTION_FRESHNESS_SEC['ros'] + 0.5
        )

        connections = adapter.read_connections()
        runtime = adapter.read_runtime_snapshot()

        self.assertEqual(connections[0].health, ConnectionHealth.DOWN)
        self.assertEqual(runtime.system_state, SystemRuntimeState.LOST_CONN)

    def test_connecting_during_startup_grace_without_fresh_topics(self) -> None:
        adapter = WorkspaceRosAdapter()
        adapter._state.start_error = None
        adapter._state.ros_started_at = adapter._now()

        connections = adapter.read_connections()

        self.assertEqual(connections[0].health, ConnectionHealth.DEGRADED)

    def test_partial_topic_staleness_only_degrades_affected_sources(self) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.readiness.received_at = now
        adapter._state.readiness.ready = True
        adapter._state.readiness.status_message = 'ready'
        adapter._state.robot_status.received_at = now - timedelta(seconds=10.0)

        connections = adapter.read_connections()
        runtime = adapter.read_runtime_snapshot()
        source_statuses = {item.name: item for item in adapter.read_source_statuses()}

        self.assertEqual(connections[0].health, ConnectionHealth.HEALTHY)
        self.assertEqual(connections[3].health, ConnectionHealth.DEGRADED)
        self.assertEqual(runtime.system_state, SystemRuntimeState.NORMAL)
        self.assertEqual(source_statuses['readiness'].freshness_state.value, 'fresh')
        self.assertEqual(source_statuses['robot_status'].freshness_state.value, 'stale')

    def test_estop_wins_over_fault_and_safety_blocked(self) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.robot_status.received_at = now
        adapter._state.robot_status.e_stopped = True
        adapter._state.robot_status.in_error = True
        adapter._state.readiness.received_at = now
        adapter._state.readiness.ready = False
        adapter._state.readiness.status_message = 'blocked'

        runtime = adapter.read_runtime_snapshot()

        self.assertEqual(runtime.system_state, SystemRuntimeState.ESTOP)

    def test_fault_wins_over_safety_blocked_when_estop_clear(self) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.robot_status.received_at = now
        adapter._state.robot_status.e_stopped = False
        adapter._state.robot_status.in_error = True
        adapter._state.readiness.received_at = now
        adapter._state.readiness.ready = False
        adapter._state.readiness.status_message = 'blocked'

        runtime = adapter.read_runtime_snapshot()

        self.assertEqual(runtime.system_state, SystemRuntimeState.FAULT)

    def test_runtime_callbacks_accept_ros_byte_fields(self) -> None:
        adapter = WorkspaceRosAdapter()

        robot_status_msg = SimpleNamespace(
            mode=SimpleNamespace(val=b"\x02"),
            e_stopped=SimpleNamespace(val=b"\x00"),
            drives_powered=SimpleNamespace(val=b"\x01"),
            motion_possible=SimpleNamespace(val=b"\x01"),
            in_motion=SimpleNamespace(val=b"\x00"),
            in_error=SimpleNamespace(val=b"\x00"),
            error_codes=[],
        )
        alert_msg = SimpleNamespace(
            level=b"\x01",
            message="fixture hold active",
            values=[
                SimpleNamespace(key="reason", value="hold"),
                SimpleNamespace(key="state", value="HOLD"),
            ],
        )

        adapter._on_robot_status(robot_status_msg)
        adapter._on_supervisor_alert(alert_msg)

        self.assertEqual(adapter._state.robot_status.mode, 2)
        self.assertIs(adapter._state.robot_status.e_stopped, False)
        self.assertEqual(adapter._state.supervisor_alert.level, 1)
        self.assertEqual(adapter._state.supervisor_alert.values["reason"], "hold")


if __name__ == '__main__':
    unittest.main()
