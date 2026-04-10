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

    def test_sim_mode_demotes_robot_status_and_prefers_joint_state_fallback(self) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.readiness.received_at = now
        adapter._state.readiness.ready = True
        adapter._state.readiness.status_message = 'simulation mode: robot status bypassed'
        adapter._state.joint_received_at = now
        adapter._state.joint_source_topic = '/joint_states'
        adapter._state.joint_topic_received_at['/joint_states'] = now

        source_statuses = {item.name: item for item in adapter.read_source_statuses()}
        runtime = adapter.read_runtime_snapshot()

        self.assertEqual(runtime.mode.value, 'sim')
        self.assertFalse(source_statuses['robot_status'].active)
        self.assertTrue(source_statuses['joint_states_fallback'].preferred)
        self.assertTrue(source_statuses['joint_states_fallback'].active)
        self.assertFalse(source_statuses['joint_states_primary'].preferred)
        self.assertFalse(source_statuses['joint_states_primary'].active)

    def test_llm_event_topics_are_not_freshness_critical_when_idle(self) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.llm.gateway_status_at = now

        source_statuses = {item.name: item for item in adapter.read_source_statuses()}

        self.assertTrue(source_statuses['gateway_status'].active)
        self.assertFalse(source_statuses['llm_debug'].active)
        self.assertFalse(source_statuses['llm_command'].active)

    def test_build_command_payload_maps_cartesian_delta_to_move_rel_in_base_link(self) -> None:
        adapter = WorkspaceRosAdapter()
        payload = adapter._build_command_payload(  # pylint: disable=protected-access
            {
                "action": "move_cartesian_delta",
                "parameters": {
                    "frame": "base_link",
                    "xMm": 0.0,
                    "yMm": 0.0,
                    "zMm": 50.0,
                },
            }
        )

        self.assertEqual(payload["primitive_type"], "MOVE_REL")
        self.assertEqual(payload["reference_frame"], "base_link")
        self.assertAlmostEqual(payload["delta_z"], 0.05)

    def test_build_command_payload_maps_joint_delta_to_absolute_move_joint(self) -> None:
        adapter = WorkspaceRosAdapter()
        payload = adapter._build_command_payload(  # pylint: disable=protected-access
            {
                "action": "move_joint_delta",
                "parameters": {
                    "jointIndexZeroBased": 1,
                    "resolvedTargetDeg": 10.0,
                },
            }
        )

        self.assertEqual(payload["primitive_type"], "MOVE_JOINT")
        self.assertEqual(payload["joint_index"], 1)
        self.assertAlmostEqual(payload["joint_angle"], 10.0 * 3.141592653589793 / 180.0)


if __name__ == '__main__':
    unittest.main()
