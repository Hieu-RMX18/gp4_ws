from __future__ import annotations

from datetime import timedelta
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from hmi.backend.domain.models import ConnectionHealth, SystemRuntimeState
from hmi.backend.ros.adapter import CONNECTION_FRESHNESS_SEC, WorkspaceRosAdapter


class _CompletedFuture:
    def __init__(self, result):
        self._result = result

    def done(self) -> bool:
        return True

    def result(self):
        return self._result


class _FakePoseClient:
    def __init__(self, response, *, ready: bool = True) -> None:
        self._response = response
        self._ready = ready
        self.requests = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        _ = timeout_sec
        return self._ready

    def call_async(self, request):
        self.requests.append(request)
        return _CompletedFuture(self._response)


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

    def test_build_command_payload_uses_normalized_command_passthrough(self) -> None:
        adapter = WorkspaceRosAdapter()
        payload = adapter._build_command_payload(  # pylint: disable=protected-access
            {
                "action": "MOVE_REL",
                "normalizedCommand": {
                    "primitive_type": "MOVE_REL",
                    "delta_x": 0.01,
                    "delta_y": 0.0,
                    "delta_z": 0.0,
                    "reference_frame": "base_link",
                },
            }
        )
        self.assertEqual(payload["primitive_type"], "MOVE_REL")
        self.assertAlmostEqual(payload["delta_x"], 0.01)

    def test_build_command_payload_passthrough_preserves_wait_and_io_fields(self) -> None:
        adapter = WorkspaceRosAdapter()
        wait_payload = adapter._build_command_payload(  # pylint: disable=protected-access
            {
                "action": "WAIT",
                "normalizedCommand": {
                    "primitive_type": "WAIT",
                    "wait_duration_sec": 2.0,
                    "reference_frame": "base_link",
                },
            }
        )
        self.assertEqual(wait_payload["primitive_type"], "WAIT")
        self.assertAlmostEqual(wait_payload["wait_duration_sec"], 2.0)
        self.assertEqual(wait_payload["reference_frame"], "base_link")

        io_payload = adapter._build_command_payload(  # pylint: disable=protected-access
            {
                "action": "IO_SET",
                "normalizedCommand": {
                    "primitive_type": "IO_SET",
                    "io_address": 10010,
                    "io_value": 1,
                    "reference_frame": "base_link",
                },
            }
        )
        self.assertEqual(io_payload["primitive_type"], "IO_SET")
        self.assertEqual(io_payload["io_address"], 10010)
        self.assertEqual(io_payload["io_value"], 1)
        self.assertEqual(io_payload["reference_frame"], "base_link")

    def test_build_command_payload_passthrough_preserves_joint_and_pose_motion_fields(self) -> None:
        adapter = WorkspaceRosAdapter()
        move_joint_payload = adapter._build_command_payload(  # pylint: disable=protected-access
            {
                "action": "MOVE_JOINT",
                "normalizedCommand": {
                    "primitive_type": "MOVE_JOINT",
                    "joint_index": 2,
                    "joint_angle": 0.5,
                    "velocity_scale": 0.06,
                    "acceleration_scale": 0.06,
                    "planner_id": "PILZ_PTP",
                    "require_approval": False,
                },
            }
        )
        self.assertEqual(move_joint_payload["primitive_type"], "MOVE_JOINT")
        self.assertEqual(move_joint_payload["joint_index"], 2)
        self.assertAlmostEqual(move_joint_payload["joint_angle"], 0.5)
        self.assertAlmostEqual(move_joint_payload["velocity_scale"], 0.06)
        self.assertAlmostEqual(move_joint_payload["acceleration_scale"], 0.06)
        self.assertEqual(move_joint_payload["planner_id"], "PILZ_PTP")
        self.assertFalse(move_joint_payload["require_approval"])

        lin_payload = adapter._build_command_payload(  # pylint: disable=protected-access
            {
                "action": "LIN",
                "normalizedCommand": {
                    "primitive_type": "LIN",
                    "target_pose": {
                        "position": {"x": 0.30, "y": 0.0, "z": 0.35},
                        "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
                    },
                    "reference_frame": "base_link",
                    "velocity_scale": 0.05,
                    "acceleration_scale": 0.04,
                    "planner_id": "PILZ_LIN",
                    "require_approval": True,
                },
            }
        )
        self.assertEqual(lin_payload["primitive_type"], "LIN")
        self.assertEqual(lin_payload["reference_frame"], "base_link")
        self.assertIn("target_pose", lin_payload)
        self.assertAlmostEqual(lin_payload["velocity_scale"], 0.05)
        self.assertAlmostEqual(lin_payload["acceleration_scale"], 0.04)
        self.assertEqual(lin_payload["planner_id"], "PILZ_LIN")
        self.assertFalse(lin_payload["require_approval"])

    def test_cartesian_path_uses_last_waypoint_as_target_pose_for_ros_dispatch(self) -> None:
        adapter = WorkspaceRosAdapter()
        payload = {
            "primitive_type": "CARTESIAN_PATH",
            "reference_frame": "base_link",
            "waypoints": [
                {
                    "position": {"x": 0.30, "y": 0.00, "z": 0.31},
                    "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
                },
                {
                    "position": {"x": 0.32, "y": 0.00, "z": 0.31},
                    "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
                },
            ],
        }

        target_pose = adapter._cartesian_path_target_pose(payload)  # pylint: disable=protected-access

        self.assertEqual(target_pose, payload["waypoints"][-1])

    def test_hardware_preflight_detects_missing_primary_joint_source(self) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.readiness.received_at = now
        adapter._state.readiness.ready = True
        adapter._state.readiness.status_message = 'hardware ready'
        adapter._state.robot_status.received_at = now
        adapter._state.robot_status.e_stopped = False
        adapter._state.robot_status.in_error = False
        adapter._state.joint_received_at = now
        adapter._state.joint_source_topic = '/joint_states'
        adapter._state.joint_topic_received_at['/joint_states'] = now
        adapter._state.validate_command_ready = True
        adapter._state.execute_motion_ready = True
        adapter._state.validate_command_ready_at = now
        adapter._state.execute_motion_ready_at = now
        preflight = adapter.evaluate_execution_preflight(target_mode='hardware')
        self.assertFalse(preflight["accepted"])
        self.assertTrue(
            any("joint_states_primary" in reason for reason in preflight["reasons"]),
            msg=preflight,
        )

    def test_get_current_pose_returns_serialized_pose(self) -> None:
        adapter = WorkspaceRosAdapter()
        adapter._node = object()  # pylint: disable=protected-access
        pose = SimpleNamespace(
            position=SimpleNamespace(x=0.31, y=-0.02, z=0.42),
            orientation=SimpleNamespace(x=0.0, y=0.707, z=0.0, w=0.707),
        )
        adapter._get_pose_client = _FakePoseClient(  # pylint: disable=protected-access
            SimpleNamespace(success=True, current_pose=pose)
        )

        fake_service = type(
            'FakeGetCurrentPose',
            (),
            {'Request': type('Request', (), {'__init__': lambda self: setattr(self, 'reference_frame', '')})},
        )
        with patch('hmi.backend.ros.adapter.GetCurrentPose', fake_service):
            payload = adapter.get_current_pose(reference_frame='base_link')

        self.assertEqual(adapter._get_pose_client.requests[0].reference_frame, 'base_link')  # pylint: disable=protected-access
        self.assertEqual(
            payload,
            {
                'position': {'x': 0.31, 'y': -0.02, 'z': 0.42},
                'orientation': {'x': 0.0, 'y': 0.707, 'z': 0.0, 'w': 0.707},
            },
        )

    def test_get_current_pose_returns_none_when_service_is_unavailable(self) -> None:
        adapter = WorkspaceRosAdapter()
        adapter._node = object()  # pylint: disable=protected-access
        adapter._get_pose_client = _FakePoseClient(None, ready=False)  # pylint: disable=protected-access

        fake_service = type(
            'FakeGetCurrentPose',
            (),
            {'Request': type('Request', (), {'__init__': lambda self: setattr(self, 'reference_frame', '')})},
        )
        with patch('hmi.backend.ros.adapter.GetCurrentPose', fake_service):
            payload = adapter.get_current_pose(reference_frame='base_link')

        self.assertIsNone(payload)


if __name__ == '__main__':
    unittest.main()
