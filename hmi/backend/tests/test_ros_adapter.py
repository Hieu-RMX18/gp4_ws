from __future__ import annotations

from datetime import timedelta
import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import hmi.backend.ros.adapter as adapter_module
from hmi.backend.domain.models import (
    ConnectionHealth,
    SystemRuntimeState,
    TelemetryFreshnessState,
)
from hmi.backend.ros.adapter import (
    CONNECTION_FRESHNESS_SEC,
    KNOWN_WORKSPACE_ENDPOINTS,
    WorkspaceRosAdapter,
)


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
        self.wait_timeouts = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        self.wait_timeouts.append(timeout_sec)
        return self._ready

    def call_async(self, request):
        self.requests.append(request)
        return _CompletedFuture(self._response)


class _FakeReviewIntent:
    class Request:
        def __init__(self) -> None:
            self.raw_text = ""
            self.runtime_mode = ""
            self.session_id = ""
            self.operator_id = ""
            self.command_id = ""
            self.review_token = ""


class _FakeReviewClient:
    def __init__(self, response, *, ready: bool = True) -> None:
        self._response = response
        self._ready = ready
        self.requests = []
        self.wait_timeouts = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        self.wait_timeouts.append(timeout_sec)
        return self._ready

    def call_async(self, request):
        self.requests.append(request)
        return _CompletedFuture(self._response)


class _ReadyServiceClient:
    def __init__(self, ready: bool) -> None:
        self._ready = ready

    def service_is_ready(self) -> bool:
        return self._ready


class _ReadyActionClient:
    def __init__(self, ready: bool) -> None:
        self._ready = ready

    def server_is_ready(self) -> bool:
        return self._ready


class _FakePose:
    def __init__(self) -> None:
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class _FakeSequenceStep:
    def __init__(self) -> None:
        self.primitive_type = ""
        self.target_pose = _FakePose()
        self.blend_radius_m = 0.0
        self.planner_id = ""
        self.velocity_scale = 0.0
        self.acceleration_scale = 0.0


class _FakeExecuteMotion:
    class Goal:
        def __init__(self) -> None:
            self.primitive_type = ""
            self.velocity_scale = 0.0
            self.acceleration_scale = 0.0
            self.planner_id = ""
            self.reference_frame = ""
            self.delta_x = 0.0
            self.delta_y = 0.0
            self.delta_z = 0.0
            self.wait_duration_sec = 0.0
            self.joint_index = 0
            self.joint_angle = 0.0
            self.io_address = 0
            self.io_value = 0
            self.joint_target = []
            self.target_pose = _FakePose()
            self.waypoints = []
            self.sequence_steps = []


class WorkspaceRosAdapterTests(unittest.TestCase):
    def test_known_write_endpoints_exclude_deprecated_text_topic(self) -> None:
        write_endpoints = set(KNOWN_WORKSPACE_ENDPOINTS["write_capable_interfaces"])

        self.assertIn("/llm_gateway/review_intent", write_endpoints)
        self.assertNotIn("/llm_text_input", write_endpoints)

    def test_lost_conn_after_startup_grace_without_fresh_topics(self) -> None:
        adapter = WorkspaceRosAdapter()
        adapter._state.start_error = None
        adapter._state.ros_started_at = adapter._now() - timedelta(
            seconds=CONNECTION_FRESHNESS_SEC["ros"] + 0.5
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
        adapter._state.readiness.status_message = "ready"
        adapter._state.robot_status.received_at = now - timedelta(seconds=10.0)

        connections = adapter.read_connections()
        runtime = adapter.read_runtime_snapshot()
        source_statuses = {item.name: item for item in adapter.read_source_statuses()}

        self.assertEqual(connections[0].health, ConnectionHealth.HEALTHY)
        self.assertEqual(connections[3].health, ConnectionHealth.DEGRADED)
        self.assertEqual(runtime.system_state, SystemRuntimeState.NORMAL)
        self.assertEqual(source_statuses["readiness"].freshness_state.value, "fresh")
        self.assertEqual(source_statuses["robot_status"].freshness_state.value, "stale")

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
        adapter._state.readiness.status_message = "blocked"

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
        adapter._state.readiness.status_message = "blocked"

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

    def test_sim_mode_demotes_robot_status_and_prefers_joint_state_fallback(
        self,
    ) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.readiness.received_at = now
        adapter._state.readiness.ready = True
        adapter._state.readiness.status_message = (
            "simulation mode: robot status bypassed"
        )
        adapter._state.joint_received_at = now
        adapter._state.joint_source_topic = "/joint_states"
        adapter._state.joint_topic_received_at["/joint_states"] = now

        source_statuses = {item.name: item for item in adapter.read_source_statuses()}
        runtime = adapter.read_runtime_snapshot()

        self.assertEqual(runtime.mode.value, "sim")
        self.assertFalse(source_statuses["robot_status"].active)
        self.assertTrue(source_statuses["joint_states_fallback"].preferred)
        self.assertTrue(source_statuses["joint_states_fallback"].active)
        self.assertFalse(source_statuses["joint_states_primary"].preferred)
        self.assertFalse(source_statuses["joint_states_primary"].active)

    def test_review_intent_source_is_inactive_when_service_drops_after_ready(
        self,
    ) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.readiness.received_at = now
        adapter._state.readiness.ready = True
        adapter._state.readiness.status_message = "simulation mode: ready"
        adapter._state.review_intent_ready = False
        adapter._state.review_intent_ready_at = now

        source_statuses = {item.name: item for item in adapter.read_source_statuses()}
        review_source = source_statuses["review_intent_service"]

        self.assertFalse(review_source.active)
        self.assertEqual(
            review_source.freshness_state,
            TelemetryFreshnessState.FRESH,
        )
        self.assertIn("waiting for /llm_gateway/review_intent", review_source.detail)

    def test_llm_event_topics_are_not_freshness_critical_when_idle(self) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.llm.gateway_status_at = now

        source_statuses = {item.name: item for item in adapter.read_source_statuses()}

        self.assertTrue(source_statuses["gateway_status"].active)
        self.assertFalse(source_statuses["llm_debug"].active)
        self.assertFalse(source_statuses["llm_command"].active)

    def test_submit_text_for_review_calls_review_intent_service(self) -> None:
        response = SimpleNamespace(
            accepted=True,
            error="",
            semantic_ir_json='{"intent":"go_home"}',
        )
        client = _FakeReviewClient(response)
        adapter = WorkspaceRosAdapter()
        adapter._node = object()  # pylint: disable=protected-access
        adapter._review_intent_client = client  # pylint: disable=protected-access

        with (
            patch.object(
                adapter_module, "ReviewIntent", _FakeReviewIntent, create=True
            ),
            patch.dict(os.environ, {"GP4_REVIEW_INTENT_TOKEN": "review-token"}),
        ):
            result = adapter.submit_text_for_review(
                raw_text="go home",
                runtime_mode="sim",
                session_id="session-a",
                operator_id="operator-a",
                command_id="command-a",
            )

        self.assertTrue(result["accepted"])
        self.assertEqual(result["semanticIr"], {"intent": "go_home"})
        self.assertEqual(result["semanticIrJson"], '{"intent":"go_home"}')
        self.assertEqual(client.requests[0].raw_text, "go home")
        self.assertEqual(client.requests[0].runtime_mode, "sim")
        self.assertEqual(client.requests[0].session_id, "session-a")
        self.assertEqual(client.requests[0].operator_id, "operator-a")
        self.assertEqual(client.requests[0].command_id, "command-a")
        self.assertNotEqual(client.requests[0].review_token, "review-token")
        self.assertEqual(
            client.requests[0].review_token,
            adapter_module.build_review_intent_token(
                shared_secret="review-token",
                raw_text="go home",
                runtime_mode="sim",
                session_id="session-a",
                operator_id="operator-a",
                command_id="command-a",
            ),
        )

    def test_submit_text_for_review_uses_short_ready_timeout_and_review_sla_timeout(
        self,
    ) -> None:
        response = SimpleNamespace(
            accepted=True,
            error="",
            semantic_ir_json='{"intent":"go_home"}',
        )
        client = _FakeReviewClient(response)
        adapter = WorkspaceRosAdapter()
        adapter._node = object()  # pylint: disable=protected-access
        adapter._review_intent_client = client  # pylint: disable=protected-access
        future_timeouts: list[float] = []

        def wait_for_future(future, timeout_sec):
            future_timeouts.append(timeout_sec)
            return future.result()

        adapter._wait_for_future = wait_for_future  # type: ignore[method-assign]

        with patch.object(
            adapter_module, "ReviewIntent", _FakeReviewIntent, create=True
        ):
            adapter.submit_text_for_review(
                raw_text="go home",
                runtime_mode="sim",
                session_id="session-a",
                operator_id="operator-a",
                command_id="command-a",
            )

        self.assertLessEqual(client.wait_timeouts[0], 2.0)
        self.assertGreaterEqual(future_timeouts[0], 30.0)

    def test_submit_text_for_review_reports_unavailable_service(self) -> None:
        client = _FakeReviewClient(None, ready=False)
        adapter = WorkspaceRosAdapter()
        adapter._node = object()  # pylint: disable=protected-access
        adapter._review_intent_client = client  # pylint: disable=protected-access

        with patch.object(
            adapter_module, "ReviewIntent", _FakeReviewIntent, create=True
        ):
            result = adapter.submit_text_for_review(
                raw_text="go home",
                runtime_mode="sim",
                session_id="session-a",
                operator_id="operator-a",
                command_id="command-a",
            )

        self.assertFalse(result["accepted"])
        self.assertIn("review_intent service not ready", result["error"])
        self.assertEqual(client.requests, [])

    def test_command_interfaces_do_not_require_review_intent_readiness(self) -> None:
        adapter = WorkspaceRosAdapter()
        adapter._node = object()  # pylint: disable=protected-access
        adapter._validate_client = _ReadyServiceClient(True)  # pylint: disable=protected-access
        adapter._execute_client = _ReadyActionClient(True)  # pylint: disable=protected-access
        adapter._review_intent_client = _ReadyServiceClient(False)  # pylint: disable=protected-access
        adapter._state.readiness.received_at = adapter._now()  # pylint: disable=protected-access
        adapter._state.readiness.status_message = "sim ready"  # pylint: disable=protected-access

        adapter._refresh_command_interface_state()  # pylint: disable=protected-access

        self.assertTrue(adapter._command_interfaces_ready())  # pylint: disable=protected-access
        self.assertIsNone(
            adapter._command_interface_block_reason()  # pylint: disable=protected-access
        )
        source_statuses = {item.name: item for item in adapter.read_source_statuses()}
        self.assertIn("review_intent_service", source_statuses)
        self.assertFalse(source_statuses["review_intent_service"].active)
        self.assertIn(
            "waiting for /llm_gateway/review_intent",
            source_statuses["review_intent_service"].detail,
        )

    def test_build_command_payload_maps_cartesian_delta_to_move_rel_in_base_link(
        self,
    ) -> None:
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

    def test_build_command_payload_maps_joint_delta_to_absolute_move_joint(
        self,
    ) -> None:
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

    def test_build_command_payload_passthrough_preserves_wait_and_io_fields(
        self,
    ) -> None:
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

    def test_build_command_payload_passthrough_preserves_joint_and_pose_motion_fields(
        self,
    ) -> None:
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
                },
            }
        )
        self.assertEqual(move_joint_payload["primitive_type"], "MOVE_JOINT")
        self.assertEqual(move_joint_payload["joint_index"], 2)
        self.assertAlmostEqual(move_joint_payload["joint_angle"], 0.5)
        self.assertAlmostEqual(move_joint_payload["velocity_scale"], 0.06)
        self.assertAlmostEqual(move_joint_payload["acceleration_scale"], 0.06)
        self.assertEqual(move_joint_payload["planner_id"], "PILZ_PTP")

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
                },
            }
        )
        self.assertEqual(lin_payload["primitive_type"], "LIN")
        self.assertEqual(lin_payload["reference_frame"], "base_link")
        self.assertIn("target_pose", lin_payload)
        self.assertAlmostEqual(lin_payload["velocity_scale"], 0.05)
        self.assertAlmostEqual(lin_payload["acceleration_scale"], 0.04)
        self.assertEqual(lin_payload["planner_id"], "PILZ_LIN")

    def test_cartesian_path_uses_last_waypoint_as_target_pose_for_ros_dispatch(
        self,
    ) -> None:
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

    def test_blended_sequence_maps_steps_to_execute_motion_goal(self) -> None:
        adapter = WorkspaceRosAdapter()
        payload = {
            "primitive_type": "BLENDED_SEQUENCE",
            "velocity_scale": 0.06,
            "acceleration_scale": 0.06,
            "planner_id": "PILZ_LIN",
            "sequence_steps": [
                {
                    "primitive_type": "LIN",
                    "target_pose": {
                        "position": {"x": 0.30, "y": 0.00, "z": 0.31},
                        "orientation": {
                            "x": 0.0,
                            "y": 1.0,
                            "z": 0.0,
                            "w": 0.0,
                        },
                    },
                    "blend_radius_m": 0.0,
                    "planner_id": "PILZ_LIN",
                    "velocity_scale": 0.06,
                    "acceleration_scale": 0.06,
                },
                {
                    "primitive_type": "LIN",
                    "target_pose": {
                        "position": {"x": 0.32, "y": 0.01, "z": 0.33},
                        "orientation": {
                            "x": 0.1,
                            "y": 0.2,
                            "z": 0.3,
                            "w": 0.9,
                        },
                    },
                    "blend_radius_m": 0.005,
                    "planner_id": "PILZ_PTP",
                    "velocity_scale": 0.04,
                    "acceleration_scale": 0.03,
                },
            ],
        }

        with (
            patch("hmi.backend.ros.command_dispatch.ExecuteMotion", _FakeExecuteMotion),
            patch("hmi.backend.ros.command_dispatch.SequenceStep", _FakeSequenceStep),
            patch("hmi.backend.ros.command_dispatch.Pose", _FakePose),
        ):
            goal = adapter._build_execute_motion_goal(payload)  # pylint: disable=protected-access

        self.assertEqual(goal.primitive_type, "BLENDED_SEQUENCE")
        self.assertEqual(len(goal.sequence_steps), 2)
        for index, expected_step in enumerate(payload["sequence_steps"]):
            actual_step = goal.sequence_steps[index]
            self.assertEqual(
                actual_step.primitive_type, expected_step["primitive_type"]
            )
            self.assertAlmostEqual(
                actual_step.blend_radius_m, expected_step["blend_radius_m"]
            )
            self.assertEqual(actual_step.planner_id, expected_step["planner_id"])
            self.assertAlmostEqual(
                actual_step.velocity_scale, expected_step["velocity_scale"]
            )
            self.assertAlmostEqual(
                actual_step.acceleration_scale, expected_step["acceleration_scale"]
            )
            expected_position = expected_step["target_pose"]["position"]
            self.assertAlmostEqual(
                actual_step.target_pose.position.x, expected_position["x"]
            )
            self.assertAlmostEqual(
                actual_step.target_pose.position.y, expected_position["y"]
            )
            self.assertAlmostEqual(
                actual_step.target_pose.position.z, expected_position["z"]
            )

    def test_hardware_preflight_detects_missing_primary_joint_source(self) -> None:
        adapter = WorkspaceRosAdapter()
        now = adapter._now()
        adapter._state.start_error = None
        adapter._state.ros_started_at = now
        adapter._state.readiness.received_at = now
        adapter._state.readiness.ready = True
        adapter._state.readiness.status_message = "hardware ready"
        adapter._state.robot_status.received_at = now
        adapter._state.robot_status.e_stopped = False
        adapter._state.robot_status.in_error = False
        adapter._state.joint_received_at = now
        adapter._state.joint_source_topic = "/joint_states"
        adapter._state.joint_topic_received_at["/joint_states"] = now
        adapter._state.validate_command_ready = True
        adapter._state.execute_motion_ready = True
        adapter._state.validate_command_ready_at = now
        adapter._state.execute_motion_ready_at = now
        preflight = adapter.evaluate_execution_preflight(target_mode="hardware")
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
            "FakeGetCurrentPose",
            (),
            {
                "Request": type(
                    "Request",
                    (),
                    {"__init__": lambda self: setattr(self, "reference_frame", "")},
                )
            },
        )
        with patch("hmi.backend.ros.adapter.GetCurrentPose", fake_service):
            payload = adapter.get_current_pose(reference_frame="base_link")

        self.assertEqual(
            adapter._get_pose_client.requests[0].reference_frame, "base_link"
        )  # pylint: disable=protected-access
        self.assertEqual(
            payload,
            {
                "position": {"x": 0.31, "y": -0.02, "z": 0.42},
                "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
            },
        )

    def test_get_current_pose_returns_none_when_service_is_unavailable(self) -> None:
        adapter = WorkspaceRosAdapter()
        adapter._node = object()  # pylint: disable=protected-access
        adapter._get_pose_client = _FakePoseClient(None, ready=False)  # pylint: disable=protected-access

        fake_service = type(
            "FakeGetCurrentPose",
            (),
            {
                "Request": type(
                    "Request",
                    (),
                    {"__init__": lambda self: setattr(self, "reference_frame", "")},
                )
            },
        )
        with patch("hmi.backend.ros.adapter.GetCurrentPose", fake_service):
            payload = adapter.get_current_pose(reference_frame="base_link")

        self.assertIsNone(payload)


if __name__ == "__main__":
    unittest.main()
