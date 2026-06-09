from __future__ import annotations

import unittest
import os
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any
from unittest.mock import patch

from hmi.backend.api.contracts import (
    CommandExecutionResultModel,
    CommandMutationResponseModel,
)
from hmi.backend.domain.models import (
    BridgeConnection,
    ConnectionHealth,
    JointPosition,
    RobotStatusSnapshot,
    RuntimeMode,
    RuntimeSnapshot,
    SystemRuntimeState,
    TelemetryFreshnessState,
    TelemetrySourceSnapshot,
)
from hmi.backend.services.audit_service import AuditService
from hmi.backend.services.session_lock_service import SessionLockService
from hmi.backend.services.supervisor_service import (
    ConflictError,
    ForbiddenActionError,
    SupervisorService,
)
from hmi.backend.services.telemetry_bridge_service import TelemetryBridgeService


class FakeSupervisorAdapter:
    def __init__(self) -> None:
        self.started = False
        self.stopped = False
        self.submit_calls: list[dict] = []
        self.confirm_calls: list[dict] = []
        self.abort_calls: list[dict] = []
        self.get_pose_calls: list[dict] = []
        self.start_traj_mode_calls = 0
        self.stop_motion_calls = 0
        self._runtime = RuntimeSnapshot(
            system_state=SystemRuntimeState.NORMAL,
            blocking=False,
            status_text="Sim telemetry fresh and runtime clear.",
            mode=RuntimeMode.SIM,
            robot_status=RobotStatusSnapshot(readiness_message="Sim ready"),
        )
        self._connections = [
            BridgeConnection(
                name="ros2", label="ROS 2", health=ConnectionHealth.HEALTHY
            ),
            BridgeConnection(
                name="moveit2", label="MoveIt 2", health=ConnectionHealth.HEALTHY
            ),
            BridgeConnection(name="llm", label="LLM", health=ConnectionHealth.HEALTHY),
            BridgeConnection(
                name="motoros2", label="MotoROS2", health=ConnectionHealth.DOWN
            ),
        ]
        self._joints = [
            JointPosition(name="joint_1_s", position_deg=0.0),
            JointPosition(name="joint_2_l", position_deg=5.0),
            JointPosition(name="joint_3_u", position_deg=10.0),
            JointPosition(name="joint_4_r", position_deg=15.0),
            JointPosition(name="joint_5_b", position_deg=20.0),
            JointPosition(name="joint_6_t", position_deg=25.0),
        ]
        self._source_statuses = [
            TelemetrySourceSnapshot(
                name="gateway_status",
                label="Gateway status",
                topic="/gateway_status",
                last_seen_at=None,
                freshness_threshold_sec=30.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                active=True,
            ),
            TelemetrySourceSnapshot(
                name="readiness",
                label="HW readiness",
                topic="/hw_adapter/ready",
                last_seen_at=None,
                freshness_threshold_sec=3.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                active=True,
            ),
            TelemetrySourceSnapshot(
                name="supervisor_alerts",
                label="Supervisor alerts",
                topic="/supervisor/alerts",
                last_seen_at=None,
                freshness_threshold_sec=5.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                active=True,
            ),
            TelemetrySourceSnapshot(
                name="joint_states_fallback",
                label="Joint states fallback",
                topic="/joint_states",
                last_seen_at=None,
                freshness_threshold_sec=3.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                preferred=True,
                active=True,
            ),
            TelemetrySourceSnapshot(
                name="robot_status",
                label="Robot status",
                topic="/yaskawa/robot_status",
                last_seen_at=None,
                freshness_threshold_sec=3.0,
                freshness_state=TelemetryFreshnessState.STALE,
                active=False,
                detail="SIM mode uses /hw_adapter/ready instead.",
            ),
            TelemetrySourceSnapshot(
                name="llm_debug",
                label="LLM debug",
                topic="/llm_debug",
                last_seen_at=None,
                freshness_threshold_sec=30.0,
                freshness_state=TelemetryFreshnessState.STALE,
                active=False,
            ),
            TelemetrySourceSnapshot(
                name="llm_command",
                label="LLM command echo",
                topic="/llm_command",
                last_seen_at=None,
                freshness_threshold_sec=30.0,
                freshness_state=TelemetryFreshnessState.STALE,
                active=False,
            ),
            TelemetrySourceSnapshot(
                name="review_intent_service",
                label="ReviewIntent service",
                topic="/llm_gateway/review_intent",
                last_seen_at=None,
                freshness_threshold_sec=3.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                active=True,
                detail="ready at /llm_gateway/review_intent",
            ),
        ]
        self._confirm_result = {
            "accepted": True,
            "adapter": "workspace_ros_adapter",
            "status": "succeeded",
            "summary": "Sim execution completed successfully.",
            "dispatchedToRos": True,
        }
        self._review_result: dict[str, Any] | None = None
        self._preflight_result = {
            "accepted": True,
            "mode": "sim",
            "reasons": [],
            "requiredSources": [],
            "sourceStatuses": [],
            "runtimeState": "NORMAL",
        }
        self._current_pose: dict[str, Any] | None = {
            "position": {"x": 0.30, "y": 0.00, "z": 0.30},
            "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
        }

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

    def submit_text_for_review(self, **kwargs):
        self.submit_calls.append(kwargs)
        if self._review_result is not None:
            return deepcopy(self._review_result)
        semantic_ir = self._default_review_semantic_ir(kwargs.get("raw_text", ""))
        result = {"accepted": True, "adapter": "fake-review"}
        if semantic_ir is not None:
            result["semanticIr"] = semantic_ir
        return result

    def _default_review_semantic_ir(self, raw_text: str) -> dict[str, Any] | None:
        normalized = " ".join(str(raw_text or "").strip().lower().split())
        if normalized in {"home", "go home", "move home", "return home"}:
            return {"intent": "go_home"}
        if normalized in {"stop", "stop motion", "cancel motion", "halt"}:
            return {"intent": "stop"}
        if normalized in {"get pose", "current pose", "where is robot", "where is tcp"}:
            return {"intent": "get_pose"}
        if normalized in {"move up 10 cm", "move up 0.1 m", "move up 1 cm"}:
            value = (
                0.1
                if normalized.endswith("0.1 m")
                else (10.0 if normalized.endswith("10 cm") else 1.0)
            )
            unit = "m" if normalized.endswith("0.1 m") else "cm"
            return {
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": 0.0, "z": value},
                "linear_unit": unit,
                "reference_frame": "base_link",
            }
        if normalized in {"move down 1 cm", "move down 5 cm"}:
            value = -5.0 if normalized.endswith("5 cm") else -1.0
            return {
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": 0.0, "z": value},
                "linear_unit": "cm",
                "reference_frame": "base_link",
            }
        if normalized == "move joint 2 5 deg":
            return {
                "intent": "move_joint",
                "joint_index": 1,
                "joint_angle": 5.0,
                "angular_unit": "deg",
            }
        if normalized == "move joint 2 +5 deg":
            return {
                "intent": "move_joint_delta",
                "joint_index": 1,
                "delta_angle": 5.0,
                "angular_unit": "deg",
            }
        if normalized in {
            "home, wait 1 s, then move up 1 cm",
            "home, wait 1 s, then move down 1 cm",
        }:
            z_delta = -1.0 if "move down" in normalized else 1.0
            return {
                "intent": "sequence",
                "steps": [
                    {"intent": "go_home"},
                    {"intent": "wait", "wait_duration_sec": 1.0},
                    {
                        "intent": "move_relative",
                        "delta": {"x": 0.0, "y": 0.0, "z": z_delta},
                        "linear_unit": "cm",
                        "reference_frame": "base_link",
                    },
                ],
            }
        if normalized in {"write gp4", "vẽ chữ gp4"}:
            return {
                "intent": "draw_text",
                "text": "GP4",
                "units": "mm",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "font": {"type": "single_stroke_builtin", "height": 20},
            }
        return None

    def confirm_command(self, **kwargs):
        self.confirm_calls.append(kwargs)
        response = dict(self._confirm_result)
        response.update(
            {
                "commandId": kwargs["command_id"],
                "planFingerprint": kwargs["plan_fingerprint"],
                "correlationId": kwargs["correlation_id"],
            }
        )
        return response

    def evaluate_execution_preflight(self, *, target_mode: str | None = None):
        result = dict(self._preflight_result)
        if target_mode is not None:
            result["mode"] = target_mode
        return result

    def abort_command(self, **kwargs):
        self.abort_calls.append(kwargs)
        return True, "cancelled before ROS dispatch"

    def get_current_pose(
        self, *, reference_frame: str = "base_link"
    ) -> dict[str, Any] | None:
        self.get_pose_calls.append({"reference_frame": reference_frame})
        if self._current_pose is None:
            return None
        return deepcopy(self._current_pose)

    def start_traj_mode(self) -> dict[str, Any]:
        self.start_traj_mode_calls += 1
        return {"accepted": True, "message": "start_traj_mode called"}

    def stop_motion(self) -> dict[str, Any]:
        self.stop_motion_calls += 1
        return {"accepted": True, "message": "stop_motion called"}

    def set_runtime(
        self, system_state: SystemRuntimeState, *, mode: RuntimeMode = RuntimeMode.SIM
    ) -> None:
        robot_status = RobotStatusSnapshot(
            servo_state="OFF"
            if mode == RuntimeMode.HARDWARE and system_state == SystemRuntimeState.SAFETY_BLOCKED
            else "ON",
            e_stop="CLEAR",
            alarm_state="NONE",
            motion_mode="AUTO" if mode == RuntimeMode.HARDWARE else None,
            readiness_message="fixture",
        )
        self._runtime = RuntimeSnapshot(
            system_state=system_state,
            blocking=system_state
            in {
                SystemRuntimeState.FAULT,
                SystemRuntimeState.ESTOP,
                SystemRuntimeState.LOST_CONN,
                SystemRuntimeState.SAFETY_BLOCKED,
            },
            status_text=f"Runtime set to {system_state.value}.",
            mode=mode,
            robot_status=robot_status,
        )

    def set_runtime_snapshot(self, runtime: RuntimeSnapshot) -> None:
        self._runtime = deepcopy(runtime)

    def set_source_freshness(self, *, stale_names: set[str]) -> None:
        next_statuses: list[TelemetrySourceSnapshot] = []
        for source in self._source_statuses:
            next_statuses.append(
                TelemetrySourceSnapshot(
                    name=source.name,
                    label=source.label,
                    topic=source.topic,
                    last_seen_at=source.last_seen_at,
                    freshness_threshold_sec=source.freshness_threshold_sec,
                    freshness_state=(
                        TelemetryFreshnessState.STALE
                        if source.name in stale_names
                        else source.freshness_state
                    ),
                    preferred=source.preferred,
                    active=source.active,
                    detail=source.detail,
                )
            )
        self._source_statuses = next_statuses

    def set_confirm_result(self, **kwargs) -> None:
        self._confirm_result.update(kwargs)

    def set_review_result(self, **kwargs) -> None:
        self._review_result = dict(kwargs)

    def set_review_intent_ready(self, ready: bool) -> None:
        next_statuses: list[TelemetrySourceSnapshot] = []
        for source in self._source_statuses:
            if source.name != "review_intent_service":
                next_statuses.append(source)
                continue
            next_statuses.append(
                TelemetrySourceSnapshot(
                    name=source.name,
                    label=source.label,
                    topic=source.topic,
                    last_seen_at=source.last_seen_at,
                    freshness_threshold_sec=source.freshness_threshold_sec,
                    freshness_state=(
                        TelemetryFreshnessState.FRESH
                        if ready
                        else TelemetryFreshnessState.STALE
                    ),
                    preferred=source.preferred,
                    active=ready,
                    detail=(
                        "ready at /llm_gateway/review_intent"
                        if ready
                        else "waiting for /llm_gateway/review_intent"
                    ),
                )
            )
        self._source_statuses = next_statuses

    def set_preflight(self, *, accepted: bool, reasons: list[str]) -> None:
        self._preflight_result = {
            "accepted": accepted,
            "mode": self._runtime.mode.value,
            "reasons": list(reasons),
            "requiredSources": [],
            "sourceStatuses": [],
            "runtimeState": self._runtime.system_state.value,
        }

    def set_current_pose(self, pose: dict[str, Any] | None) -> None:
        self._current_pose = deepcopy(pose)


class SupervisorServiceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.audit = AuditService(Path(self.temp_dir.name) / "audit.sqlite3")
        self.session_lock = SessionLockService()
        self.adapter = FakeSupervisorAdapter()
        self.telemetry = TelemetryBridgeService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            poll_interval_sec=3600.0,
        )
        self.supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        self.supervisor.bind_telemetry_service(self.telemetry)
        self.session_id = "session-a"
        self.operator_id = "operator-a"

    def _acquire_lease(self) -> str:
        lease = self.session_lock.acquire_controller(self.session_id, self.operator_id)
        return lease.lease_token

    def test_ros_adapter_property_exposes_constructor_instance(self) -> None:
        self.assertIs(self.supervisor.ros_adapter, self.adapter)

    def test_capabilities_allow_deterministic_commands_when_review_intent_is_not_ready(
        self,
    ) -> None:
        self.adapter.set_review_intent_ready(False)

        overlay = self.supervisor.snapshot_overlay(self.session_id, self.operator_id)

        self.assertTrue(overlay["capabilities"]["canSubmitCommands"])
        self.assertTrue(overlay["capabilities"]["commandIngressAvailable"])

    def test_review_intent_outage_preserves_existing_controller_lease(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_review_intent_ready(False)

        overlay = self.supervisor.snapshot_overlay(self.session_id, self.operator_id)

        self.assertTrue(overlay["capabilities"]["canSubmitCommands"])
        self.assertTrue(overlay["capabilities"]["canConfirmCommands"])
        self.assertTrue(overlay["lease"]["ownsControl"])
        self.assertEqual(overlay["lease"]["leaseToken"], lease_token)

    def test_hardware_capabilities_allow_deterministic_commands_without_review_token(
        self,
    ) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)

        with patch.dict(os.environ, {}, clear=True):
            overlay = supervisor.snapshot_overlay(self.session_id, self.operator_id)

        self.assertTrue(overlay["capabilities"]["canSubmitCommands"])
        self.assertTrue(overlay["capabilities"]["commandIngressAvailable"])

    def test_sim_capabilities_allow_deterministic_commands_without_review_token(
        self,
    ) -> None:
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.SIM)

        with patch.dict(os.environ, {}, clear=True):
            overlay = self.supervisor.snapshot_overlay(
                self.session_id,
                self.operator_id,
            )

        self.assertTrue(overlay["capabilities"]["canSubmitCommands"])
        self.assertTrue(overlay["capabilities"]["commandIngressAvailable"])

    def test_hardware_capabilities_allow_review_without_review_token(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)

        with patch.dict(os.environ, {}, clear=True):
            overlay = supervisor.snapshot_overlay(self.session_id, self.operator_id)

        self.assertTrue(overlay["capabilities"]["canSubmitCommands"])
        self.assertTrue(overlay["capabilities"]["commandIngressAvailable"])
        self.assertIn("request", overlay["lease"]["statusText"])
        self.assertNotIn("locked", overlay["lease"]["statusText"])

    def test_hardware_capabilities_allow_review_with_empty_environment(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)

        with patch.dict(
            os.environ,
            {},
            clear=True,
        ):
            overlay = supervisor.snapshot_overlay(
                self.session_id,
                self.operator_id,
            )

        self.assertTrue(overlay["capabilities"]["canSubmitCommands"])
        self.assertTrue(overlay["capabilities"]["commandIngressAvailable"])

    def test_raw_text_rejects_when_gateway_review_returns_no_semantic_ir(self) -> None:
        self.adapter.set_review_result(
            accepted=False,
            adapter="fake-gateway-review",
            error="review_intent service not ready",
        )
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )

        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertIn("review_intent", response["reason"])
        self.assertEqual(self.adapter.confirm_calls, [])

    def test_external_structured_semantic_intent_is_rejected(self) -> None:
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="",
            structured_intent={"intent": "go_home"},
            mode="sim",
        )

        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertIn("structuredIntent", response["reason"])
        self.assertEqual(self.adapter.submit_calls, [])

    def test_command_rejected_without_valid_control_lease(self) -> None:
        with self.assertRaises(ForbiddenActionError):
            self.supervisor.submit_intent(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=None,
                raw_text="home",
                mode="sim",
            )

    def test_command_rejected_for_blocking_runtime_states(self) -> None:
        lease_token = self._acquire_lease()
        for runtime_state in (
            SystemRuntimeState.ESTOP,
            SystemRuntimeState.FAULT,
            SystemRuntimeState.LOST_CONN,
            SystemRuntimeState.SAFETY_BLOCKED,
        ):
            with self.subTest(runtime_state=runtime_state.value):
                self.adapter.set_runtime(runtime_state)
                response = self.supervisor.submit_intent(
                    session_id=self.session_id,
                    operator_id=self.operator_id,
                    lease_token=lease_token,
                    raw_text="home",
                    mode="sim",
                )
                self.assertFalse(response["accepted"])
                self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
                self.assertIn(runtime_state.value, response["reason"])
                self.assertEqual(self.adapter.confirm_calls, [])
                self.adapter.set_runtime(SystemRuntimeState.NORMAL)

    def test_ambiguous_command_does_not_execute(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="draw a dragon on the table",
            mode="sim",
        )
        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertEqual(self.adapter.confirm_calls, [])

    def test_review_failure_does_not_fall_back_to_local_motion_parser(self) -> None:
        self.adapter.set_review_result(
            accepted=False,
            error="review service unavailable",
        )
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move up 10 cm",
            mode="sim",
        )

        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertIn("review service unavailable", response["reason"])
        self.assertEqual(self.adapter.confirm_calls, [])

    def test_cartesian_text_intent_uses_base_link_for_sim_move_rel(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move up 10 cm",
            mode="sim",
        )
        self.assertTrue(response["accepted"])
        self.assertEqual(response["command"]["parsedIntent"]["action"], "MOVE_REL")
        self.assertEqual(
            response["command"]["parsedIntent"]["normalizedCommand"]["reference_frame"],
            "base_link",
        )

    def test_cartesian_text_intent_accepts_meters_but_summarizes_mm(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move up 0.1 m",
            mode="sim",
        )

        self.assertTrue(response["accepted"])
        normalized_command = response["command"]["parsedIntent"]["normalizedCommand"]
        self.assertEqual(normalized_command["primitive_type"], "MOVE_REL")
        self.assertAlmostEqual(normalized_command["delta_z"], 0.1)
        self.assertIn(
            "dz=100.0 mm", response["command"]["parsedIntent"]["targetSummary"]
        )

    def test_joint_text_intent_maps_to_absolute_joint_target(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move joint 2 5 deg",
            mode="sim",
        )
        self.assertTrue(response["accepted"])
        normalized_command = response["command"]["parsedIntent"]["normalizedCommand"]
        self.assertEqual(normalized_command["primitive_type"], "MOVE_JOINT")
        self.assertEqual(normalized_command["joint_index"], 1)
        self.assertAlmostEqual(
            normalized_command["joint_angle"], 5.0 * 3.141592653589793 / 180.0
        )

    def test_signed_joint_text_intent_maps_to_relative_joint_target(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move joint 2 +5 deg",
            mode="sim",
        )
        self.assertTrue(response["accepted"])
        normalized_command = response["command"]["parsedIntent"]["normalizedCommand"]
        self.assertEqual(normalized_command["primitive_type"], "MOVE_JOINT")
        self.assertEqual(normalized_command["joint_index"], 1)
        self.assertAlmostEqual(
            normalized_command["joint_angle"], 10.0 * 3.141592653589793 / 180.0
        )

    def test_confirmation_required_command_stops_before_execution_boundary(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        self.assertTrue(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "NEEDS_CONFIRMATION")
        self.assertEqual(self.adapter.confirm_calls, [])

    def test_gateway_review_semantic_ir_drives_hmi_parse_without_execution(
        self,
    ) -> None:
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={"intent": "move_named_pose", "pose_name": "ready"},
        )
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move to ready pose",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(self.adapter.confirm_calls, [])
        self.assertEqual(len(self.adapter.submit_calls), 1)
        parsed = response["command"]["parsedIntent"]
        self.assertEqual(parsed["source"], "structured")
        self.assertEqual(response["command"]["intentSource"], "text")
        self.assertEqual(
            response["command"]["structuredIntent"],
            {"intent": "move_named_pose", "pose_name": "ready"},
        )
        self.assertEqual(parsed["action"], "PTP")
        normalized = parsed["normalizedCommand"]
        self.assertEqual(normalized["primitive_type"], "PTP")
        self.assertEqual(normalized["planner_id"], "PILZ_PTP")
        self.assertAlmostEqual(normalized["joint_target"][0], 1.938101818035138)
        self.assertEqual(response["command"]["lifecycleState"], "NEEDS_CONFIRMATION")

    def test_gateway_review_canonical_pose_alias_reaches_confirmation(
        self,
    ) -> None:
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={"intent": "move_named_pose", "pose_name": "poseA"},
        )
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move to pose A",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(self.adapter.confirm_calls, [])
        parsed = response["command"]["parsedIntent"]
        self.assertEqual(parsed["action"], "PTP")
        normalized = parsed["normalizedCommand"]
        self.assertEqual(normalized["primitive_type"], "PTP")
        self.assertEqual(normalized["planner_id"], "PILZ_PTP")
        self.assertEqual(len(normalized["joint_target"]), 6)
        self.assertEqual(response["command"]["lifecycleState"], "NEEDS_CONFIRMATION")

    def test_gateway_review_sequence_semantic_ir_enters_sequence_path(
        self,
    ) -> None:
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={
                "intent": "sequence",
                "steps": [
                    {"intent": "go_home"},
                    {"intent": "wait", "wait_duration_sec": 1.0},
                ],
            },
        )
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="perform the reviewed plan",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(len(self.adapter.submit_calls), 1)
        self.assertEqual(self.adapter.confirm_calls, [])
        self.assertEqual(
            [step["parsedIntent"]["action"] for step in response["sequence"]["steps"]],
            ["HOME", "WAIT"],
        )
        self.assertEqual(
            response["sequence"]["structuredIntent"]["intent"],
            "sequence",
        )
        self.assertEqual(response["sequence"]["intentSource"], "text")
        self.assertTrue(
            all(
                step["intentSource"] == "text" for step in response["sequence"]["steps"]
            )
        )

    def test_gateway_review_factory_task_metadata_is_visible_in_sequence_summary(
        self,
    ) -> None:
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={
                "intent": "sequence",
                "steps": [
                    {"intent": "go_home"},
                    {"intent": "wait", "wait_duration_sec": 1.0},
                ],
                "metadata": {
                    "factory_task": {"task_id": "home-wait", "mode": "supervised_hardware"},
                    "runtime_plan": {"type": "sequence", "children": []},
                    "policy_decisions": [
                        {
                            "node_path": "root",
                            "decision": "allow",
                            "reason": "runtime control remains behind supervisor validation",
                            "risk_level": "medium",
                        }
                    ],
                },
            },
        )
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="perform the reviewed factory task",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        summary = response["sequence"]["planSummary"]
        self.assertEqual(summary["factoryTask"]["task_id"], "home-wait")
        self.assertEqual(summary["factoryTaskRuntimePlan"]["type"], "sequence")
        self.assertEqual(summary["factoryTaskPolicyDecisions"][0]["decision"], "allow")

    def test_gateway_review_sequence_return_to_start_uses_start_joints(
        self,
    ) -> None:
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={
                "intent": "sequence",
                "steps": [
                    {"intent": "move_named_pose", "pose_name": "poseA"},
                    {"intent": "return_to_start"},
                ],
            },
        )
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move to pose A then return to start",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(
            [step["parsedIntent"]["action"] for step in response["sequence"]["steps"]],
            ["PTP", "MOVE_JOINTS"],
        )
        normalized = response["sequence"]["steps"][1]["parsedIntent"][
            "normalizedCommand"
        ]
        self.assertEqual(normalized["primitive_type"], "MOVE_JOINTS")
        self.assertEqual(len(normalized["joint_target"]), 6)
        self.assertAlmostEqual(normalized["joint_target"][0], 0.0)
        self.assertAlmostEqual(normalized["joint_target"][1], 0.08726646259971647)
        self.assertEqual(
            response["sequence"]["structuredIntent"]["steps"][1]["intent"],
            "return_to_start",
        )
        self.assertIn(
            "joint_target",
            response["sequence"]["structuredIntent"]["steps"][1],
        )

    def test_gateway_review_named_pose_sequence_confirms_in_order(
        self,
    ) -> None:
        self.adapter.set_review_result(
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
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move to poseA then poseB then home",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(self.adapter.confirm_calls, [])
        self.assertEqual(
            [step["parsedIntent"]["action"] for step in response["sequence"]["steps"]],
            ["PTP", "PTP", "HOME"],
        )

        confirm_response = self.supervisor.confirm_sequence(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            sequence_id=response["sequenceId"],
            plan_fingerprint=response["sequence"]["planFingerprint"],
        )

        self.assertTrue(confirm_response["accepted"])
        self.assertEqual(confirm_response["sequence"]["finalState"], "SUCCEEDED")
        self.assertEqual(len(self.adapter.confirm_calls), 3)
        self.assertEqual(
            [call["parsed_intent"]["action"] for call in self.adapter.confirm_calls],
            ["PTP", "PTP", "HOME"],
        )

    def test_gateway_review_named_home_sequence_runs_step_by_step(
        self,
    ) -> None:
        """Named/home sequences stay step-by-step so each step passes safety gates."""
        self.adapter.set_review_result(
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
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move to poseA then poseB then home",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(response["sequence"]["stepCount"], 3)

        actions = [step["parsedIntent"]["action"] for step in response["sequence"]["steps"]]
        self.assertEqual(actions, ["PTP", "PTP", "HOME"])

        confirm_response = self.supervisor.confirm_sequence(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            sequence_id=response["sequenceId"],
            plan_fingerprint=response["sequence"]["planFingerprint"],
        )
        self.assertTrue(confirm_response["accepted"])
        self.assertEqual(confirm_response["sequence"]["finalState"], "SUCCEEDED")
        self.assertEqual(len(self.adapter.confirm_calls), 3)
        self.assertEqual(
            [call["parsed_intent"]["action"] for call in self.adapter.confirm_calls],
            ["PTP", "PTP", "HOME"],
        )

    def test_sim_auto_confirm_executes_immediately_when_enabled(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
            sim_auto_confirm=True,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        lease_token = self._acquire_lease()
        response = supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        self.assertTrue(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "SUCCEEDED")
        self.assertEqual(response["command"]["finalState"], "SUCCEEDED")
        self.assertEqual(len(self.adapter.confirm_calls), 1)

    def test_stale_critical_telemetry_rejects_execution_path(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_source_freshness(stale_names={"joint_states_fallback"})
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertIn("joint_states_fallback", response["reason"])

    def test_hardware_mode_allows_command_when_evidence_gate_is_locked(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        self.adapter.set_preflight(accepted=True, reasons=[])
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="hardware",
        )
        self.assertTrue(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "NEEDS_CONFIRMATION")
        self.assertEqual(response["command"]["mode"], "hardware")

    def test_hardware_mode_allows_command_when_gate_and_preflight_pass(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        self.adapter.set_preflight(accepted=True, reasons=[])
        response = supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="hardware",
        )
        self.assertTrue(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "NEEDS_CONFIRMATION")
        self.assertEqual(response["command"]["mode"], "hardware")

    def test_hardware_submit_ignores_gateway_and_execution_boundary_staleness(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        self.adapter.set_source_freshness(
            stale_names={"gateway_status", "supervisor_alerts"}
        )
        self.adapter.set_preflight(
            accepted=False,
            reasons=[
                "required telemetry source gateway_status is stale.",
                "required telemetry source supervisor_alerts is stale.",
                "required telemetry source validate_command_service is inactive.",
                "required telemetry source validate_command_service is stale.",
            ],
        )

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="hardware",
        )

        self.assertTrue(response["accepted"], response.get("reason"))
        self.assertEqual(response["command"]["lifecycleState"], "NEEDS_CONFIRMATION")
        self.assertIsNotNone(response["command"]["planFingerprint"])

    def test_hardware_confirmation_still_blocks_when_validate_service_is_unavailable(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        self.adapter.set_source_freshness(
            stale_names={"gateway_status", "supervisor_alerts"}
        )
        self.adapter.set_preflight(
            accepted=False,
            reasons=[
                "required telemetry source gateway_status is stale.",
                "required telemetry source supervisor_alerts is stale.",
                "required telemetry source validate_command_service is inactive.",
                "required telemetry source validate_command_service is stale.",
            ],
        )

        submit_response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="hardware",
        )

        self.assertTrue(submit_response["accepted"], submit_response.get("reason"))

        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=submit_response["commandId"],
            plan_fingerprint=submit_response["command"]["planFingerprint"],
        )

        self.assertFalse(confirm_response["accepted"])
        self.assertIn("validate_command_service", confirm_response["reason"])
        self.assertEqual(self.adapter.confirm_calls, [])

    def test_servo_start_requires_hardware_mode_before_adapter_call(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        lease_token = self._acquire_lease()

        response = supervisor.start_servo(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
        )

        self.assertFalse(response["accepted"])
        self.assertIn("hardware mode", response["message"])
        self.assertEqual(self.adapter.start_traj_mode_calls, 0)

    def test_servo_start_requires_controller_lease_before_adapter_call(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)

        with self.assertRaises(ForbiddenActionError):
            supervisor.start_servo(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=None,
            )
        self.assertEqual(self.adapter.start_traj_mode_calls, 0)

    def test_servo_start_requires_controller_lease_when_runtime_unknown(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.UNKNOWN)
        self.adapter.set_preflight(accepted=True, reasons=[])

        with self.assertRaises(ForbiddenActionError):
            supervisor.start_servo(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=None,
            )
        self.assertEqual(self.adapter.start_traj_mode_calls, 0)

    def test_servo_start_can_power_drives_when_runtime_blocked_by_servo_off(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime_snapshot(
            RuntimeSnapshot(
                system_state=SystemRuntimeState.SAFETY_BLOCKED,
                blocking=True,
                status_text="not ready. drives_powered=FALSE",
                mode=RuntimeMode.HARDWARE,
                robot_status=RobotStatusSnapshot(
                    servo_state="OFF",
                    e_stop="CLEAR",
                    alarm_state="NONE",
                    motion_mode="AUTO",
                    readiness_message="Servo is off.",
                ),
            )
        )
        self.adapter.set_preflight(
            accepted=False,
            reasons=["required telemetry source validate_command_service is stale."],
        )

        response = self.supervisor.start_servo(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
        )

        self.assertTrue(response["accepted"], response["message"])
        self.assertEqual(self.adapter.start_traj_mode_calls, 1)

    def test_servo_start_rejects_active_estop_before_adapter_call(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime_snapshot(
            RuntimeSnapshot(
                system_state=SystemRuntimeState.ESTOP,
                blocking=True,
                status_text="Emergency stop active.",
                mode=RuntimeMode.HARDWARE,
                robot_status=RobotStatusSnapshot(
                    servo_state="OFF",
                    e_stop="ACTIVE",
                    alarm_state="NONE",
                    motion_mode="AUTO",
                    readiness_message="E-stop active.",
                ),
            )
        )

        response = self.supervisor.start_servo(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
        )

        self.assertFalse(response["accepted"])
        self.assertIn("e-stop", response["message"].lower())
        self.assertEqual(self.adapter.start_traj_mode_calls, 0)

    def test_servo_start_requires_auto_mode_before_adapter_call(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime_snapshot(
            RuntimeSnapshot(
                system_state=SystemRuntimeState.NORMAL,
                blocking=False,
                status_text="Manual mode.",
                mode=RuntimeMode.HARDWARE,
                robot_status=RobotStatusSnapshot(
                    servo_state="OFF",
                    e_stop="CLEAR",
                    alarm_state="NONE",
                    motion_mode="MANUAL",
                    readiness_message="Manual mode.",
                ),
            )
        )

        response = self.supervisor.start_servo(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
        )

        self.assertFalse(response["accepted"])
        self.assertIn("auto", response["message"].lower())
        self.assertEqual(self.adapter.start_traj_mode_calls, 0)

    def test_servo_hold_calls_adapter_even_when_hardware_gate_is_locked(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)

        response = self.supervisor.hold_servo(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
        )

        self.assertTrue(response["accepted"])
        self.assertEqual(self.adapter.stop_motion_calls, 1)

    def test_servo_hold_ignores_command_execution_preflight_when_stopping(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        self.adapter.set_preflight(
            accepted=False,
            reasons=["required telemetry source validate_command_service is stale."],
        )

        response = self.supervisor.hold_servo(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
        )

        self.assertTrue(response["accepted"], response["message"])
        self.assertEqual(self.adapter.stop_motion_calls, 1)

    
    def test_reviewed_sequence_ignores_stale_review_intent_event_source(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_source_freshness(stale_names={"review_intent_service"})
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={
                "intent": "sequence",
                "steps": [
                    {"intent": "go_home"},
                    {"intent": "wait", "wait_duration_sec": 1.0},
                ],
            },
        )

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home then wait 1 s",
            mode="sim",
        )

        self.assertTrue(response["accepted"], response.get("reason"))
        self.assertEqual(response["jobType"], "sequence")

    def test_missing_structured_fields_fail_closed_with_operator_visible_reason(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="",
            structured_intent={
                "primitive_type": "MOVE_JOINT",
                "joint_index": 1,
            },
            mode="sim",
        )
        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertIn("structuredIntent is not accepted", response["reason"])

    def test_external_structured_primitive_type_payload_is_rejected(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="",
            structured_intent={
                "primitive_type": "MOVE_JOINT",
                "joint_index": 1,
                "joint_angle": 0.1,
            },
            mode="sim",
        )

        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertIn("structuredIntent is not accepted", response["reason"])

    def test_preflight_failures_reject_command_with_explicit_reason(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_preflight(
            accepted=False,
            reasons=["required telemetry source joint_states_fallback is stale."],
        )
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        self.assertFalse(response["accepted"])
        self.assertIn("joint_states_fallback", response["reason"])

    def test_fast_path_review_still_requires_hardware_preflight(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
        )
        supervisor.bind_telemetry_service(self.telemetry)
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        self.adapter.set_review_result(
            accepted=True,
            semanticIr={
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": 0.0, "z": 5.0},
                "linear_unit": "cm",
                "reference_frame": "base_link",
            },
        )
        self.adapter.set_preflight(
            accepted=False,
            reasons=["required telemetry source robot_status is stale."],
        )
        lease_token = self._acquire_lease()

        response = supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move up 5 cm",
            mode="hardware",
        )

        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertIn("robot_status", response["reason"])
        self.assertEqual(self.adapter.confirm_calls, [])

    def test_confirmation_expires_correctly(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        command_id = response["commandId"]
        command = self.supervisor._commands[command_id]
        command.confirmation_expires_at = command.created_at
        expired = self.supervisor.get_command(command_id)
        self.assertEqual(expired["lifecycleState"], "EXPIRED")

    def test_mismatched_plan_fingerprint_and_session_fail_closed(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        command_id = response["commandId"]
        with self.assertRaises(ConflictError):
            self.supervisor.confirm_command(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                command_id=command_id,
                plan_fingerprint="wrong-fingerprint",
            )
        other_lease = self.session_lock.acquire_controller(
            "session-b", "operator-b", force_takeover=True, takeover_reason="test"
        )
        with self.assertRaises(ForbiddenActionError):
            self.supervisor.cancel_command(
                session_id="session-b",
                operator_id="operator-b",
                lease_token=other_lease.lease_token,
                command_id=command_id,
                reason="forbidden",
            )

    def test_confirmed_command_reaches_execution_boundary_only_after_checks(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move up 10 cm",
            mode="sim",
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            plan_fingerprint=response["command"]["planFingerprint"],
        )
        self.assertTrue(confirm_response["accepted"])
        self.assertEqual(len(self.adapter.confirm_calls), 1)
        self.assertEqual(confirm_response["command"]["lifecycleState"], "SUCCEEDED")
        self.assertEqual(confirm_response["command"]["finalState"], "SUCCEEDED")

    def test_get_pose_uses_query_service_without_motion_execution(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="get pose",
            mode="sim",
        )

        self.assertTrue(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "SUCCEEDED")
        self.assertEqual(response["command"]["finalState"], "SUCCEEDED")
        self.assertEqual(self.adapter.confirm_calls, [])
        self.assertEqual(
            self.adapter.get_pose_calls, [{"reference_frame": "base_link"}]
        )
        self.assertTrue(response["command"]["executionResult"]["queryOnly"])
        self.assertEqual(
            response["command"]["executionResult"]["pose"],
            {
                "position": {"x": 0.30, "y": 0.00, "z": 0.30},
                "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
            },
        )
        self.assertEqual(
            response["command"]["executionResult"]["poseMm"],
            {
                "position": {"x": 300.0, "y": 0.0, "z": 300.0},
                "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
            },
        )
        self.assertIn("x=300.0 mm", response["command"]["executionResult"]["summary"])

    def test_get_pose_completes_as_read_only_query_during_submit(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="get pose",
            mode="sim",
        )

        self.assertTrue(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "SUCCEEDED")
        self.assertEqual(response["command"]["finalState"], "SUCCEEDED")
        self.assertIsNone(response["command"]["confirmationExpiresAt"])
        self.assertEqual(self.adapter.confirm_calls, [])
        self.assertEqual(
            self.adapter.get_pose_calls, [{"reference_frame": "base_link"}]
        )
        self.assertTrue(response["command"]["executionResult"]["queryOnly"])
        self.assertEqual(response["command"]["executionResult"]["status"], "succeeded")

    def test_get_pose_ignores_gateway_and_validate_service_staleness(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        self.adapter.set_source_freshness(
            stale_names={"gateway_status", "supervisor_alerts"}
        )
        self.adapter.set_preflight(
            accepted=False,
            reasons=[
                "required telemetry source gateway_status is stale.",
                "required telemetry source supervisor_alerts is stale.",
                "required telemetry source validate_command_service is inactive.",
                "required telemetry source validate_command_service is stale.",
            ],
        )

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="get pose",
            mode="hardware",
        )

        self.assertTrue(response["accepted"], response.get("reason"))
        self.assertEqual(response["command"]["lifecycleState"], "SUCCEEDED")
        self.assertEqual(response["command"]["finalState"], "SUCCEEDED")
        self.assertEqual(
            self.adapter.get_pose_calls[-1], {"reference_frame": "base_link"}
        )

    def test_pose_query_execution_result_matches_api_contract(self) -> None:
        CommandExecutionResultModel.model_validate(
            {
                "accepted": True,
                "adapter": "workspace_ros_adapter",
                "status": "succeeded",
                "summary": "GET_POSE result: x=300.0 mm, y=0.0 mm, z=300.0 mm in base_link.",
                "dispatchedToRos": False,
                "queryOnly": True,
                "referenceFrame": "base_link",
                "pose": {
                    "position": {"x": 0.30, "y": 0.00, "z": 0.30},
                    "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
                },
                "poseMm": {
                    "position": {"x": 300.0, "y": 0.0, "z": 300.0},
                    "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
                },
            }
        )

    def test_rejected_command_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="draw a dragon on the table",
            mode="sim",
        )
        self.assertFalse(response["accepted"])
        self.assertEqual(response["command"]["lifecycleState"], "REJECTED")
        self.assertEqual(response["command"]["finalState"], "REJECTED")
        self.assertIsNotNone(response["command"]["rejectReason"])

    def test_expired_command_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        command_id = response["commandId"]
        command = self.supervisor._commands[command_id]
        command.confirmation_expires_at = command.created_at
        expired = self.supervisor.get_command(command_id)
        self.assertEqual(expired["lifecycleState"], "EXPIRED")
        self.assertEqual(expired["finalState"], "EXPIRED")
        self.assertEqual(expired["rejectReason"], "confirmation window expired")

    def test_cancelled_command_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        cancel_response = self.supervisor.cancel_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            reason="operator cancelled review",
        )
        self.assertTrue(cancel_response["accepted"])
        self.assertEqual(cancel_response["command"]["lifecycleState"], "CANCELLED")
        self.assertEqual(cancel_response["command"]["finalState"], "CANCELLED")
        self.assertEqual(
            cancel_response["command"]["rejectReason"], "operator cancelled review"
        )

    def test_failed_command_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_confirm_result(
            accepted=False,
            status="failed",
            summary="ExecuteMotion failed inside fake adapter.",
            dispatchedToRos=True,
        )
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move up 10 cm",
            mode="sim",
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            plan_fingerprint=response["command"]["planFingerprint"],
        )
        self.assertFalse(confirm_response["accepted"])
        self.assertEqual(confirm_response["command"]["lifecycleState"], "FAILED")
        self.assertEqual(confirm_response["command"]["finalState"], "FAILED")
        self.assertEqual(
            confirm_response["command"]["rejectReason"],
            "ExecuteMotion failed inside fake adapter.",
        )

    def test_cancelled_execution_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_confirm_result(
            accepted=False,
            status="cancelled",
            summary="Execution was cancelled by operator.",
            dispatchedToRos=True,
        )
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move up 10 cm",
            mode="sim",
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            plan_fingerprint=response["command"]["planFingerprint"],
        )
        self.assertFalse(confirm_response["accepted"])
        self.assertEqual(confirm_response["command"]["lifecycleState"], "CANCELLED")
        self.assertEqual(confirm_response["command"]["finalState"], "CANCELLED")
        self.assertEqual(
            confirm_response["command"]["rejectReason"],
            "Execution was cancelled by operator.",
        )

    def test_succeeded_terminal_event_carries_final_state(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            plan_fingerprint=response["command"]["planFingerprint"],
        )
        self.assertTrue(confirm_response["accepted"])
        self.assertEqual(confirm_response["command"]["lifecycleState"], "SUCCEEDED")
        self.assertEqual(confirm_response["command"]["finalState"], "SUCCEEDED")
        self.assertIsNone(confirm_response["command"]["rejectReason"])

    def test_nonaccepted_confirmation_gate_does_not_expose_false_terminal_state(
        self,
    ) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        self.adapter.set_source_freshness(stale_names={"joint_states_fallback"})
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            plan_fingerprint=response["command"]["planFingerprint"],
        )
        self.assertFalse(confirm_response["accepted"])
        self.assertEqual(
            confirm_response["command"]["lifecycleState"], "NEEDS_CONFIRMATION"
        )
        self.assertIsNone(confirm_response["command"]["finalState"])

    def test_execution_adapter_failure_marks_command_failed(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_confirm_result(
            accepted=False,
            status="failed",
            summary="ExecuteMotion failed inside fake adapter.",
            dispatchedToRos=True,
        )
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="move up 10 cm",
            mode="sim",
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            plan_fingerprint=response["command"]["planFingerprint"],
        )
        self.assertFalse(confirm_response["accepted"])
        self.assertEqual(confirm_response["command"]["lifecycleState"], "FAILED")

    def test_cancel_works_for_pending_commands(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        cancel_response = self.supervisor.cancel_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            reason="operator cancelled review",
        )
        self.assertTrue(cancel_response["accepted"])
        self.assertEqual(cancel_response["command"]["lifecycleState"], "CANCELLED")
        self.assertEqual(self.adapter.abort_calls, [])

    def test_audit_trail_records_major_transitions(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response["commandId"],
            plan_fingerprint=response["command"]["planFingerprint"],
        )
        detail = self.audit.get_command_detail(response["commandId"])
        self.assertIsNotNone(detail)
        transitions = [row["to_state"] for row in detail["timeline"]]
        self.assertEqual(
            transitions,
            [
                "RECEIVED",
                "PARSING",
                "VALIDATING",
                "NEEDS_CONFIRMATION",
                "CONFIRMED",
                "EXECUTION_REQUESTED",
                "EXECUTING",
                "SUCCEEDED",
            ],
        )
        runtime_messages = [row["message"] for row in detail["runtime_events"]]
        self.assertIn("validation result recorded", runtime_messages)
        self.assertIn("execution boundary response recorded", runtime_messages)

    def test_step_messages_exist_for_parse_validate_confirm_and_result(self) -> None:
        lease_token = self._acquire_lease()
        submit_response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )
        self.assertTrue(submit_response["accepted"])
        submit_messages = [
            msg["text"] for msg in submit_response["snapshot"]["messages"]
        ]
        self.assertTrue(any("Step 1/6 PARSING" in text for text in submit_messages))
        self.assertTrue(any("Step 2/6 VALIDATING" in text for text in submit_messages))
        self.assertTrue(
            any("Step 3/6 NEEDS_CONFIRMATION" in text for text in submit_messages)
        )

        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=submit_response["commandId"],
            plan_fingerprint=submit_response["command"]["planFingerprint"],
        )
        confirm_messages = [
            msg["text"] for msg in confirm_response["snapshot"]["messages"]
        ]
        self.assertTrue(any("Step 4/6 CONFIRMED" in text for text in confirm_messages))
        self.assertTrue(
            any("Step 5/6 EXECUTION_REQUESTED" in text for text in confirm_messages)
        )
        self.assertTrue(any("Step 6/6 RESULT" in text for text in confirm_messages))

    def test_step_messages_include_source_labels_and_match_api_contract(self) -> None:
        lease_token = self._acquire_lease()
        submit_response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home",
            mode="sim",
        )

        parsed = CommandMutationResponseModel.model_validate(submit_response)
        sources_by_text = {
            msg.text: msg.source for msg in parsed.snapshot.messages if msg.source
        }

        self.assertEqual(sources_by_text["home"], "operator")
        self.assertTrue(
            any(
                source == "safety" and "VALIDATING" in text
                for text, source in sources_by_text.items()
            )
        )

    def test_terminal_command_trace_logs_are_human_readable(self) -> None:
        lease_token = self._acquire_lease()
        with self.assertLogs("uvicorn.error", level="INFO") as captured:
            response = self.supervisor.submit_intent(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                raw_text="home",
                mode="sim",
            )
            self.supervisor.confirm_command(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                command_id=response["commandId"],
                plan_fingerprint=response["command"]["planFingerprint"],
            )

        output = "\n".join(captured.output)
        self.assertIn("[HMI CMD] request.received", output)
        self.assertIn("[HMI CMD] parse.accepted", output)
        self.assertIn("[HMI CMD] validation.accepted", output)
        self.assertIn("[HMI CMD] confirmation.accepted", output)
        self.assertIn("[HMI CMD] execution.requested", output)
        self.assertIn("[HMI CMD] terminal.succeeded", output)

    def test_rejection_trace_logs_include_reason(self) -> None:
        lease_token = self._acquire_lease()
        with self.assertLogs("uvicorn.error", level="INFO") as captured:
            self.supervisor.submit_intent(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                raw_text="draw a dragon on the table",
                mode="sim",
            )

        output = "\n".join(captured.output)
        self.assertIn("[HMI CMD] terminal.rejected", output)
        self.assertIn("review_intent did not accept this command", output)

    def test_text_sequence_creates_parent_and_child_steps(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home, wait 1 s, then move up 1 cm",
            mode="sim",
        )
        self.assertTrue(response["accepted"])
        self.assertEqual(response["jobType"], "sequence")
        self.assertIsNotNone(response["sequence"])
        self.assertEqual(response["sequence"]["stepCount"], 3)
        self.assertEqual(response["sequence"]["lifecycleState"], "NEEDS_CONFIRMATION")
        self.assertEqual(
            [step["parsedIntent"]["action"] for step in response["sequence"]["steps"]],
            ["HOME", "WAIT", "MOVE_REL"],
        )
        self.assertIsNotNone(response["snapshot"]["activeSequence"])
        self.assertEqual(
            response["snapshot"]["activeSequence"]["sequenceId"], response["sequenceId"]
        )

    def test_text_sequence_response_matches_api_contract(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home, wait 1 s, then move up 1 cm",
            mode="sim",
        )

        parsed = CommandMutationResponseModel.model_validate(response)

        self.assertIsNotNone(parsed.sequence)
        self.assertIsNotNone(parsed.sequence.validationResult)
        self.assertTrue(parsed.sequence.validationResult.hardwareGate.unlocked)
        self.assertIsNotNone(parsed.snapshot)
        self.assertIsNotNone(parsed.snapshot.activeSequence)
        self.assertIsNotNone(parsed.snapshot.activeSequence.validationResult)
        self.assertTrue(
            parsed.snapshot.activeSequence.validationResult.hardwareGate.unlocked
        )

    def test_sequence_confirm_executes_child_steps_in_order(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home, wait 1 s, then move up 1 cm",
            mode="sim",
        )
        confirm_response = self.supervisor.confirm_sequence(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            sequence_id=response["sequenceId"],
            plan_fingerprint=response["sequence"]["planFingerprint"],
        )
        self.assertTrue(confirm_response["accepted"])
        self.assertEqual(confirm_response["jobType"], "sequence")
        self.assertEqual(confirm_response["sequence"]["finalState"], "SUCCEEDED")
        self.assertEqual(confirm_response["sequence"]["currentStepIndex"], 2)
        self.assertEqual(len(self.adapter.confirm_calls), 3)
        self.assertEqual(
            [call["parsed_intent"]["action"] for call in self.adapter.confirm_calls],
            ["HOME", "WAIT", "MOVE_REL"],
        )

    def test_sequence_blocks_new_submission_until_terminal(self) -> None:
        lease_token = self._acquire_lease()
        self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="home, wait 1 s, then move up 1 cm",
            mode="sim",
        )
        with self.assertRaises(ConflictError):
            self.supervisor.submit_intent(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                raw_text="stop",
                mode="sim",
            )

    def test_structured_draw_shape_enters_sequence_path_and_preserves_macro_summary(
        self,
    ) -> None:
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={
                "intent": "draw_shape",
                "shape_type": "circle",
                "units": "mm",
                "frame_id": "base_link",
                "params": {"radius": 20},
            },
        )
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="draw a 20 mm circle",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(response["sequence"]["planSummary"]["macroName"], "draw_shape")
        self.assertEqual(response["sequence"]["planSummary"]["shapeType"], "circle")
        self.assertIn("Draw circle", response["sequence"]["summaryLabel"])
        self.assertGreater(response["sequence"]["stepCount"], 1)
        for step in response["sequence"]["steps"]:
            normalized = step["parsedIntent"]["normalizedCommand"]
            self.assertNotIn("plan_only", normalized)
            self.assertNotIn("chunk_index", normalized)
            self.assertNotIn("stroke_index", normalized)

    def test_text_draw_request_uses_current_pose_default_tool_workplane(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="write GP4",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(response["sequence"]["planSummary"]["macroName"], "draw_text")
        self.assertEqual(response["sequence"]["planSummary"]["text"], "GP4")
        self.assertIn("Draw text", response["sequence"]["summaryLabel"])
        hydrated_origin = response["sequence"]["structuredIntent"]["workplane"][
            "origin"
        ]
        self.assertAlmostEqual(hydrated_origin["position"]["x"], 0.30)
        self.assertAlmostEqual(hydrated_origin["position"]["y"], 0.00)
        self.assertAlmostEqual(hydrated_origin["position"]["z"], 0.30)

    def test_vietnamese_draw_text_routes_to_sequence(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="vẽ chữ gp4",
            mode="sim",
        )

        self.assertTrue(response["accepted"], msg=response)
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(response["sequence"]["planSummary"]["macroName"], "draw_text")
        self.assertEqual(response["sequence"]["planSummary"]["text"], "GP4")

    def test_reviewed_draw_text_over_sequence_budget_rejects_before_execution(
        self,
    ) -> None:
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={
                "intent": "draw_text",
                "text": "GP4GP4GP4",
                "units": "mm",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "font": {"type": "single_stroke_builtin", "height": 20},
            },
        )
        lease_token = self._acquire_lease()

        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="write the long GP4 label",
            mode="sim",
        )

        self.assertFalse(response["accepted"])
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(response["sequence"]["finalState"], "REJECTED")
        self.assertIn("sequence_length", response["reason"])
        self.assertEqual(self.adapter.confirm_calls, [])

    def test_draw_plan_only_is_rejected_as_sequence(self) -> None:
        self.adapter.set_review_result(
            accepted=True,
            adapter="fake-gateway-review",
            semanticIr={
                "intent": "draw_text",
                "text": "GP4",
                "units": "mm",
                "frame_id": "base_link",
                "execution_mode": "plan_only",
                "font": {"type": "single_stroke_builtin", "height": 20},
            },
        )
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="write GP4 as a plan-only drawing",
            mode="sim",
        )

        self.assertFalse(response["accepted"])
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(response["sequence"]["finalState"], "REJECTED")
        self.assertIn("plan_only", response["reason"])

    def test_draw_rejects_when_current_pose_is_unavailable(self) -> None:
        self.adapter.set_current_pose(None)
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text="write GP4",
            mode="sim",
        )

        self.assertFalse(response["accepted"])
        self.assertEqual(response["jobType"], "sequence")
        self.assertEqual(response["sequence"]["finalState"], "REJECTED")
        self.assertIn("/get_current_pose", response["reason"])


if __name__ == "__main__":
    unittest.main()
