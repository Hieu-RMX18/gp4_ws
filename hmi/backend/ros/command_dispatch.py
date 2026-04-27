from __future__ import annotations
import json
import time
from typing import Any
from ..domain.models import RuntimeMode, SystemRuntimeState, TelemetryFreshnessState
try:
    from geometry_msgs.msg import Pose
    from interfaces.action import ExecuteMotion
    from interfaces.srv import ValidateCommand
    from rclpy.action import ActionClient
except Exception:  # pragma: no cover - depends on sourced ROS environment
    Pose = None
    ExecuteMotion = None
    ValidateCommand = None
    ActionClient = None
DEFAULT_MOTION_VELOCITY_SCALE = 0.06
DEFAULT_MOTION_ACCELERATION_SCALE = 0.06
DEFAULT_VALIDATE_TIMEOUT_SEC = 5.0
DEFAULT_ACTION_WAIT_TIMEOUT_SEC = 5.0
DEFAULT_EXECUTION_TIMEOUT_SEC = 120.0
class CommandDispatchMixin:
    """ValidateCommand + ExecuteMotion dispatch methods for WorkspaceRosAdapter."""

    def confirm_command(
        self,
        *,
        command_id: str,
        plan_fingerprint: str,
        operator_id: str,
        session_id: str,
        lease_id: str,
        correlation_id: str,
        parsed_intent: dict[str, Any] | None = None,
        requested_mode: str | None = None,
    ) -> dict[str, Any]:
        self._trace(
            "confirm.request",
            command_id=command_id,
            correlation_id=correlation_id,
            plan_fingerprint=plan_fingerprint,
            operator_id=operator_id,
            session_id=session_id,
        )
        runtime = self.read_runtime_snapshot()
        if requested_mode is not None and requested_mode != runtime.mode.value:
            reason = (
                f"requested mode {requested_mode} does not match runtime mode {runtime.mode.value}."
            )
            self._trace(
                "confirm.blocked",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=reason,
                runtime_mode=runtime.mode.value,
            )
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary=reason,
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )

        preflight = self.evaluate_execution_preflight(target_mode=runtime.mode.value)
        if not preflight["accepted"]:
            reason = "; ".join(preflight["reasons"]) or "execution preflight failed"
            self._trace(
                "confirm.blocked",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=reason,
                runtime_mode=runtime.mode.value,
                preflight=preflight,
            )
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary=reason,
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )

        if parsed_intent is None:
            self._trace(
                "confirm.blocked",
                command_id=command_id,
                correlation_id=correlation_id,
                reason="parsed intent missing at execution boundary",
            )
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary='Parsed intent is unavailable at the execution boundary.',
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )

        try:
            command_payload = self._build_command_payload(parsed_intent)
        except ValueError as exc:
            self._trace(
                "payload.build_failed",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=str(exc),
                parsed_intent=parsed_intent,
            )
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary=str(exc),
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )

        validation_result = self._validate_motion_request(
            command_payload,
            command_id=command_id,
            correlation_id=correlation_id,
        )
        if not validation_result["accepted"]:
            self._trace(
                "validate.rejected",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=validation_result["summary"],
            )
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary=validation_result["summary"],
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )

        return self._dispatch_execute_motion(
            command_id=command_id,
            plan_fingerprint=plan_fingerprint,
            operator_id=operator_id,
            session_id=session_id,
            lease_id=lease_id,
            correlation_id=correlation_id,
            command_payload=command_payload,
        )

    def abort_command(self, *, command_id: str) -> tuple[bool, str]:
        with self._goal_lock:
            goal_handle = self._goal_handles.get(command_id)
            correlation_id = self._goal_correlation_ids.get(command_id)

        if goal_handle is None:
            self._trace(
                "abort.skipped",
                command_id=command_id,
                correlation_id=correlation_id,
                reason="no active ExecuteMotion goal for command",
            )
            return True, f"Command {command_id} cancelled before any ROS execution request."

        try:
            cancel_future = goal_handle.cancel_goal_async()
            wrapped = self._wait_for_future(cancel_future, DEFAULT_ACTION_WAIT_TIMEOUT_SEC)
        except Exception as exc:
            self._trace(
                "abort.failed",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=str(exc),
            )
            return False, f"Failed to cancel ExecuteMotion goal for {command_id}: {exc}"

        goals_canceling = getattr(wrapped, 'goals_canceling', [])
        if goals_canceling:
            self._trace(
                "abort.accepted",
                command_id=command_id,
                correlation_id=correlation_id,
                summary="ExecuteMotion cancellation accepted by action server",
            )
            return True, f"Cancellation requested for ExecuteMotion goal {command_id}."
        self._trace(
            "abort.rejected",
            command_id=command_id,
            correlation_id=correlation_id,
            summary="ExecuteMotion cancellation was not accepted by action server",
        )
        return False, f"ExecuteMotion goal {command_id} did not accept cancellation."

    def _execution_response(
        self,
        *,
        accepted: bool,
        status: str,
        summary: str,
        command_id: str,
        plan_fingerprint: str,
        operator_id: str,
        session_id: str,
        lease_id: str,
        correlation_id: str,
        dispatched_to_ros: bool,
    ) -> dict[str, Any]:
        return {
            "accepted": accepted,
            "adapter": "workspace_ros_adapter",
            "status": status,
            "summary": summary,
            "commandId": command_id,
            "planFingerprint": plan_fingerprint,
            "operatorId": operator_id,
            "sessionId": session_id,
            "leaseId": lease_id,
            "correlationId": correlation_id,
            "dispatchedToRos": dispatched_to_ros,
        }

    def evaluate_execution_preflight(self, *, target_mode: str | None = None) -> dict[str, Any]:
        runtime = self.read_runtime_snapshot()
        requested_mode_text = str(target_mode or runtime.mode.value).strip().lower() or runtime.mode.value
        requested_mode = RuntimeMode(requested_mode_text) if requested_mode_text in RuntimeMode._value2member_map_ else RuntimeMode.UNKNOWN
        source_statuses = self.read_source_statuses()
        source_map = {source.name: source for source in source_statuses}
        reasons: list[str] = []

        if requested_mode not in {RuntimeMode.SIM, RuntimeMode.HARDWARE}:
            reasons.append(f"runtime mode {requested_mode.value} is not command-capable.")

        if runtime.mode != requested_mode:
            reasons.append(
                f"requested mode {requested_mode.value} does not match runtime mode {runtime.mode.value}."
            )

        if runtime.system_state != SystemRuntimeState.NORMAL:
            reasons.append(f"runtime state {runtime.system_state.value} blocks execution.")

        required_sources: list[str]
        if requested_mode == RuntimeMode.SIM:
            required_sources = [
                "gateway_status",
                "readiness",
                "supervisor_alerts",
                "joint_states_fallback",
                "validate_command_service",
                "execute_motion_action",
            ]
        elif requested_mode == RuntimeMode.HARDWARE:
            required_sources = [
                "gateway_status",
                "readiness",
                "supervisor_alerts",
                "robot_status",
                "joint_states_primary",
                "validate_command_service",
                "execute_motion_action",
            ]
        else:
            required_sources = []

        for source_name in required_sources:
            source = source_map.get(source_name)
            if source is None:
                reasons.append(f"required telemetry source {source_name} is missing.")
                continue
            if not source.active:
                reasons.append(f"required telemetry source {source_name} is inactive.")
            if source.freshness_state != TelemetryFreshnessState.FRESH:
                reasons.append(
                    f"required telemetry source {source_name} is {source.freshness_state.value}."
                )

        if requested_mode == RuntimeMode.HARDWARE:
            primary_joint_source = source_map.get("joint_states_primary")
            if primary_joint_source is not None:
                if not primary_joint_source.preferred:
                    reasons.append("joint_states_primary is not marked preferred in hardware mode.")
                if not primary_joint_source.active:
                    reasons.append("joint_states_primary is not the active joint source in hardware mode.")

            robot_status = source_map.get("robot_status")
            if robot_status is not None and not robot_status.active:
                reasons.append("robot_status source must stay active in hardware mode.")

        return {
            "accepted": len(reasons) == 0,
            "mode": requested_mode.value,
            "reasons": reasons,
            "requiredSources": required_sources,
            "runtimeState": runtime.system_state.value,
            "sourceStatuses": [
                {
                    "name": source.name,
                    "freshnessState": source.freshness_state.value,
                    "active": source.active,
                    "preferred": source.preferred,
                    "detail": source.detail,
                }
                for source in source_statuses
            ],
        }

    def _build_command_payload(self, parsed_intent: dict[str, Any]) -> dict[str, Any]:
        normalized_command = parsed_intent.get("normalizedCommand")
        if isinstance(normalized_command, dict) and normalized_command.get("primitive_type"):
            command_payload = dict(normalized_command)
            return command_payload

        action = str(parsed_intent.get("action") or "").strip()
        parameters = dict(parsed_intent.get("parameters") or {})

        if parsed_intent.get("primitive_type"):
            return dict(parsed_intent)

        primitive_action = action.upper()
        if primitive_action in {
            "HOME",
            "PTP",
            "LIN",
            "CIRC",
            "CARTESIAN_PATH",
            "MOVE_REL",
            "MOVE_JOINT",
            "MOVE_JOINTS",
            "WAIT",
            "STOP",
            "SET_SPEED",
            "IO_SET",
            "ALARM_RESET",
            "GET_POSE",
        }:
            payload = {"primitive_type": primitive_action}
            payload.update(parameters)
            return payload

        if action == "move_home":
            return {
                "primitive_type": "HOME",
                "velocity_scale": DEFAULT_MOTION_VELOCITY_SCALE,
                "acceleration_scale": DEFAULT_MOTION_ACCELERATION_SCALE,
                "planner_id": "PILZ_PTP",
                "reference_frame": "base_link",
            }

        if action == "stop":
            return {
                "primitive_type": "STOP",
                "reference_frame": "base_link",
            }

        if action == "move_cartesian_delta":
            return self._build_move_cartesian_delta_payload(parameters)

        if action == "move_joint_delta":
            return self._build_move_joint_delta_payload(parameters)

        raise ValueError(f"Unsupported supervisor action '{action}'.")

    def _validate_motion_request(
        self,
        command_payload: dict[str, Any],
        *,
        command_id: str,
        correlation_id: str,
    ) -> dict[str, Any]:
        self._trace(
            "validate.request",
            command_id=command_id,
            correlation_id=correlation_id,
            payload=command_payload,
        )
        if self._node is None or ValidateCommand is None:
            return {
                "accepted": False,
                "summary": "ROS node is unavailable; ValidateCommand cannot be called.",
            }
        if self._validate_client is None:
            return {
                "accepted": False,
                "summary": "ValidateCommand client is not initialized.",
            }
        if not self._validate_client.wait_for_service(timeout_sec=DEFAULT_VALIDATE_TIMEOUT_SEC):
            return {
                "accepted": False,
                "summary": f"ValidateCommand service unavailable at {self._validate_command_service}.",
            }

        request = ValidateCommand.Request()
        request.command_json = json.dumps(command_payload, ensure_ascii=True, separators=(",", ":"))
        request.primitive_type = str(command_payload["primitive_type"])
        request.velocity_scale = float(command_payload.get("velocity_scale", 0.0))
        target_pose_payload = self._cartesian_path_target_pose(command_payload)
        if target_pose_payload is not None and Pose is not None:
            pose = self._dict_to_pose(target_pose_payload)
            request.target_pose = pose

        try:
            response = self._wait_for_future(
                self._validate_client.call_async(request),
                DEFAULT_VALIDATE_TIMEOUT_SEC,
            )
        except Exception as exc:
            self._trace(
                "validate.error",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=str(exc),
            )
            return {
                "accepted": False,
                "summary": f"ValidateCommand call failed: {exc}",
            }

        if not response.valid:
            self._trace(
                "validate.response",
                command_id=command_id,
                correlation_id=correlation_id,
                valid=False,
                reason=response.reason or "ValidateCommand rejected the request.",
            )
            return {
                "accepted": False,
                "summary": response.reason or "ValidateCommand rejected the request.",
            }

        if response.sanitized_json:
            try:
                sanitized_payload = json.loads(response.sanitized_json)
            except json.JSONDecodeError:
                sanitized_payload = None
            if isinstance(sanitized_payload, dict):
                command_payload.clear()
                command_payload.update(sanitized_payload)
        self._trace(
            "validate.response",
            command_id=command_id,
            correlation_id=correlation_id,
            valid=True,
            sanitized_payload=command_payload,
        )
        return {
            "accepted": True,
            "summary": "ValidateCommand accepted the request.",
        }

    def _dispatch_execute_motion(
        self,
        *,
        command_id: str,
        plan_fingerprint: str,
        operator_id: str,
        session_id: str,
        lease_id: str,
        correlation_id: str,
        command_payload: dict[str, Any],
    ) -> dict[str, Any]:
        self._trace(
            "execute.request",
            command_id=command_id,
            correlation_id=correlation_id,
            payload=command_payload,
            action_name=self._execute_motion_action,
        )
        if self._node is None or ExecuteMotion is None or ActionClient is None:
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary='ROS node is unavailable; ExecuteMotion cannot be called.',
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )
        if self._execute_client is None:
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary='ExecuteMotion client is not initialized.',
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )
        if not self._execute_client.wait_for_server(timeout_sec=DEFAULT_ACTION_WAIT_TIMEOUT_SEC):
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary=f'ExecuteMotion action server unavailable at {self._execute_motion_action}.',
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )

        goal = self._build_execute_motion_goal(command_payload)
        try:
            goal_handle = self._wait_for_future(
                self._execute_client.send_goal_async(goal),
                DEFAULT_ACTION_WAIT_TIMEOUT_SEC,
            )
        except Exception as exc:
            self._trace(
                "execute.send_failed",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=str(exc),
            )
            return self._execution_response(
                accepted=False,
                status='failed',
                summary=f'ExecuteMotion send failed: {exc}',
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )

        if goal_handle is None or not goal_handle.accepted:
            self._trace(
                "execute.rejected",
                command_id=command_id,
                correlation_id=correlation_id,
                reason="ExecuteMotion action server rejected the goal",
            )
            return self._execution_response(
                accepted=False,
                status='blocked',
                summary='ExecuteMotion action server rejected the goal.',
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=False,
            )

        with self._goal_lock:
            self._goal_handles[command_id] = goal_handle
            self._goal_correlation_ids[command_id] = correlation_id
        self._trace(
            "execute.accepted",
            command_id=command_id,
            correlation_id=correlation_id,
            summary="ExecuteMotion goal accepted by action server",
        )
        try:
            wrapped_result = self._wait_for_future(
                goal_handle.get_result_async(),
                DEFAULT_EXECUTION_TIMEOUT_SEC,
            )
        except Exception as exc:
            self._trace(
                "execute.result_failed",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=str(exc),
            )
            return self._execution_response(
                accepted=False,
                status='failed',
                summary=f'ExecuteMotion result wait failed: {exc}',
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=True,
            )
        finally:
            with self._goal_lock:
                self._goal_handles.pop(command_id, None)
                self._goal_correlation_ids.pop(command_id, None)

        result = getattr(wrapped_result, 'result', None)
        if result is None:
            self._trace(
                "execute.result_failed",
                command_id=command_id,
                correlation_id=correlation_id,
                reason="ExecuteMotion returned no result payload",
            )
            return self._execution_response(
                accepted=False,
                status='failed',
                summary='ExecuteMotion returned no result payload.',
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=True,
            )

        if getattr(result, 'success', False):
            self._trace(
                "execute.result",
                command_id=command_id,
                correlation_id=correlation_id,
                status="succeeded",
                summary=str(result.message or 'ExecuteMotion completed successfully.'),
            )
            return self._execution_response(
                accepted=True,
                status='succeeded',
                summary=str(result.message or 'ExecuteMotion completed successfully.'),
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=True,
            )

        summary = str(result.message or 'ExecuteMotion failed.')
        if 'cancel' in summary.lower():
            self._trace(
                "execute.result",
                command_id=command_id,
                correlation_id=correlation_id,
                status="cancelled",
                summary=summary,
            )
            return self._execution_response(
                accepted=False,
                status='cancelled',
                summary=summary,
                command_id=command_id,
                plan_fingerprint=plan_fingerprint,
                operator_id=operator_id,
                session_id=session_id,
                lease_id=lease_id,
                correlation_id=correlation_id,
                dispatched_to_ros=True,
            )

        self._trace(
            "execute.result",
            command_id=command_id,
            correlation_id=correlation_id,
            status="failed",
            summary=summary,
        )
        return self._execution_response(
            accepted=False,
            status='failed',
            summary=summary,
            command_id=command_id,
            plan_fingerprint=plan_fingerprint,
            operator_id=operator_id,
            session_id=session_id,
            lease_id=lease_id,
            correlation_id=correlation_id,
            dispatched_to_ros=True,
        )

    def _build_execute_motion_goal(self, command_payload: dict[str, Any]) -> Any:
        goal = ExecuteMotion.Goal()
        goal.primitive_type = str(command_payload["primitive_type"])
        goal.velocity_scale = float(command_payload.get("velocity_scale", 0.0))
        goal.acceleration_scale = float(command_payload.get("acceleration_scale", 0.0))
        goal.planner_id = str(command_payload.get("planner_id", ""))
        goal.reference_frame = str(command_payload.get("reference_frame", ""))
        goal.delta_x = float(command_payload.get("delta_x", 0.0))
        goal.delta_y = float(command_payload.get("delta_y", 0.0))
        goal.delta_z = float(command_payload.get("delta_z", 0.0))
        goal.wait_duration_sec = float(command_payload.get("wait_duration_sec", 0.0))
        goal.joint_index = int(command_payload.get("joint_index", 0))
        goal.joint_angle = float(command_payload.get("joint_angle", 0.0))
        goal.io_address = int(command_payload.get("io_address", 0))
        goal.io_value = int(command_payload.get("io_value", 0))
        goal.joint_target = [float(value) for value in command_payload.get("joint_target", [])]

        target_pose_payload = self._cartesian_path_target_pose(command_payload)
        if target_pose_payload is not None and Pose is not None:
            goal.target_pose = self._dict_to_pose(target_pose_payload)
        if "waypoints" in command_payload and Pose is not None:
            goal.waypoints = [
                self._dict_to_pose(waypoint)
                for waypoint in command_payload.get("waypoints", [])
            ]

        return goal

    def _cartesian_path_target_pose(self, command_payload: dict[str, Any]) -> dict[str, Any] | None:
        target_pose = command_payload.get("target_pose")
        if isinstance(target_pose, dict):
            return target_pose
        if str(command_payload.get("primitive_type") or "").upper() != "CARTESIAN_PATH":
            return None
        waypoints = command_payload.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            return None
        final_waypoint = waypoints[-1]
        if not isinstance(final_waypoint, dict):
            return None
        return final_waypoint

    def _dict_to_pose(self, payload: dict[str, Any]) -> Any:
        pose = Pose()
        position = payload.get("position", {})
        orientation = payload.get("orientation", {})
        pose.position.x = float(position.get("x", 0.0))
        pose.position.y = float(position.get("y", 0.0))
        pose.position.z = float(position.get("z", 0.0))
        pose.orientation.x = float(orientation.get("x", 0.0))
        pose.orientation.y = float(orientation.get("y", 0.0))
        pose.orientation.z = float(orientation.get("z", 0.0))
        pose.orientation.w = float(orientation.get("w", 1.0))
        return pose

    def _wait_for_future(self, future: Any, timeout_sec: float) -> Any:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS future timed out.")
            time.sleep(0.05)
        return future.result()
