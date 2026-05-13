from __future__ import annotations

import json
import math
from typing import Any
from uuid import uuid4

from ..domain.models import (
    CommandKind,
    CommandLifecycleState,
    CommandRecord,
    CommandRiskLevel,
    RuntimeMode,
    RuntimeSnapshot,
)


def _utcnow():
    from datetime import datetime, timezone

    return datetime.now(timezone.utc)


# Deterministic quick-command map: quickCommandId -> Semantic IR dict.
# These bypass the LLM and go straight to IntentRouter, but still require
# all HMI gates (lease, confirmation, validation, execution).
_QUICK_COMMAND_STATIC_MAP: dict[str, dict[str, Any]] = {
    "home": {"intent": "go_home"},
    "stop": {"intent": "stop"},
    "get_pose": {"intent": "get_pose", "reference_frame": "base_link"},
    "up_5cm": {
        "intent": "move_relative",
        "delta": {"x": 0.0, "y": 0.0, "z": 0.05},
        "reference_frame": "base_link",
    },
    "down_2cm": {
        "intent": "move_relative",
        "delta": {"x": 0.0, "y": 0.0, "z": -0.02},
        "reference_frame": "base_link",
    },
    "wait_2s": {"intent": "wait", "wait_duration_sec": 2.0},
}


class SupervisorSubmissionMixin:
    """Mixin providing submit_intent, _submit_single_command, _submit_sequence."""

    def _build_joint_delta_semantic_ir(
        self, joint_deltas_deg: list[float]
    ) -> dict[str, Any] | None:
        """Build a move_joints Semantic IR from current joint positions + deltas (deg)."""
        try:
            current_joints = self._ros.read_joint_positions()
        except Exception:
            return None
        if len(current_joints) != len(joint_deltas_deg):
            return None
        joint_target: list[float] = []
        for joint, delta_deg in zip(current_joints, joint_deltas_deg):
            if joint.position_deg is None:
                return None
            joint_target.append(math.radians(joint.position_deg + delta_deg))
        return {
            "intent": "move_joints",
            "joint_target": joint_target,
            "reference_frame": "base_link",
        }

    def _resolve_quick_command_semantic_ir(
        self, quick_command_id: str
    ) -> dict[str, Any] | None:
        if quick_command_id in _QUICK_COMMAND_STATIC_MAP:
            return dict(_QUICK_COMMAND_STATIC_MAP[quick_command_id])
        if quick_command_id == "joint_1_plus_5":
            deltas_deg = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            return self._build_joint_delta_semantic_ir(deltas_deg)
        return None

    def submit_intent(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        raw_text: str | None = None,
        quick_command_id: str | None = None,
        structured_intent: dict[str, Any] | None = None,
        mode: str,
    ) -> dict[str, Any]:
        self._expire_pending_confirmations()
        raw_text = (raw_text or "").strip()
        quick_command_id = (quick_command_id or "").strip()
        if not raw_text and not quick_command_id and structured_intent is None:
            from .supervisor_service import ConflictError

            raise ConflictError("intentText or quickCommandId is required")
        if raw_text and quick_command_id:
            from .supervisor_service import ConflictError

            raise ConflictError("submit exactly one of intentText or quickCommandId")

        lease = self._assert_controller(session_id, operator_id, lease_token)
        runtime = self._current_runtime()
        requested_mode = (
            RuntimeMode(mode)
            if mode in RuntimeMode._value2member_map_
            else runtime.mode
        )
        self._assert_no_active_job()
        command_id = str(uuid4())
        correlation_id = str(uuid4())
        if structured_intent is not None:
            return self._reject_intent_submission(
                session_id=session_id,
                operator_id=operator_id,
                lease=lease,
                runtime=runtime,
                requested_mode=requested_mode,
                raw_text=raw_text,
                structured_intent=structured_intent,
                command_id=command_id,
                correlation_id=correlation_id,
                reason=(
                    "structuredIntent is not accepted from the HMI command API; "
                    "submit intentText or quickCommandId."
                ),
            )

        effective_structured_intent: dict[str, Any] | None = None
        if quick_command_id:
            effective_structured_intent = self._resolve_quick_command_semantic_ir(
                quick_command_id
            )
            if effective_structured_intent is None:
                if quick_command_id == "joint_1_plus_5":
                    reason = (
                        "Joint state unavailable or incomplete for quickCommandId "
                        "'joint_1_plus_5'."
                    )
                else:
                    reason = f"Unknown quickCommandId: '{quick_command_id}'."
                return self._reject_intent_submission(
                    session_id=session_id,
                    operator_id=operator_id,
                    lease=lease,
                    runtime=runtime,
                    requested_mode=requested_mode,
                    raw_text=raw_text,
                    structured_intent=None,
                    command_id=command_id,
                    correlation_id=correlation_id,
                    reason=reason,
                )
            self._trace(
                "quick_command.deterministic_semantic_ir",
                command_id=command_id,
                correlation_id=correlation_id,
                quick_command_id=quick_command_id,
                semantic_ir=json.dumps(effective_structured_intent),
            )
        else:
            review_result = self._submit_text_for_gateway_review(
                raw_text=raw_text,
                structured_intent=structured_intent,
                requested_mode=requested_mode,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command_id,
                correlation_id=correlation_id,
            )
            effective_structured_intent = self._semantic_ir_from_review_result(
                review_result
            )
            if effective_structured_intent is None:
                review_error = (
                    review_result.get("error")
                    if isinstance(review_result, dict)
                    else "review_intent returned no Semantic IR"
                )
                return self._reject_intent_submission(
                    session_id=session_id,
                    operator_id=operator_id,
                    lease=lease,
                    runtime=runtime,
                    requested_mode=requested_mode,
                    raw_text=raw_text,
                    structured_intent=None,
                    command_id=command_id,
                    correlation_id=correlation_id,
                    reason=(
                        "review_intent did not accept this command: "
                        f"{review_error or 'missing Semantic IR'}"
                    ),
                )
        sequence_segments: list[str] = []
        prepared_sequence = self._prepare_sequence_request(
            raw_text=raw_text,
            structured_intent=effective_structured_intent,
            requested_mode=requested_mode,
        )
        if prepared_sequence is not None or self._is_sequence_request(
            effective_structured_intent, sequence_segments
        ):
            return self._submit_sequence(
                session_id=session_id,
                operator_id=operator_id,
                lease=lease,
                runtime=runtime,
                requested_mode=requested_mode,
                raw_text=raw_text,
                structured_intent=effective_structured_intent,
                sequence_segments=sequence_segments,
                prepared_sequence=prepared_sequence,
                sequence_id=command_id,
                correlation_id=correlation_id,
            )
        return self._submit_single_command(
            session_id=session_id,
            operator_id=operator_id,
            lease=lease,
            runtime=runtime,
            requested_mode=requested_mode,
            raw_text=raw_text,
            structured_intent=effective_structured_intent,
            command_id=command_id,
            correlation_id=correlation_id,
        )

    def _submit_text_for_gateway_review(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        requested_mode: RuntimeMode,
        session_id: str,
        operator_id: str,
        command_id: str,
        correlation_id: str,
    ) -> dict[str, Any]:
        reviewer = getattr(self._ros, "submit_text_for_review", None)
        if not callable(reviewer) or not raw_text:
            return {}
        try:
            review_result = reviewer(
                raw_text=raw_text,
                runtime_mode=requested_mode.value,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command_id,
            )
        except Exception as exc:
            self._trace(
                "review.unavailable",
                command_id=command_id,
                correlation_id=correlation_id,
                session_id=session_id,
                operator_id=operator_id,
                reason=str(exc),
            )
            return {"accepted": False, "error": str(exc)}
        if not isinstance(review_result, dict):
            return {"accepted": False, "error": "review result was not a mapping"}
        if structured_intent is not None:
            return dict(review_result)
        if self._semantic_ir_from_review_result(review_result) is None:
            self._trace(
                "review.fallback_to_local_parse",
                command_id=command_id,
                correlation_id=correlation_id,
                session_id=session_id,
                operator_id=operator_id,
                accepted=review_result.get("accepted"),
                error=review_result.get("error"),
            )
        else:
            self._trace(
                "review.semantic_ir_accepted",
                command_id=command_id,
                correlation_id=correlation_id,
                session_id=session_id,
                operator_id=operator_id,
            )
        return dict(review_result)

    def _reject_intent_submission(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease: Any,
        runtime: RuntimeSnapshot,
        requested_mode: RuntimeMode,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        command_id: str,
        correlation_id: str,
        reason: str,
    ) -> dict[str, Any]:
        command = CommandRecord(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            raw_text=raw_text,
            lifecycle_state=CommandLifecycleState.RECEIVED,
            summary_label=(raw_text or "rejected structured command")[:80],
            mode=requested_mode,
            created_at=_utcnow(),
            intent_source="text" if raw_text else "structured",
            correlation_id=correlation_id,
            structured_intent=structured_intent,
        )
        command.validation_result = self._build_validation_result(
            runtime=runtime,
            lease=lease,
            parsed_intent=None,
            blocking_reasons=[reason],
            risk_level=None,
            requires_confirmation=False,
        )

        with self._lock:
            self._commands[command_id] = command
            self._active_command_id = command_id
            self._active_sequence_id = None

        self._append_message(
            origin="operator",
            text=raw_text or "Submitted structured command.",
            command_id=command_id,
        )
        self._audit.upsert_command(command)
        self._audit.record_transition(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            from_state=None,
            to_state=CommandLifecycleState.RECEIVED,
            runtime_state=runtime.system_state,
            reason="command received from HMI v2 intent endpoint",
            payload={"mode": command.mode.value, "intentSource": command.intent_source},
        )
        self._reject_command(
            command,
            reason=reason,
            runtime_state=runtime.system_state,
            session_id=session_id,
            operator_id=operator_id,
        )
        return self._command_response(
            session_id, operator_id, command, accepted=False, reason=reason
        )

    @staticmethod
    def _semantic_ir_from_review_result(
        review_result: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        if not isinstance(review_result, dict) or not review_result.get("accepted"):
            return None

        semantic_ir = review_result.get("semanticIr") or review_result.get(
            "semantic_ir"
        )
        if isinstance(semantic_ir, dict):
            return dict(semantic_ir)

        semantic_ir_json = review_result.get("semanticIrJson") or review_result.get(
            "semantic_ir_json"
        )
        if not isinstance(semantic_ir_json, str) or not semantic_ir_json.strip():
            return None
        try:
            decoded = json.loads(semantic_ir_json)
        except json.JSONDecodeError:
            return None
        if not isinstance(decoded, dict):
            return None
        return decoded

    def _submit_single_command(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease: Any,
        runtime: RuntimeSnapshot,
        requested_mode: RuntimeMode,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        command_id: str | None = None,
        correlation_id: str | None = None,
    ) -> dict[str, Any]:
        command_id = command_id or str(uuid4())
        correlation_id = correlation_id or str(uuid4())
        command = CommandRecord(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            raw_text=raw_text,
            lifecycle_state=CommandLifecycleState.RECEIVED,
            summary_label=(raw_text or "structured command")[:80],
            mode=requested_mode,
            created_at=_utcnow(),
            intent_source="text" if raw_text else "structured",
            correlation_id=correlation_id,
            structured_intent=structured_intent,
        )
        self._trace(
            "request.received",
            command_id=command_id,
            correlation_id=correlation_id,
            session_id=session_id,
            operator_id=operator_id,
            mode=command.mode.value,
            has_structured_intent=structured_intent is not None,
            raw_text=raw_text or "<structured-intent>",
        )

        with self._lock:
            self._commands[command_id] = command
            self._active_command_id = command_id
            self._active_sequence_id = None

        self._append_message(
            origin="operator",
            text=raw_text or "Submitted structured command.",
            command_id=command_id,
        )
        self._audit.upsert_command(command)
        self._audit.record_transition(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            from_state=None,
            to_state=CommandLifecycleState.RECEIVED,
            runtime_state=runtime.system_state,
            reason="command received from HMI v2 intent endpoint",
            payload={"mode": command.mode.value, "intentSource": command.intent_source},
        )

        self._transition_command(
            command,
            next_state=CommandLifecycleState.PARSING,
            reason="supervisor parsing operator intent",
            runtime_state=runtime.system_state,
            message_text=f"Step 1/6 PARSING: received intent '{raw_text or 'structured command'}'.",
            message_tag=CommandLifecycleState.PARSING.value,
        )

        parsed_intent, parse_error = self._parse_intent(
            raw_text,
            structured_intent,
            mode=command.mode,
        )
        if parsed_intent is None:
            self._trace(
                "parse.rejected",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=parse_error,
            )
            self._audit.record_runtime_event(
                system_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command.command_id,
                message="intent resolution failed",
                payload={"reason": parse_error},
            )
            command.reject_reason = parse_error
            command.validation_result = self._build_validation_result(
                runtime=runtime,
                lease=lease,
                parsed_intent=None,
                blocking_reasons=[parse_error],
                risk_level=None,
                requires_confirmation=False,
            )
            self._reject_command(
                command,
                reason=parse_error,
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._command_response(
                session_id, operator_id, command, accepted=False, reason=parse_error
            )

        command.parsed_intent = parsed_intent
        self._audit.record_runtime_event(
            system_state=runtime.system_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=command.command_id,
            message="intent resolution accepted",
            payload={
                "action": parsed_intent.get("action"),
                "normalizedCommand": parsed_intent.get("normalizedCommand"),
                "normalizationNotes": parsed_intent.get("normalizationNotes") or [],
            },
        )

        command.summary_label = parsed_intent["targetSummary"]
        command.planner_used = self._planner_for_intent(parsed_intent)
        command.frame_used = self._frame_for_intent(parsed_intent)
        self._trace(
            "parse.accepted",
            command_id=command_id,
            correlation_id=correlation_id,
            action=parsed_intent.get("action"),
            targetSummary=parsed_intent.get("targetSummary"),
            parameters=parsed_intent.get("parameters"),
            planner=command.planner_used,
            frame=command.frame_used,
        )
        self._transition_command(
            command,
            next_state=CommandLifecycleState.VALIDATING,
            reason="supervisor validating parsed intent",
            runtime_state=runtime.system_state,
            payload={"parsedIntent": parsed_intent},
            message_text=(
                f"Step 2/6 VALIDATING: action={parsed_intent.get('action')} "
                f"planner={command.planner_used or '--'} frame={command.frame_used or '--'}."
            ),
            message_tag=CommandLifecycleState.VALIDATING.value,
        )

        validation = self._validate_command(
            runtime=runtime,
            lease=lease,
            parsed_intent=parsed_intent,
            requested_mode=command.mode,
        )
        command.validation_result = validation
        command.plan_summary = {
            "normalizedIntent": parsed_intent["normalizedText"],
            "parsedAction": parsed_intent["action"],
            "targetSummary": parsed_intent["targetSummary"],
            "requiresConfirmation": validation["requiresConfirmation"],
            "normalizedCommand": parsed_intent.get("normalizedCommand"),
        }
        command.plan_fingerprint = validation["planFingerprint"]
        command.risk_level = (
            CommandRiskLevel(validation["riskLevel"])
            if validation["riskLevel"] is not None
            else None
        )
        self._audit.upsert_command(command)
        self._audit.record_runtime_event(
            system_state=runtime.system_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=command.command_id,
            message="validation result recorded",
            payload=validation,
        )
        if validation["accepted"]:
            self._trace(
                "validation.accepted",
                command_id=command_id,
                correlation_id=correlation_id,
                risk_level=validation.get("riskLevel"),
                confirmation_reasons=validation.get("confirmationReasons"),
                plan_fingerprint=validation.get("planFingerprint"),
            )
        else:
            self._trace(
                "validation.rejected",
                command_id=command_id,
                correlation_id=correlation_id,
                blocking_reasons=validation.get("blockingReasons"),
                risk_level=validation.get("riskLevel"),
            )
        if not validation["accepted"]:
            reject_reason = (
                "; ".join(validation["blockingReasons"]) or "validation failed"
            )
            command.reject_reason = reject_reason
            self._reject_command(
                command,
                reason=reject_reason,
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._command_response(
                session_id, operator_id, command, accepted=False, reason=reject_reason
            )

        if self._is_get_pose_command(command):
            return self._execute_get_pose_query(
                command=command,
                session_id=session_id,
                operator_id=operator_id,
                runtime=runtime,
            )

        command.confirmation_expires_at = _utcnow() + self._confirmation_window
        self._transition_command(
            command,
            next_state=CommandLifecycleState.NEEDS_CONFIRMATION,
            reason="validated plan now requires operator confirmation",
            runtime_state=runtime.system_state,
            payload={
                "planFingerprint": command.plan_fingerprint,
                "reviewExpiresAt": command.confirmation_expires_at.isoformat(),
                "riskLevel": validation["riskLevel"],
                "confirmationReasons": validation["confirmationReasons"],
            },
            message_text=(
                "Step 3/6 NEEDS_CONFIRMATION: validation passed. "
                "Review the normalized plan and confirm before execution boundary handoff."
            ),
            message_tag=CommandLifecycleState.NEEDS_CONFIRMATION.value,
        )
        if (
            self._sim_auto_confirm
            and command.mode == RuntimeMode.SIM
            and command.plan_fingerprint is not None
        ):
            return self._confirm_command_internal(
                command=command,
                lease=lease,
                session_id=session_id,
                operator_id=operator_id,
                plan_fingerprint=command.plan_fingerprint,
            )
        return self._command_response(
            session_id, operator_id, command, accepted=True, reason=None
        )

    def _submit_sequence(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease: Any,
        runtime: RuntimeSnapshot,
        requested_mode: RuntimeMode,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        sequence_segments: list[str],
        prepared_sequence: dict[str, Any] | None = None,
        sequence_id: str | None = None,
        correlation_id: str | None = None,
    ) -> dict[str, Any]:
        sequence_id = sequence_id or str(uuid4())
        correlation_id = correlation_id or str(uuid4())
        sequence = CommandRecord(
            command_id=sequence_id,
            command_kind=CommandKind.SEQUENCE,
            session_id=session_id,
            operator_id=operator_id,
            raw_text=raw_text,
            lifecycle_state=CommandLifecycleState.RECEIVED,
            summary_label=(raw_text or "structured sequence")[:80],
            mode=requested_mode,
            created_at=_utcnow(),
            intent_source="text" if raw_text else "structured",
            correlation_id=correlation_id,
            structured_intent=structured_intent,
        )
        with self._lock:
            self._commands[sequence_id] = sequence
            self._active_sequence_id = sequence_id
            self._active_command_id = None
        self._append_message(
            origin="operator",
            text=raw_text or "Submitted structured sequence.",
            command_id=sequence_id,
        )
        self._audit.upsert_command(sequence)
        self._audit.record_transition(
            command_id=sequence_id,
            session_id=session_id,
            operator_id=operator_id,
            from_state=None,
            to_state=CommandLifecycleState.RECEIVED,
            runtime_state=runtime.system_state,
            reason="sequence received from HMI v2 intent endpoint",
            payload={
                "mode": sequence.mode.value,
                "intentSource": sequence.intent_source,
                "kind": sequence.command_kind.value,
            },
        )
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.PARSING,
            reason="supervisor parsing sequence intent",
            runtime_state=runtime.system_state,
            message_text="Step 1/6 PARSING: received multi-step sequence intent.",
            message_tag=CommandLifecycleState.PARSING.value,
        )
        if prepared_sequence is None:
            parsed_steps, diagnostics, parse_error, route_metadata = (
                self._parse_sequence_steps(
                    raw_text=raw_text,
                    structured_intent=structured_intent,
                    mode=sequence.mode,
                    sequence_segments=sequence_segments,
                )
            )
        else:
            parsed_steps = prepared_sequence.get("parsed_steps")
            diagnostics = list(prepared_sequence.get("diagnostics") or [])
            parse_error = prepared_sequence.get("parse_error")
            route_metadata = prepared_sequence.get("route_metadata")
            if prepared_sequence.get("structured_intent") is not None:
                sequence.structured_intent = prepared_sequence["structured_intent"]

        sequence.summary_label = self._sequence_summary_label(
            parsed_steps=parsed_steps or [],
            route_metadata=route_metadata,
            raw_text=raw_text,
            structured_intent=sequence.structured_intent,
        )
        sequence.plan_summary = self._sequence_plan_summary(
            raw_text=raw_text,
            structured_intent=sequence.structured_intent,
            parsed_steps=parsed_steps or [],
            diagnostics=diagnostics,
            route_metadata=route_metadata,
            requires_confirmation=parse_error is None,
        )
        if parsed_steps is None:
            sequence.reject_reason = parse_error
            sequence.validation_result = self._build_validation_result(
                runtime=runtime,
                lease=lease,
                parsed_intent=None,
                blocking_reasons=[parse_error or "sequence parsing failed"],
                risk_level=None,
                requires_confirmation=False,
            )
            self._reject_top_level_record(
                sequence,
                reason=parse_error or "sequence parsing failed",
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._sequence_response(
                session_id, operator_id, sequence, accepted=False, reason=parse_error
            )

        sequence.sequence_step_count = len(parsed_steps)
        sequence.current_step_index = 0
        sequence.sequence_diagnostics = list(diagnostics)
        child_commands: list[CommandRecord] = []
        for step_index, parsed_step in enumerate(parsed_steps):
            child_command = CommandRecord(
                command_id=str(uuid4()),
                session_id=session_id,
                operator_id=operator_id,
                raw_text=(
                    sequence_segments[step_index]
                    if step_index < len(sequence_segments)
                    else parsed_step["targetSummary"]
                ),
                lifecycle_state=CommandLifecycleState.RECEIVED,
                summary_label=parsed_step["targetSummary"],
                mode=sequence.mode,
                created_at=_utcnow(),
                intent_source=sequence.intent_source,
                correlation_id=f"{correlation_id}:{step_index + 1}",
                parsed_intent=parsed_step,
                planner_used=self._planner_for_intent(parsed_step),
                frame_used=self._frame_for_intent(parsed_step),
                parent_sequence_id=sequence_id,
                sequence_step_index=step_index,
                sequence_step_count=len(parsed_steps),
            )
            child_commands.append(child_command)
            sequence.child_command_ids.append(child_command.command_id)
            self._commands[child_command.command_id] = child_command
            self._audit.upsert_command(child_command)
            self._audit.record_transition(
                command_id=child_command.command_id,
                session_id=session_id,
                operator_id=operator_id,
                from_state=None,
                to_state=CommandLifecycleState.RECEIVED,
                runtime_state=runtime.system_state,
                reason="sequence step received from HMI v2 intent endpoint",
                payload={
                    "parentSequenceId": sequence_id,
                    "stepIndex": step_index,
                    "stepCount": len(parsed_steps),
                },
            )
            self._transition_command(
                child_command,
                next_state=CommandLifecycleState.PARSING,
                reason="supervisor parsing sequence step intent",
                runtime_state=runtime.system_state,
            )
            self._transition_command(
                child_command,
                next_state=CommandLifecycleState.VALIDATING,
                reason="supervisor validating parsed sequence step",
                runtime_state=runtime.system_state,
            )

        self._active_command_id = (
            child_commands[0].command_id if child_commands else None
        )
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.VALIDATING,
            reason="supervisor validating parsed sequence",
            runtime_state=runtime.system_state,
            payload={"stepCount": len(parsed_steps)},
            message_text=f"Step 2/6 VALIDATING: {len(parsed_steps)} ordered steps accepted for sequence validation.",
            message_tag=CommandLifecycleState.VALIDATING.value,
        )
        validation = self._validate_sequence(
            runtime=runtime,
            lease=lease,
            parsed_steps=parsed_steps,
            requested_mode=sequence.mode,
            diagnostics=diagnostics,
        )
        sequence.validation_result = validation
        sequence.plan_fingerprint = validation["planFingerprint"]
        if validation["riskLevel"] is not None:
            sequence.risk_level = CommandRiskLevel(validation["riskLevel"])
        for child_command in child_commands:
            child_validation = self._validate_command(
                runtime=runtime,
                lease=lease,
                parsed_intent=child_command.parsed_intent,
                requested_mode=child_command.mode,
            )
            child_command.validation_result = child_validation
            child_command.plan_fingerprint = child_validation["planFingerprint"]
            if child_validation["riskLevel"] is not None:
                child_command.risk_level = CommandRiskLevel(
                    child_validation["riskLevel"]
                )
            if not child_validation["accepted"]:
                validation["accepted"] = False
                validation["blockingReasons"] = list(
                    dict.fromkeys(
                        validation["blockingReasons"]
                        + child_validation["blockingReasons"]
                    )
                )
            self._audit.upsert_command(child_command)
        self._audit.upsert_command(sequence)
        if not validation["accepted"]:
            reject_reason = (
                "; ".join(validation["blockingReasons"]) or "sequence validation failed"
            )
            sequence.reject_reason = reject_reason
            self._reject_top_level_record(
                sequence,
                reason=reject_reason,
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._sequence_response(
                session_id, operator_id, sequence, accepted=False, reason=reject_reason
            )

        sequence.confirmation_expires_at = _utcnow() + self._confirmation_window
        for child_command in child_commands:
            child_command.confirmation_expires_at = sequence.confirmation_expires_at
            self._transition_command(
                child_command,
                next_state=CommandLifecycleState.NEEDS_CONFIRMATION,
                reason="sequence-level validation passed; waiting for parent confirmation",
                runtime_state=runtime.system_state,
            )
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.NEEDS_CONFIRMATION,
            reason="validated sequence now requires operator confirmation",
            runtime_state=runtime.system_state,
            payload={
                "planFingerprint": sequence.plan_fingerprint,
                "reviewExpiresAt": sequence.confirmation_expires_at.isoformat(),
                "riskLevel": validation["riskLevel"],
                "confirmationReasons": validation["confirmationReasons"],
            },
            message_text=(
                "Step 3/6 NEEDS_CONFIRMATION: sequence validation passed. "
                "Review the ordered steps and confirm once before execution."
            ),
            message_tag=CommandLifecycleState.NEEDS_CONFIRMATION.value,
        )
        if (
            self._sim_auto_confirm
            and sequence.mode == RuntimeMode.SIM
            and sequence.plan_fingerprint is not None
        ):
            return self.confirm_sequence(
                session_id=session_id,
                operator_id=operator_id,
                lease_token=lease.lease_token,
                sequence_id=sequence.command_id,
                plan_fingerprint=sequence.plan_fingerprint,
            )
        return self._sequence_response(
            session_id, operator_id, sequence, accepted=True, reason=None
        )
