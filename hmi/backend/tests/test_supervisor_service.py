from __future__ import annotations

import unittest
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory

from hmi.backend.domain.models import (
    BridgeConnection,
    ConnectionHealth,
    HardwareGateChecklistSnapshot,
    HardwareGateStatusSnapshot,
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


class AlwaysUnlockedHardwareGate:
    def evaluate(self) -> HardwareGateStatusSnapshot:
        return HardwareGateStatusSnapshot(
            unlocked=True,
            reasons=[],
            flag_enabled=True,
            evidence_path="hmi/data/hardware_gate.json",
            approved_by="qa.engineer",
            approved_at="2026-04-18T12:00:00Z",
            report_path="hmi/HARDWARE_TELEMETRY_VALIDATION.md",
            report_sha256="f" * 64,
            report_sha256_match=True,
            checklist=HardwareGateChecklistSnapshot(
                timing_jitter=True,
                disconnect_reconnect=True,
                robot_status_semantics=True,
                joint_source_precedence=True,
                audit_visibility=True,
            ),
        )


class FakeSupervisorAdapter:
    def __init__(self) -> None:
        self.started = False
        self.stopped = False
        self.submit_calls: list[dict] = []
        self.confirm_calls: list[dict] = []
        self.abort_calls: list[dict] = []
        self._runtime = RuntimeSnapshot(
            system_state=SystemRuntimeState.NORMAL,
            blocking=False,
            status_text='Sim telemetry fresh and runtime clear.',
            mode=RuntimeMode.SIM,
            robot_status=RobotStatusSnapshot(readiness_message='Sim ready'),
        )
        self._connections = [
            BridgeConnection(name='ros2', label='ROS 2', health=ConnectionHealth.HEALTHY),
            BridgeConnection(name='moveit2', label='MoveIt 2', health=ConnectionHealth.HEALTHY),
            BridgeConnection(name='llm', label='LLM', health=ConnectionHealth.HEALTHY),
            BridgeConnection(name='motoros2', label='MotoROS2', health=ConnectionHealth.DOWN),
        ]
        self._joints = [
            JointPosition(name='joint_1_s', position_deg=0.0),
            JointPosition(name='joint_2_l', position_deg=5.0),
            JointPosition(name='joint_3_u', position_deg=10.0),
            JointPosition(name='joint_4_r', position_deg=15.0),
            JointPosition(name='joint_5_b', position_deg=20.0),
            JointPosition(name='joint_6_t', position_deg=25.0),
        ]
        self._source_statuses = [
            TelemetrySourceSnapshot(
                name='gateway_status',
                label='Gateway status',
                topic='/gateway_status',
                last_seen_at=None,
                freshness_threshold_sec=30.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                active=True,
            ),
            TelemetrySourceSnapshot(
                name='readiness',
                label='HW readiness',
                topic='/hw_adapter/ready',
                last_seen_at=None,
                freshness_threshold_sec=3.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                active=True,
            ),
            TelemetrySourceSnapshot(
                name='supervisor_alerts',
                label='Supervisor alerts',
                topic='/supervisor/alerts',
                last_seen_at=None,
                freshness_threshold_sec=5.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                active=True,
            ),
            TelemetrySourceSnapshot(
                name='joint_states_fallback',
                label='Joint states fallback',
                topic='/joint_states',
                last_seen_at=None,
                freshness_threshold_sec=3.0,
                freshness_state=TelemetryFreshnessState.FRESH,
                preferred=True,
                active=True,
            ),
            TelemetrySourceSnapshot(
                name='robot_status',
                label='Robot status',
                topic='/yaskawa/robot_status',
                last_seen_at=None,
                freshness_threshold_sec=3.0,
                freshness_state=TelemetryFreshnessState.STALE,
                active=False,
                detail='SIM mode uses /hw_adapter/ready instead.',
            ),
            TelemetrySourceSnapshot(
                name='llm_debug',
                label='LLM debug',
                topic='/llm_debug',
                last_seen_at=None,
                freshness_threshold_sec=30.0,
                freshness_state=TelemetryFreshnessState.STALE,
                active=False,
            ),
            TelemetrySourceSnapshot(
                name='llm_command',
                label='LLM command echo',
                topic='/llm_command',
                last_seen_at=None,
                freshness_threshold_sec=30.0,
                freshness_state=TelemetryFreshnessState.STALE,
                active=False,
            ),
        ]
        self._confirm_result = {
            'accepted': True,
            'adapter': 'workspace_ros_adapter',
            'status': 'succeeded',
            'summary': 'Sim execution completed successfully.',
            'dispatchedToRos': True,
        }
        self._preflight_result = {
            'accepted': True,
            'mode': 'sim',
            'reasons': [],
            'requiredSources': [],
            'sourceStatuses': [],
            'runtimeState': 'NORMAL',
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
        return {'accepted': True, 'adapter': 'fake-review'}

    def confirm_command(self, **kwargs):
        self.confirm_calls.append(kwargs)
        response = dict(self._confirm_result)
        response.update({
            'commandId': kwargs['command_id'],
            'planFingerprint': kwargs['plan_fingerprint'],
            'correlationId': kwargs['correlation_id'],
        })
        return response

    def evaluate_execution_preflight(self, *, target_mode: str | None = None):
        result = dict(self._preflight_result)
        if target_mode is not None:
            result['mode'] = target_mode
        return result

    def abort_command(self, **kwargs):
        self.abort_calls.append(kwargs)
        return True, 'cancelled before ROS dispatch'

    def set_runtime(self, system_state: SystemRuntimeState, *, mode: RuntimeMode = RuntimeMode.SIM) -> None:
        self._runtime = RuntimeSnapshot(
            system_state=system_state,
            blocking=system_state in {
                SystemRuntimeState.FAULT,
                SystemRuntimeState.ESTOP,
                SystemRuntimeState.LOST_CONN,
                SystemRuntimeState.SAFETY_BLOCKED,
            },
            status_text=f'Runtime set to {system_state.value}.',
            mode=mode,
            robot_status=RobotStatusSnapshot(readiness_message='fixture'),
        )

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
                        TelemetryFreshnessState.STALE if source.name in stale_names else source.freshness_state
                    ),
                    preferred=source.preferred,
                    active=source.active,
                    detail=source.detail,
                )
            )
        self._source_statuses = next_statuses

    def set_confirm_result(self, **kwargs) -> None:
        self._confirm_result.update(kwargs)

    def set_preflight(self, *, accepted: bool, reasons: list[str]) -> None:
        self._preflight_result = {
            'accepted': accepted,
            'mode': self._runtime.mode.value,
            'reasons': list(reasons),
            'requiredSources': [],
            'sourceStatuses': [],
            'runtimeState': self._runtime.system_state.value,
        }


class SupervisorServiceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.audit = AuditService(Path(self.temp_dir.name) / 'audit.sqlite3')
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
        self.session_id = 'session-a'
        self.operator_id = 'operator-a'

    def _acquire_lease(self) -> str:
        lease = self.session_lock.acquire_controller(self.session_id, self.operator_id)
        return lease.lease_token

    def test_command_rejected_without_valid_control_lease(self) -> None:
        with self.assertRaises(ForbiddenActionError):
            self.supervisor.submit_intent(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=None,
                raw_text='home',
                mode='sim',
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
                    raw_text='home',
                    mode='sim',
                )
                self.assertFalse(response['accepted'])
                self.assertEqual(response['command']['lifecycleState'], 'REJECTED')
                self.assertIn(runtime_state.value, response['reason'])
                self.assertEqual(self.adapter.confirm_calls, [])
                self.adapter.set_runtime(SystemRuntimeState.NORMAL)

    def test_ambiguous_command_does_not_execute(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='draw a dragon on the table',
            mode='sim',
        )
        self.assertFalse(response['accepted'])
        self.assertEqual(response['command']['lifecycleState'], 'REJECTED')
        self.assertEqual(self.adapter.confirm_calls, [])

    def test_cartesian_text_intent_uses_base_link_for_sim_move_rel(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='move up 10 cm',
            mode='sim',
        )
        self.assertTrue(response['accepted'])
        self.assertEqual(response['command']['parsedIntent']['action'], 'MOVE_REL')
        self.assertEqual(response['command']['parsedIntent']['normalizedCommand']['reference_frame'], 'base_link')

    def test_joint_text_intent_maps_to_absolute_joint_target(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='move joint 2 5 deg',
            mode='sim',
        )
        self.assertTrue(response['accepted'])
        normalized_command = response['command']['parsedIntent']['normalizedCommand']
        self.assertEqual(normalized_command['primitive_type'], 'MOVE_JOINT')
        self.assertEqual(normalized_command['joint_index'], 1)
        self.assertAlmostEqual(normalized_command['joint_angle'], 5.0 * 3.141592653589793 / 180.0)

    def test_confirmation_required_command_stops_before_execution_boundary(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        self.assertTrue(response['accepted'])
        self.assertEqual(response['command']['lifecycleState'], 'NEEDS_CONFIRMATION')
        self.assertEqual(self.adapter.confirm_calls, [])

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
            raw_text='home',
            mode='sim',
        )
        self.assertTrue(response['accepted'])
        self.assertEqual(response['command']['lifecycleState'], 'SUCCEEDED')
        self.assertEqual(response['command']['finalState'], 'SUCCEEDED')
        self.assertEqual(len(self.adapter.confirm_calls), 1)

    def test_stale_critical_telemetry_rejects_execution_path(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_source_freshness(stale_names={'joint_states_fallback'})
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        self.assertFalse(response['accepted'])
        self.assertEqual(response['command']['lifecycleState'], 'REJECTED')
        self.assertIn('joint_states_fallback', response['reason'])

    def test_hardware_mode_requires_dual_gate_before_command_ingress(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='hardware',
        )
        self.assertFalse(response['accepted'])
        self.assertEqual(response['command']['lifecycleState'], 'REJECTED')
        self.assertIn('HMI_ENABLE_HARDWARE_COMMANDS', response['reason'])

    def test_hardware_mode_allows_command_when_gate_and_preflight_pass(self) -> None:
        supervisor = SupervisorService(
            audit_service=self.audit,
            session_lock_service=self.session_lock,
            ros_adapter=self.adapter,
            confirmation_window_sec=5.0,
            hardware_gate_evaluator=AlwaysUnlockedHardwareGate(),
        )
        supervisor.bind_telemetry_service(self.telemetry)
        lease_token = self._acquire_lease()
        self.adapter.set_runtime(SystemRuntimeState.NORMAL, mode=RuntimeMode.HARDWARE)
        self.adapter.set_preflight(accepted=True, reasons=[])
        response = supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='hardware',
        )
        self.assertTrue(response['accepted'])
        self.assertEqual(response['command']['lifecycleState'], 'NEEDS_CONFIRMATION')
        self.assertEqual(response['command']['mode'], 'hardware')

    def test_missing_structured_fields_fail_closed_with_operator_visible_reason(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='',
            structured_intent={
                'primitive_type': 'MOVE_JOINT',
                'joint_index': 1,
            },
            mode='sim',
        )
        self.assertFalse(response['accepted'])
        self.assertEqual(response['command']['lifecycleState'], 'REJECTED')
        self.assertIn('joint_angle', response['reason'])

    def test_preflight_failures_reject_command_with_explicit_reason(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_preflight(
            accepted=False,
            reasons=['required telemetry source joint_states_fallback is stale.'],
        )
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        self.assertFalse(response['accepted'])
        self.assertIn('joint_states_fallback', response['reason'])

    def test_confirmation_expires_correctly(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        command_id = response['commandId']
        command = self.supervisor._commands[command_id]
        command.confirmation_expires_at = command.created_at
        expired = self.supervisor.get_command(command_id)
        self.assertEqual(expired['lifecycleState'], 'EXPIRED')

    def test_mismatched_plan_fingerprint_and_session_fail_closed(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        command_id = response['commandId']
        with self.assertRaises(ConflictError):
            self.supervisor.confirm_command(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                command_id=command_id,
                plan_fingerprint='wrong-fingerprint',
            )
        other_lease = self.session_lock.acquire_controller('session-b', 'operator-b', force_takeover=True, takeover_reason='test')
        with self.assertRaises(ForbiddenActionError):
            self.supervisor.cancel_command(
                session_id='session-b',
                operator_id='operator-b',
                lease_token=other_lease.lease_token,
                command_id=command_id,
                reason='forbidden',
            )

    def test_confirmed_command_reaches_execution_boundary_only_after_checks(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='move up 10 cm',
            mode='sim',
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            plan_fingerprint=response['command']['planFingerprint'],
        )
        self.assertTrue(confirm_response['accepted'])
        self.assertEqual(len(self.adapter.confirm_calls), 1)
        self.assertEqual(confirm_response['command']['lifecycleState'], 'SUCCEEDED')
        self.assertEqual(confirm_response['command']['finalState'], 'SUCCEEDED')

    def test_rejected_command_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='draw a dragon on the table',
            mode='sim',
        )
        self.assertFalse(response['accepted'])
        self.assertEqual(response['command']['lifecycleState'], 'REJECTED')
        self.assertEqual(response['command']['finalState'], 'REJECTED')
        self.assertIsNotNone(response['command']['rejectReason'])

    def test_expired_command_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        command_id = response['commandId']
        command = self.supervisor._commands[command_id]
        command.confirmation_expires_at = command.created_at
        expired = self.supervisor.get_command(command_id)
        self.assertEqual(expired['lifecycleState'], 'EXPIRED')
        self.assertEqual(expired['finalState'], 'EXPIRED')
        self.assertEqual(expired['rejectReason'], 'confirmation window expired')

    def test_cancelled_command_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        cancel_response = self.supervisor.cancel_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            reason='operator cancelled review',
        )
        self.assertTrue(cancel_response['accepted'])
        self.assertEqual(cancel_response['command']['lifecycleState'], 'CANCELLED')
        self.assertEqual(cancel_response['command']['finalState'], 'CANCELLED')
        self.assertEqual(cancel_response['command']['rejectReason'], 'operator cancelled review')

    def test_failed_command_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_confirm_result(
            accepted=False,
            status='failed',
            summary='ExecuteMotion failed inside fake adapter.',
            dispatchedToRos=True,
        )
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='move up 10 cm',
            mode='sim',
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            plan_fingerprint=response['command']['planFingerprint'],
        )
        self.assertFalse(confirm_response['accepted'])
        self.assertEqual(confirm_response['command']['lifecycleState'], 'FAILED')
        self.assertEqual(confirm_response['command']['finalState'], 'FAILED')
        self.assertEqual(confirm_response['command']['rejectReason'], 'ExecuteMotion failed inside fake adapter.')

    def test_cancelled_execution_event_carries_terminal_fields(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_confirm_result(
            accepted=False,
            status='cancelled',
            summary='Execution was cancelled by operator.',
            dispatchedToRos=True,
        )
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='move up 10 cm',
            mode='sim',
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            plan_fingerprint=response['command']['planFingerprint'],
        )
        self.assertFalse(confirm_response['accepted'])
        self.assertEqual(confirm_response['command']['lifecycleState'], 'CANCELLED')
        self.assertEqual(confirm_response['command']['finalState'], 'CANCELLED')
        self.assertEqual(confirm_response['command']['rejectReason'], 'Execution was cancelled by operator.')

    def test_succeeded_terminal_event_carries_final_state(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            plan_fingerprint=response['command']['planFingerprint'],
        )
        self.assertTrue(confirm_response['accepted'])
        self.assertEqual(confirm_response['command']['lifecycleState'], 'SUCCEEDED')
        self.assertEqual(confirm_response['command']['finalState'], 'SUCCEEDED')
        self.assertIsNone(confirm_response['command']['rejectReason'])

    def test_nonaccepted_confirmation_gate_does_not_expose_false_terminal_state(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        self.adapter.set_source_freshness(stale_names={'joint_states_fallback'})
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            plan_fingerprint=response['command']['planFingerprint'],
        )
        self.assertFalse(confirm_response['accepted'])
        self.assertEqual(confirm_response['command']['lifecycleState'], 'NEEDS_CONFIRMATION')
        self.assertIsNone(confirm_response['command']['finalState'])

    def test_execution_adapter_failure_marks_command_failed(self) -> None:
        lease_token = self._acquire_lease()
        self.adapter.set_confirm_result(
            accepted=False,
            status='failed',
            summary='ExecuteMotion failed inside fake adapter.',
            dispatchedToRos=True,
        )
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='move up 10 cm',
            mode='sim',
        )
        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            plan_fingerprint=response['command']['planFingerprint'],
        )
        self.assertFalse(confirm_response['accepted'])
        self.assertEqual(confirm_response['command']['lifecycleState'], 'FAILED')

    def test_cancel_works_for_pending_commands(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        cancel_response = self.supervisor.cancel_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            reason='operator cancelled review',
        )
        self.assertTrue(cancel_response['accepted'])
        self.assertEqual(cancel_response['command']['lifecycleState'], 'CANCELLED')
        self.assertEqual(self.adapter.abort_calls, [])

    def test_audit_trail_records_major_transitions(self) -> None:
        lease_token = self._acquire_lease()
        response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=response['commandId'],
            plan_fingerprint=response['command']['planFingerprint'],
        )
        detail = self.audit.get_command_detail(response['commandId'])
        self.assertIsNotNone(detail)
        transitions = [row['to_state'] for row in detail['timeline']]
        self.assertEqual(
            transitions,
            [
                'RECEIVED',
                'PARSING',
                'VALIDATING',
                'NEEDS_CONFIRMATION',
                'CONFIRMED',
                'EXECUTION_REQUESTED',
                'EXECUTING',
                'SUCCEEDED',
            ],
        )
        runtime_messages = [row['message'] for row in detail['runtime_events']]
        self.assertIn('validation result recorded', runtime_messages)
        self.assertIn('execution boundary response recorded', runtime_messages)

    def test_step_messages_exist_for_parse_validate_confirm_and_result(self) -> None:
        lease_token = self._acquire_lease()
        submit_response = self.supervisor.submit_intent(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            raw_text='home',
            mode='sim',
        )
        self.assertTrue(submit_response['accepted'])
        submit_messages = [msg['text'] for msg in submit_response['snapshot']['messages']]
        self.assertTrue(any('Step 1/6 PARSING' in text for text in submit_messages))
        self.assertTrue(any('Step 2/6 VALIDATING' in text for text in submit_messages))
        self.assertTrue(any('Step 3/6 NEEDS_CONFIRMATION' in text for text in submit_messages))

        confirm_response = self.supervisor.confirm_command(
            session_id=self.session_id,
            operator_id=self.operator_id,
            lease_token=lease_token,
            command_id=submit_response['commandId'],
            plan_fingerprint=submit_response['command']['planFingerprint'],
        )
        confirm_messages = [msg['text'] for msg in confirm_response['snapshot']['messages']]
        self.assertTrue(any('Step 4/6 CONFIRMED' in text for text in confirm_messages))
        self.assertTrue(any('Step 5/6 EXECUTION_REQUESTED' in text for text in confirm_messages))
        self.assertTrue(any('Step 6/6 RESULT' in text for text in confirm_messages))

    def test_terminal_command_trace_logs_are_human_readable(self) -> None:
        lease_token = self._acquire_lease()
        with self.assertLogs('uvicorn.error', level='INFO') as captured:
            response = self.supervisor.submit_intent(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                raw_text='home',
                mode='sim',
            )
            self.supervisor.confirm_command(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                command_id=response['commandId'],
                plan_fingerprint=response['command']['planFingerprint'],
            )

        output = '\n'.join(captured.output)
        self.assertIn('[HMI CMD] request.received', output)
        self.assertIn('[HMI CMD] parse.accepted', output)
        self.assertIn('[HMI CMD] validation.accepted', output)
        self.assertIn('[HMI CMD] confirmation.accepted', output)
        self.assertIn('[HMI CMD] execution.requested', output)
        self.assertIn('[HMI CMD] terminal.succeeded', output)

    def test_rejection_trace_logs_include_reason(self) -> None:
        lease_token = self._acquire_lease()
        with self.assertLogs('uvicorn.error', level='INFO') as captured:
            self.supervisor.submit_intent(
                session_id=self.session_id,
                operator_id=self.operator_id,
                lease_token=lease_token,
                raw_text='draw a dragon on the table',
                mode='sim',
            )

        output = '\n'.join(captured.output)
        self.assertIn('[HMI CMD] parse.rejected', output)
        self.assertIn('[HMI CMD] terminal.rejected', output)
        self.assertIn('reason=intent is ambiguous or unsupported', output)

if __name__ == '__main__':
    unittest.main()
