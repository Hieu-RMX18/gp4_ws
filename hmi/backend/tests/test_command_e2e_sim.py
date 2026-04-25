from __future__ import annotations

import asyncio
import json
import os
import signal
import socket
import subprocess
import time
import unittest
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

import websockets


WORKSPACE_DIR = Path(__file__).resolve().parents[3]
INSTALL_SETUP = WORKSPACE_DIR / 'install' / 'setup.bash'
APP_MODULE = 'hmi.backend.api.app:app'
SUPPORTED_SCHEMA_VERSION = 'telemetry.v1'


class CommandE2ESimTests(unittest.IsolatedAsyncioTestCase):
    _domain_counter = 0

    def setUp(self) -> None:
        self._temp_dir = TemporaryDirectory()
        self.addCleanup(self._temp_dir.cleanup)
        self._log_root = Path(self._temp_dir.name)
        self._domain_id = self._reserve_ros_domain_id()
        try:
            self._port = self._reserve_tcp_port()
        except PermissionError as exc:
            raise unittest.SkipTest(
                'local TCP sockets are not permitted in this execution environment'
            ) from exc
        self._audit_dir = self._log_root / 'audit'
        self._audit_dir.mkdir(parents=True, exist_ok=True)
        self._environment = os.environ.copy()
        self._environment['PYTHONPATH'] = self._build_pythonpath()
        self._environment['ROS_DOMAIN_ID'] = str(self._domain_id)
        self._environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        self._environment['HMI_SIM_AUTO_CONFIRM'] = 'false'
        self._environment.setdefault('ROS_LOG_DIR', str(self._log_root / 'ros_logs'))
        self._sim_process: subprocess.Popen[str] | None = None
        self._api_process: subprocess.Popen[str] | None = None
        self._sim_log_handle = None
        self._api_log_handle = None
        self._api_restart_count = 0

    async def asyncTearDown(self) -> None:
        self._stop_process(self._api_process)
        self._stop_process(self._sim_process)
        if self._api_log_handle is not None:
            self._api_log_handle.close()
            self._api_log_handle = None
        if self._sim_log_handle is not None:
            self._sim_log_handle.close()
            self._sim_log_handle = None

    async def test_home_command_executes_to_success_in_sim(self) -> None:
        self._start_sim_stack()
        self._start_api_server()

        snapshot = await self._wait_until_sim_ready()
        self.assertEqual(snapshot['schemaVersion'], SUPPORTED_SCHEMA_VERSION)
        self.assertEqual(snapshot['mode'], 'sim')
        self.assertTrue(snapshot['capabilities']['canAcquireLease'])

        result = await self._run_command_case(
            session_id='e2e-session-home',
            operator_id='e2e-operator-home',
            intent_text='home',
            expected_action='HOME',
        )

        self.assertEqual(result['command']['executionResult']['status'], 'succeeded')
        self.assertEqual(result['command']['finalState'], 'SUCCEEDED')

    async def test_supported_hmi_commands_execute_in_sim(self) -> None:
        self._start_sim_stack()
        self._start_api_server()
        await self._wait_until_sim_ready()

        cases = [
            {'intent_text': 'home', 'expected_action': 'HOME'},
            {'intent_text': 'move down 1 cm', 'expected_action': 'MOVE_REL'},
            {'intent_text': 'move joint 2 5 deg', 'expected_action': 'MOVE_JOINT'},
            {'intent_text': 'stop', 'expected_action': 'STOP'},
        ]
        for index, case in enumerate(cases, start=1):
            with self.subTest(intent=case['intent_text']):
                result = await self._run_command_case(
                    session_id=f'e2e-session-{index}',
                    operator_id=f'e2e-operator-{index}',
                    intent_text=case['intent_text'],
                    expected_action=case['expected_action'],
                )
                self.assertEqual(result['command']['executionResult']['status'], 'succeeded')
                self.assertEqual(result['command']['lifecycleState'], 'SUCCEEDED')
                self.assertEqual(result['command']['finalState'], 'SUCCEEDED')

    async def test_text_sequence_executes_to_success_in_sim(self) -> None:
        self._start_sim_stack()
        self._start_api_server()
        await self._wait_until_sim_ready()

        result = await self._run_sequence_case(
            session_id='e2e-session-sequence',
            operator_id='e2e-operator-sequence',
            intent_text='home, wait 1 s, then move down 1 cm',
            expected_actions=['HOME', 'WAIT', 'MOVE_REL'],
        )

        self.assertEqual(result['sequence']['executionResult']['status'], 'succeeded')
        self.assertEqual(result['sequence']['lifecycleState'], 'SUCCEEDED')
        self.assertEqual(result['sequence']['finalState'], 'SUCCEEDED')
        self.assertEqual(
            [step['finalState'] for step in result['sequence']['steps']],
            ['SUCCEEDED', 'SUCCEEDED', 'SUCCEEDED'],
        )

    async def test_structured_draw_shape_executes_to_success_in_sim(self) -> None:
        self._start_sim_stack()
        self._start_api_server()
        await self._wait_until_sim_ready()
        await self._prime_draw_pose('draw-shape')

        result = await self._run_sequence_case(
            session_id='e2e-session-draw-shape',
            operator_id='e2e-operator-draw-shape',
            structured_intent={
                'intent': 'draw_shape',
                'shape_type': 'circle',
                'units': 'mm',
                'frame_id': 'base_link',
                'params': {'radius': 10},
            },
            expected_macro_name='draw_shape',
        )

        self.assertEqual(result['sequence']['finalState'], 'SUCCEEDED')
        self.assertEqual(result['sequence']['planSummary']['shapeType'], 'circle')

    async def test_structured_draw_text_executes_to_success_in_sim(self) -> None:
        self._start_sim_stack()
        self._start_api_server()
        await self._wait_until_sim_ready()
        await self._prime_draw_pose('draw-text-structured')

        result = await self._run_sequence_case(
            session_id='e2e-session-draw-text-structured',
            operator_id='e2e-operator-draw-text-structured',
            structured_intent={
                'intent': 'draw_text',
                'text': 'GP4',
                'units': 'mm',
                'frame_id': 'base_link',
                'font': {'type': 'single_stroke_builtin', 'height': 10},
            },
            expected_macro_name='draw_text',
        )

        self.assertEqual(result['sequence']['finalState'], 'SUCCEEDED')
        self.assertEqual(result['sequence']['planSummary']['text'], 'GP4')

    async def test_text_draw_text_executes_to_success_in_sim(self) -> None:
        self._start_sim_stack()
        self._start_api_server()
        await self._wait_until_sim_ready()
        await self._prime_draw_pose('draw-text-free-text')

        result = await self._run_sequence_case(
            session_id='e2e-session-draw-text-free-text',
            operator_id='e2e-operator-draw-text-free-text',
            intent_text='write GP4',
            expected_macro_name='draw_text',
        )

        self.assertEqual(result['sequence']['finalState'], 'SUCCEEDED')
        self.assertEqual(result['sequence']['planSummary']['text'], 'GP4')

    async def _wait_until_sim_ready(self) -> dict[str, Any]:
        await asyncio.to_thread(
            self._wait_for_snapshot,
            expected_transport='connected',
            expected_telemetry='fresh',
            expected_runtime='NORMAL',
        )
        await asyncio.sleep(1.0)
        return await asyncio.to_thread(
            self._wait_for_snapshot,
            expected_transport='connected',
            expected_telemetry='fresh',
            expected_runtime='NORMAL',
        )

    async def _prime_draw_pose(self, suffix: str) -> None:
        result = await self._run_command_case(
            session_id=f'e2e-session-prime-{suffix}',
            operator_id=f'e2e-operator-prime-{suffix}',
            intent_text='move down 5 cm',
            expected_action='MOVE_REL',
        )
        self.assertEqual(result['command']['finalState'], 'SUCCEEDED')

    async def _run_command_case(
        self,
        *,
        session_id: str,
        operator_id: str,
        intent_text: str,
        expected_action: str,
    ) -> dict[str, Any]:
        lease_response = await asyncio.to_thread(
            self._post_json,
            '/api/hmi/lease/acquire',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'requestedRole': 'controller',
            },
        )
        self.assertTrue(lease_response['accepted'])
        lease_token = lease_response['lease']['leaseToken']
        self.assertIsNotNone(lease_token)

        command_response = await asyncio.to_thread(
            self._post_json,
            '/api/hmi/commands/intent',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'leaseToken': lease_token,
                'intentText': intent_text,
                'mode': 'sim',
            },
        )
        self.assertTrue(command_response['accepted'])
        self.assertEqual(command_response['command']['lifecycleState'], 'NEEDS_CONFIRMATION')
        self.assertEqual(command_response['command']['parsedIntent']['action'], expected_action)

        command_id = command_response['commandId']
        plan_fingerprint = command_response['command']['planFingerprint']
        self.assertIsInstance(plan_fingerprint, str)
        self.assertTrue(plan_fingerprint)

        stream_ready = asyncio.Event()
        stream_task = asyncio.create_task(
            self._collect_stream_events(
                session_id=session_id,
                operator_id=operator_id,
                stop_command_id=command_id,
                stop_states={'SUCCEEDED', 'FAILED', 'CANCELLED'},
                ready_event=stream_ready,
            )
        )
        await stream_ready.wait()

        confirm_response = await asyncio.to_thread(
            self._post_json,
            f'/api/hmi/commands/{urllib.parse.quote(command_id)}/confirm',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'leaseToken': lease_token,
                'planFingerprint': plan_fingerprint,
            },
            timeout_sec=60.0,
        )
        self.assertTrue(confirm_response['accepted'], msg=json.dumps(confirm_response, indent=2))
        self.assertEqual(
            confirm_response['command']['lifecycleState'],
            'SUCCEEDED',
            msg=json.dumps(confirm_response, indent=2),
        )
        self.assertEqual(
            confirm_response['command']['finalState'],
            'SUCCEEDED',
            msg=json.dumps(confirm_response, indent=2),
        )

        terminal_event = await stream_task
        self.assertEqual(terminal_event['type'], 'command_lifecycle')
        self.assertEqual(terminal_event['command']['commandId'], command_id)
        self.assertEqual(terminal_event['command']['lifecycleState'], 'SUCCEEDED')
        self.assertEqual(terminal_event['command']['finalState'], 'SUCCEEDED')

        replay_payload = await asyncio.to_thread(self._get_json, '/api/hmi/replay?limit=25')
        replay_ids = [item['commandId'] for item in replay_payload['items']]
        self.assertIn(command_id, replay_ids)

        command_payload = await asyncio.to_thread(
            self._get_json,
            f'/api/hmi/commands/{urllib.parse.quote(command_id)}',
        )
        self.assertEqual(command_payload['lifecycleState'], 'SUCCEEDED')
        self.assertEqual(command_payload['finalState'], 'SUCCEEDED')

        release_response = await asyncio.to_thread(
            self._post_json,
            '/api/hmi/lease/release',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'leaseToken': lease_token,
            },
        )
        self.assertTrue(release_response['accepted'])

        return {
            'commandId': command_id,
            'command': command_payload,
            'confirmResponse': confirm_response,
            'terminalEvent': terminal_event,
        }

    async def _run_sequence_case(
        self,
        *,
        session_id: str,
        operator_id: str,
        intent_text: str | None = None,
        structured_intent: dict[str, Any] | None = None,
        expected_actions: list[str] | None = None,
        expected_macro_name: str | None = None,
    ) -> dict[str, Any]:
        lease_response = await asyncio.to_thread(
            self._post_json,
            '/api/hmi/lease/acquire',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'requestedRole': 'controller',
            },
        )
        self.assertTrue(lease_response['accepted'])
        lease_token = lease_response['lease']['leaseToken']
        self.assertIsNotNone(lease_token)

        sequence_response = await asyncio.to_thread(
            self._post_json,
            '/api/hmi/commands/intent',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'leaseToken': lease_token,
                'mode': 'sim',
                **({'intentText': intent_text} if intent_text is not None else {}),
                **({'structuredIntent': structured_intent} if structured_intent is not None else {}),
            },
        )
        self.assertTrue(sequence_response['accepted'], msg=json.dumps(sequence_response, indent=2))
        self.assertEqual(sequence_response['jobType'], 'sequence')
        self.assertEqual(sequence_response['sequence']['lifecycleState'], 'NEEDS_CONFIRMATION')
        if expected_actions is not None:
            self.assertEqual(
                [step['parsedIntent']['action'] for step in sequence_response['sequence']['steps']],
                expected_actions,
            )
        else:
            self.assertGreater(sequence_response['sequence']['stepCount'], 1)
        if expected_macro_name is not None:
            self.assertEqual(sequence_response['sequence']['planSummary']['macroName'], expected_macro_name)

        sequence_id = sequence_response['sequenceId']
        plan_fingerprint = sequence_response['sequence']['planFingerprint']
        self.assertIsInstance(plan_fingerprint, str)
        self.assertTrue(plan_fingerprint)

        stream_ready = asyncio.Event()
        stream_task = asyncio.create_task(
            self._collect_terminal_stream_event(
                session_id=session_id,
                operator_id=operator_id,
                event_type='sequence_lifecycle',
                payload_key='sequence',
                entity_id_field='sequenceId',
                stop_entity_id=sequence_id,
                stop_states={'SUCCEEDED', 'FAILED', 'CANCELLED'},
                ready_event=stream_ready,
            )
        )
        await stream_ready.wait()

        reacquire_response = await asyncio.to_thread(
            self._post_json,
            '/api/hmi/lease/acquire',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'requestedRole': 'controller',
            },
        )
        self.assertTrue(reacquire_response['accepted'])
        lease_token = reacquire_response['lease']['leaseToken']

        confirm_response = await asyncio.to_thread(
            self._post_json,
            f'/api/hmi/sequences/{urllib.parse.quote(sequence_id)}/confirm',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'leaseToken': lease_token,
                'planFingerprint': plan_fingerprint,
            },
            timeout_sec=180.0,
        )
        self.assertTrue(confirm_response['accepted'], msg=json.dumps(confirm_response, indent=2))
        self.assertEqual(confirm_response['jobType'], 'sequence')
        self.assertEqual(
            confirm_response['sequence']['lifecycleState'],
            'SUCCEEDED',
            msg=json.dumps(confirm_response, indent=2),
        )
        self.assertEqual(
            confirm_response['sequence']['finalState'],
            'SUCCEEDED',
            msg=json.dumps(confirm_response, indent=2),
        )

        terminal_event = await stream_task
        self.assertEqual(terminal_event['type'], 'sequence_lifecycle')
        self.assertEqual(terminal_event['sequence']['sequenceId'], sequence_id)
        self.assertEqual(terminal_event['sequence']['lifecycleState'], 'SUCCEEDED')
        self.assertEqual(terminal_event['sequence']['finalState'], 'SUCCEEDED')

        replay_payload = await asyncio.to_thread(self._get_json, '/api/hmi/replay?limit=25')
        sequence_item = next(
            (item for item in replay_payload['items'] if item['commandId'] == sequence_id),
            None,
        )
        self.assertIsNotNone(sequence_item)
        self.assertEqual(sequence_item['kind'], 'sequence')

        sequence_payload = await asyncio.to_thread(
            self._get_json,
            f'/api/hmi/sequences/{urllib.parse.quote(sequence_id)}',
        )
        self.assertEqual(sequence_payload['lifecycleState'], 'SUCCEEDED')
        self.assertEqual(sequence_payload['finalState'], 'SUCCEEDED')
        if expected_actions is not None:
            self.assertEqual(sequence_payload['stepCount'], len(expected_actions))
            self.assertEqual(
                [step['parsedIntent']['action'] for step in sequence_payload['steps']],
                expected_actions,
            )
        else:
            self.assertGreater(sequence_payload['stepCount'], 1)
        if expected_macro_name is not None:
            self.assertEqual(sequence_payload['planSummary']['macroName'], expected_macro_name)
        self.assertTrue(sequence_payload['steps'])
        self.assertTrue(all(step['finalState'] == 'SUCCEEDED' for step in sequence_payload['steps']))

        release_response = await asyncio.to_thread(
            self._post_json,
            '/api/hmi/lease/release',
            {
                'sessionId': session_id,
                'operatorId': operator_id,
                'leaseToken': lease_token,
            },
        )
        self.assertTrue(release_response['accepted'])

        return {
            'sequenceId': sequence_id,
            'sequence': sequence_payload,
            'confirmResponse': confirm_response,
            'terminalEvent': terminal_event,
        }

    def _build_pythonpath(self) -> str:
        paths = [str(WORKSPACE_DIR)]
        existing = self._environment.get('PYTHONPATH')
        if existing:
            paths.append(existing)
        return ':'.join(paths)

    def _reserve_ros_domain_id(self) -> int:
        type(self)._domain_counter += 1
        seed = os.getpid() + int(time.time() * 1000) + type(self)._domain_counter
        return 120 + (seed % 40)

    def _reserve_tcp_port(self) -> int:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(('127.0.0.1', 0))
            return int(sock.getsockname()[1])

    def _start_sim_stack(self) -> None:
        if self._sim_process is not None:
            return
        command = (
            f'source "{INSTALL_SETUP}" && '
            f'ros2 launch gp4_bringup sim.launch.py '
            f'use_rviz:=false audit_log_path:="{self._audit_dir}"'
        )
        self._sim_log_handle = (self._log_root / 'sim.log').open('w', encoding='utf-8')
        self._sim_process = subprocess.Popen(
            ['bash', '-lc', command],
            cwd=str(WORKSPACE_DIR),
            env=self._environment,
            stdout=self._sim_log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    def _start_api_server(self) -> None:
        if self._api_process is not None:
            return
        command = (
            f'source "{INSTALL_SETUP}" && '
            f'python3 -m uvicorn {APP_MODULE} --host 127.0.0.1 --port {self._port}'
        )
        self._api_log_handle = (self._log_root / 'api.log').open('w', encoding='utf-8')
        self._api_process = subprocess.Popen(
            ['bash', '-lc', command],
            cwd=str(WORKSPACE_DIR),
            env=self._environment,
            stdout=self._api_log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    def _base_url(self) -> str:
        return f'http://127.0.0.1:{self._port}'

    def _wait_for_snapshot(
        self,
        *,
        expected_transport: str,
        expected_telemetry: str,
        expected_runtime: str,
        timeout_sec: float = 150.0,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_sec
        last_error: Exception | None = None
        startup_started_at = time.monotonic()
        while time.monotonic() < deadline:
            self._assert_process_alive(self._sim_process, 'sim stack')
            self._assert_process_alive(self._api_process, 'api server')
            try:
                payload = self._get_json(
                    '/api/hmi/snapshot?session_id=probe-session&operator_id=probe-operator'
                )
            except Exception as exc:  # pragma: no cover - transient startup path
                last_error = exc
                if (
                    self._api_restart_count < 3
                    and self._is_connection_refused(exc)
                    and (time.monotonic() - startup_started_at) >= 20.0
                ):
                    self._restart_api_server()
                    self._api_restart_count += 1
                    startup_started_at = time.monotonic()
                time.sleep(0.5)
                continue
            if (
                payload.get('transportState') == expected_transport
                and payload.get('telemetryState') == expected_telemetry
                and payload.get('runtime', {}).get('systemState') == expected_runtime
            ):
                return payload
            time.sleep(0.5)
        if last_error is not None:
            raise AssertionError(f'snapshot did not become ready: {last_error}')
        raise AssertionError('snapshot did not reach expected state before timeout')

    @staticmethod
    def _is_connection_refused(error: Exception) -> bool:
        if isinstance(error, urllib.error.URLError):
            reason = str(error.reason).lower()
            return 'connection refused' in reason
        return False

    def _restart_api_server(self) -> None:
        self._stop_process(self._api_process)
        self._api_process = None
        if self._api_log_handle is not None:
            self._api_log_handle.close()
            self._api_log_handle = None
        self._port = self._reserve_tcp_port()
        self._start_api_server()

    def _assert_process_alive(self, process: subprocess.Popen[str] | None, label: str) -> None:
        if process is None:
            raise AssertionError(f'{label} process was not started')
        code = process.poll()
        if code is not None:
            raise AssertionError(f'{label} exited early with code {code}')

    def _get_json(self, path: str) -> dict[str, Any]:
        url = f'{self._base_url()}{path}'
        with urllib.request.urlopen(url, timeout=5.0) as response:
            return json.loads(response.read().decode('utf-8'))

    def _post_json(
        self,
        path: str,
        payload: dict[str, Any],
        *,
        timeout_sec: float = 30.0,
    ) -> dict[str, Any]:
        body = json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(
            f'{self._base_url()}{path}',
            data=body,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise AssertionError(f'POST {path} failed: {exc.code} {detail}') from exc

    async def _collect_stream_events(
        self,
        *,
        session_id: str,
        operator_id: str,
        stop_command_id: str,
        stop_states: set[str],
        ready_event: asyncio.Event | None = None,
        timeout_sec: float = 90.0,
    ) -> dict[str, Any]:
        return await self._collect_terminal_stream_event(
            session_id=session_id,
            operator_id=operator_id,
            event_type='command_lifecycle',
            payload_key='command',
            entity_id_field='commandId',
            stop_entity_id=stop_command_id,
            stop_states=stop_states,
            ready_event=ready_event,
            timeout_sec=timeout_sec,
        )

    async def _collect_terminal_stream_event(
        self,
        *,
        session_id: str,
        operator_id: str,
        event_type: str,
        payload_key: str,
        entity_id_field: str,
        stop_entity_id: str,
        stop_states: set[str],
        ready_event: asyncio.Event | None = None,
        timeout_sec: float = 90.0,
    ) -> dict[str, Any]:
        uri = (
            f'ws://127.0.0.1:{self._port}/api/hmi/stream?'
            f'session_id={urllib.parse.quote(session_id)}&operator_id={urllib.parse.quote(operator_id)}'
        )
        deadline = time.monotonic() + timeout_sec
        async with websockets.connect(uri, max_size=4 * 1024 * 1024) as websocket:
            if ready_event is not None:
                ready_event.set()
            while time.monotonic() < deadline:
                remaining = max(0.1, deadline - time.monotonic())
                raw = await asyncio.wait_for(websocket.recv(), timeout=remaining)
                event = json.loads(raw)
                if event.get('type') != event_type:
                    continue
                payload = event.get(payload_key) or {}
                if payload.get(entity_id_field) != stop_entity_id:
                    continue
                if payload.get('lifecycleState') in stop_states:
                    return event
        raise AssertionError(f'timed out waiting for terminal {event_type} event')

    def _stop_process(self, process: subprocess.Popen[str] | None) -> None:
        if process is None:
            return
        if process.poll() is not None:
            return
        self._terminate_process_group(process, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._terminate_process_group(process, signal.SIGKILL)
            process.wait(timeout=5)

    @staticmethod
    def _terminate_process_group(process: subprocess.Popen[str], sig: signal.Signals) -> None:
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except ProcessLookupError:
            return
        except Exception:
            if sig == signal.SIGTERM:
                process.terminate()
            else:
                process.kill()


if __name__ == '__main__':
    unittest.main()
