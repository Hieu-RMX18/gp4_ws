from __future__ import annotations

import asyncio
import json
import os
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
        self._port = self._reserve_tcp_port()
        self._audit_dir = self._log_root / 'audit'
        self._audit_dir.mkdir(parents=True, exist_ok=True)
        self._environment = os.environ.copy()
        self._environment['PYTHONPATH'] = self._build_pythonpath()
        self._environment['ROS_DOMAIN_ID'] = str(self._domain_id)
        self._environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        self._environment.setdefault('ROS_LOG_DIR', str(self._log_root / 'ros_logs'))
        self._sim_process: subprocess.Popen[str] | None = None
        self._api_process: subprocess.Popen[str] | None = None

    async def asyncTearDown(self) -> None:
        self._stop_process(self._api_process)
        self._stop_process(self._sim_process)

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
            expected_action='move_home',
        )

        self.assertEqual(result['command']['executionResult']['status'], 'succeeded')
        self.assertEqual(result['command']['finalState'], 'SUCCEEDED')

    async def test_supported_hmi_commands_execute_in_sim(self) -> None:
        self._start_sim_stack()
        self._start_api_server()
        await self._wait_until_sim_ready()

        cases = [
            {'intent_text': 'home', 'expected_action': 'move_home'},
            {'intent_text': 'move up 10 cm', 'expected_action': 'move_cartesian_delta'},
            {'intent_text': 'move joint 2 5 deg', 'expected_action': 'move_joint_delta'},
            {'intent_text': 'stop', 'expected_action': 'stop'},
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
        self._sim_process = subprocess.Popen(
            ['bash', '-lc', command],
            cwd=str(WORKSPACE_DIR),
            env=self._environment,
            stdout=(self._log_root / 'sim.log').open('w', encoding='utf-8'),
            stderr=subprocess.STDOUT,
            text=True,
        )

    def _start_api_server(self) -> None:
        if self._api_process is not None:
            return
        command = (
            f'source "{INSTALL_SETUP}" && '
            f'python3 -m uvicorn {APP_MODULE} --host 127.0.0.1 --port {self._port}'
        )
        self._api_process = subprocess.Popen(
            ['bash', '-lc', command],
            cwd=str(WORKSPACE_DIR),
            env=self._environment,
            stdout=(self._log_root / 'api.log').open('w', encoding='utf-8'),
            stderr=subprocess.STDOUT,
            text=True,
        )

    def _base_url(self) -> str:
        return f'http://127.0.0.1:{self._port}'

    def _wait_for_snapshot(
        self,
        *,
        expected_transport: str,
        expected_telemetry: str,
        expected_runtime: str,
        timeout_sec: float = 90.0,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_sec
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            self._assert_process_alive(self._sim_process, 'sim stack')
            self._assert_process_alive(self._api_process, 'api server')
            try:
                payload = self._get_json(
                    '/api/hmi/snapshot?session_id=probe-session&operator_id=probe-operator'
                )
            except Exception as exc:  # pragma: no cover - transient startup path
                last_error = exc
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
                if event.get('type') != 'command_lifecycle':
                    continue
                command = event.get('command') or {}
                if command.get('commandId') != stop_command_id:
                    continue
                if command.get('lifecycleState') in stop_states:
                    return event
        raise AssertionError('timed out waiting for terminal command lifecycle event')

    def _stop_process(self, process: subprocess.Popen[str] | None) -> None:
        if process is None:
            return
        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
