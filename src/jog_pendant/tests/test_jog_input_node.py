# Copyright 2026 hieu2
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import importlib.util
import math
import sys
import threading
import types
from pathlib import Path
from types import SimpleNamespace


class _FakeTime:
    def __init__(self, seconds: float) -> None:
        self.nanoseconds = int(seconds * 1_000_000_000)

    def to_msg(self) -> dict[str, int]:
        return {'sec': int(self.nanoseconds // 1_000_000_000)}


class _FakeClock:
    def __init__(self, seconds: float) -> None:
        self._seconds = seconds

    def now(self) -> _FakeTime:
        return _FakeTime(self._seconds)


class _FakeLogger:
    def __init__(self) -> None:
        self.warns: list[str] = []
        self.infos: list[str] = []
        self.debugs: list[str] = []

    def warn(self, message: str) -> None:
        self.warns.append(message)

    def info(self, message: str) -> None:
        self.infos.append(message)

    def debug(self, message: str) -> None:
        self.debugs.append(message)


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


def _install_stub_modules() -> None:
    if 'rclpy' not in sys.modules:
        rclpy_module = types.ModuleType('rclpy')
        rclpy_module.init = lambda *args, **kwargs: None
        rclpy_module.shutdown = lambda *args, **kwargs: None
        rclpy_module.spin = lambda *args, **kwargs: None
        sys.modules['rclpy'] = rclpy_module

    node_module = types.ModuleType('rclpy.node')
    node_module.Node = object
    sys.modules['rclpy.node'] = node_module

    qos_module = types.ModuleType('rclpy.qos')

    class QoSProfile:
        def __init__(self, depth: int, reliability: object) -> None:
            self.depth = depth
            self.reliability = reliability

    class Reliability:
        RELIABLE = 'reliable'

    qos_module.QoSProfile = QoSProfile
    qos_module.Reliability = Reliability
    sys.modules['rclpy.qos'] = qos_module

    control_msgs_module = types.ModuleType('control_msgs.msg')

    class JointJog:
        def __init__(self) -> None:
            self.header = SimpleNamespace(stamp=None)
            self.joint_names: list[str] = []
            self.displacements: list[float] = []
            self.velocities: list[float] = []
            self.duration: float = 0.0

    control_msgs_module.JointJog = JointJog
    sys.modules['control_msgs.msg'] = control_msgs_module

    interfaces_module = types.ModuleType('interfaces.msg')
    interfaces_module.JogCommand = object
    sys.modules['interfaces.msg'] = interfaces_module


_install_stub_modules()
SCRIPT_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'jog_input_node.py'
SPEC = importlib.util.spec_from_file_location('jog_input_node', SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _build_fake_self(*, now_sec: float = 10.0):
    logger = _FakeLogger()
    publisher = _FakePublisher()
    fake_self = SimpleNamespace(
        _joint_names=list(MODULE.GP4_JOINT_NAMES),
        _joint_max_velocity=[
            MODULE.GP4_JOINT_MAX_VELOCITY_RAD_S[name]
            for name in MODULE.GP4_JOINT_NAMES
        ],
        _max_velocity_scale=0.3,
        _min_velocity_scale=0.01,
        _max_step_degrees=10.0,
        _min_step_degrees=0.01,
        _watchdog_timeout_ms=200,
        _default_velocity_scale=0.05,
        _servo_cmd_topic='delta_joint_cmds',
        _state='IDLE',
        _active_joint_index=None,
        _active_direction=0,
        _active_velocity_scale=0.05,
        _active_mode='continuous',
        _last_heartbeat_sec=now_sec,
        _state_lock=threading.Lock(),
        _jog_pub=publisher,
        get_clock=lambda: _FakeClock(now_sec),
        get_logger=lambda: logger,
    )
    fake_self._publish_continuous = lambda *args, **kwargs: (
        MODULE.JogInputNode._publish_continuous(fake_self, *args, **kwargs)
    )
    fake_self._publish_discrete = lambda *args, **kwargs: (
        MODULE.JogInputNode._publish_discrete(fake_self, *args, **kwargs)
    )
    fake_self._halt_locked = lambda *args, **kwargs: (
        MODULE.JogInputNode._halt_locked(fake_self, *args, **kwargs)
    )
    return fake_self, publisher, logger


def test_invalid_joint_index_is_rejected_without_publish() -> None:
    fake_self, publisher, logger = _build_fake_self()
    message = SimpleNamespace(
        joint_index=99,
        direction=1,
        mode='continuous',
        velocity_scale=0.05,
        step_degrees=1.0,
    )

    MODULE.JogInputNode._on_jog_command(fake_self, message)

    assert publisher.messages == []
    assert logger.warns
    assert fake_self._state == 'IDLE'


def test_continuous_command_clamps_velocity_scale_to_safe_max() -> None:
    fake_self, publisher, _logger = _build_fake_self()
    message = SimpleNamespace(
        joint_index=0,
        direction=1,
        mode='continuous',
        velocity_scale=99.0,
        step_degrees=1.0,
    )

    MODULE.JogInputNode._on_jog_command(fake_self, message)

    assert len(publisher.messages) == 1
    published = publisher.messages[0]
    assert published.joint_names == ['joint_1_s']
    assert math.isclose(
        published.velocities[0],
        MODULE.GP4_JOINT_MAX_VELOCITY_RAD_S['joint_1_s'] * 0.3,
        rel_tol=1e-9,
    )
    assert published.duration == 0.0
    assert fake_self._state == 'ACTIVE'
    assert fake_self._active_joint_index == 0
    assert fake_self._active_direction == 1


def test_discrete_command_clamps_step_and_returns_to_idle() -> None:
    fake_self, publisher, _logger = _build_fake_self()
    message = SimpleNamespace(
        joint_index=2,
        direction=-1,
        mode='discrete',
        velocity_scale=0.0,
        step_degrees=999.0,
    )

    MODULE.JogInputNode._on_jog_command(fake_self, message)

    assert len(publisher.messages) == 1
    published = publisher.messages[0]
    assert published.joint_names == ['joint_3_u']
    assert math.isclose(published.displacements[0], -math.radians(10.0), rel_tol=1e-9)
    assert math.isclose(
        published.velocities[0],
        -MODULE.GP4_JOINT_MAX_VELOCITY_RAD_S['joint_3_u'] * 0.05,
        rel_tol=1e-9,
    )
    assert 0.05 <= published.duration <= 5.0
    assert fake_self._state == 'IDLE'
    assert fake_self._active_joint_index is None
    assert fake_self._active_direction == 0


def test_watchdog_halts_active_motion_after_timeout() -> None:
    fake_self, publisher, _logger = _build_fake_self(now_sec=1.0)
    fake_self._state = 'ACTIVE'
    fake_self._active_joint_index = 1
    fake_self._active_direction = 1
    fake_self._last_heartbeat_sec = 0.0

    MODULE.JogInputNode._on_watchdog(fake_self)

    assert len(publisher.messages) == 1
    published = publisher.messages[0]
    assert published.joint_names == ['joint_2_l']
    assert published.velocities == [0.0]
    assert published.duration == 0.0
    assert fake_self._state == 'IDLE'
    assert fake_self._active_joint_index is None
    assert fake_self._active_direction == 0
