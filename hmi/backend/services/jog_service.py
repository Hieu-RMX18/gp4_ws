"""
JogService — backend ROS bridge for jog command handling.

Subscribes to:
  /servo_bridge/status  (interfaces/msg/ServoBridgeStatus)

Service clients:
  /servo_bridge/activate   (std_srvs/srv/Trigger)
  /servo_bridge/deactivate  (std_srvs/srv/Trigger)

Provides:
  - REST endpoints: POST /api/hmi/jog/activate, /deactivate, /command
  - WebSocket events: { type: 'jog_bridge_status', jogBridgeStatus: {...} }
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from threading import Lock, Thread
from typing import Any

try:
    import rclpy
except Exception:  # pragma: no cover - depends on sourced ROS environment
    rclpy = None

try:
    from interfaces.msg import JogCommand as JogCommandMsg
    from interfaces.msg import ServoBridgeStatus as ServoBridgeStatusMsg
    from std_srvs.srv import Trigger
except Exception:  # pragma: no cover - ROS env may not be sourced
    JogCommandMsg = None
    ServoBridgeStatusMsg = None
    Trigger = None


class JogBridgeState(str, Enum):
    IDLE = 'IDLE'
    STARTING = 'STARTING'
    READY = 'READY'
    ACTIVE = 'ACTIVE'
    HALTING = 'HALTING'
    HALTED = 'HALTED'
    ERROR = 'ERROR'
    REJECTED_NOT_READY = 'REJECTED_NOT_READY'
    REJECTED_FJT_ACTIVE = 'REJECTED_FJT_ACTIVE'
    TIMEOUT = 'TIMEOUT'
    BUSY_RETRY = 'BUSY_RETRY'


@dataclass(slots=True)
class JogBridgeStatusView:
    state: JogBridgeState
    points_queued: int
    effective_hz: float
    robot_ready: bool
    servo_active: bool
    bridge_active: bool
    last_error: str
    rejection_reason: str


# Default status when ROS bridge is not running
DEFAULT_JOG_STATUS = JogBridgeStatusView(
    state=JogBridgeState.IDLE,
    points_queued=0,
    effective_hz=0.0,
    robot_ready=False,
    servo_active=False,
    bridge_active=False,
    last_error='',
    rejection_reason='',
)


def _build_jog_status_event(status: JogBridgeStatusView) -> dict[str, Any]:
    return {
        'type': 'jog_bridge_status',
        'jogBridgeStatus': {
            'state': status.state.value,
            'pointsQueued': status.points_queued,
            'effectiveHz': status.effective_hz,
            'robotReady': status.robot_ready,
            'servoActive': status.servo_active,
            'bridgeActive': status.bridge_active,
            'lastError': status.last_error,
            'rejectionReason': status.rejection_reason,
        },
    }


class JogService:
    """
    Backend service for the jog pendant.

    Subscribes to /servo_bridge/status and forwards events to WebSocket clients.
    Exposes REST endpoints that forward to the ROS service clients in servo_bridge_node.
    """

    def __init__(
        self,
        *,
        status_topic: str = '/servo_bridge/status',
        activate_service: str = '/servo_bridge/activate',
        deactivate_service: str = '/servo_bridge/deactivate',
        jog_command_topic: str = '/web_jog_command',
        activate_timeout_sec: float = 5.0,
    ) -> None:
        self._status_topic = status_topic
        self._activate_service = activate_service
        self._deactivate_service = deactivate_service
        self._jog_command_topic = jog_command_topic
        self._activate_timeout_sec = activate_timeout_sec

        self._status = DEFAULT_JOG_STATUS
        self._status_lock = Lock()
        self._subscribers: dict[int, Any] = {}
        self._subscriber_lock = Lock()

        self._context: Any = None
        self._node: Any = None
        self._executor: Any = None
        self._thread: Thread | None = None
        self._stop_requested = False

        self._activate_client: Any = None
        self._deactivate_client: Any = None
        self._jog_pub: Any = None

    def start(self) -> None:
        if rclpy is None or JogCommandMsg is None:
            return
        if self._thread is not None:
            return

        try:
            self._context = rclpy.context.Context()
            rclpy.init(args=None, context=self._context)
            self._node = rclpy.create_node(
                'gp4_hmi_jog_pendant_service', context=self._context
            )
            self._executor = rclpy.executors.SingleThreadedExecutor(
                context=self._context
            )
            self._executor.add_node(self._node)
            self._create_clients()
            self._create_subscriptions()
            self._stop_requested = False
            self._thread = Thread(
                target=self._spin, name='gp4_hmi_jog_pendant_service-spin', daemon=True
            )
            self._thread.start()
        except Exception:
            self.stop()

    def stop(self) -> None:
        self._stop_requested = True
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=0.2)
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self._context is not None and rclpy is not None:
            try:
                if self._context.ok():
                    rclpy.shutdown(context=self._context)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._executor = None
        self._node = None
        self._context = None

    def subscribe(self) -> Any:
        import queue
        q: Any = queue.Queue()
        with self._subscriber_lock:
            self._subscribers[id(q)] = q
        return q

    def unsubscribe(self, q: Any) -> None:
        with self._subscriber_lock:
            self._subscribers.pop(id(q), None)

    def _broadcast(self, event: dict[str, Any]) -> None:
        with self._subscriber_lock:
            for q in list(self._subscribers.values()):
                try:
                    q.put_nowait(event)
                except Exception:
                    pass

    def get_status(self) -> JogBridgeStatusView:
        with self._status_lock:
            return self._status

    def activate_bridge(self) -> tuple[bool, str]:
        """
        Call /servo_bridge/activate.
        Returns (accepted, message).
        """
        if self._node is None or self._activate_client is None or Trigger is None:
            return False, 'ROS node unavailable — jog bridge service not started.'

        if not self._activate_client.service_is_ready():
            return False, f'Activation service not ready at {self._activate_service}.'

        request = Trigger.Request()
        future = self._activate_client.call_async(request)

        deadline = datetime.now(timezone.utc).timestamp() + self._activate_timeout_sec
        while not future.done():
            if datetime.now(timezone.utc).timestamp() >= deadline:
                return False, 'Activation service call timed out.'
            import time
            time.sleep(0.05)

        try:
            response = future.result()
        except Exception as exc:
            return False, f'Activation call failed: {exc}'

        if response is None:
            return False, 'Activation returned no response.'

        success = getattr(response, 'success', False)
        message = str(getattr(response, 'message', ''))
        return bool(success), message

    def deactivate_bridge(self) -> tuple[bool, str]:
        """
        Call /servo_bridge/deactivate.
        Returns (accepted, message).
        """
        if self._node is None or self._deactivate_client is None or Trigger is None:
            return False, 'ROS node unavailable — jog bridge service not started.'

        if not self._deactivate_client.service_is_ready():
            return False, f'Deactivation service not ready at {self._deactivate_service}.'

        request = Trigger.Request()
        future = self._deactivate_client.call_async(request)

        deadline = datetime.now(timezone.utc).timestamp() + self._activate_timeout_sec
        while not future.done():
            if datetime.now(timezone.utc).timestamp() >= deadline:
                return False, 'Deactivation service call timed out.'
            import time
            time.sleep(0.05)

        try:
            response = future.result()
        except Exception as exc:
            return False, f'Deactivation call failed: {exc}'

        if response is None:
            return False, 'Deactivation returned no response.'

        success = getattr(response, 'success', False)
        message = str(getattr(response, 'message', ''))
        return bool(success), message

    def send_jog_command(self, *, joint_index: int, direction: int, mode: str,
                          velocity_scale: float, step_degrees: float) -> tuple[bool, str]:
        """
        Publish a JogCommand message to /web_jog_command.
        Returns (success, reason).
        """
        if self._node is None or self._jog_pub is None:
            return False, 'ROS node or publisher unavailable'

        if JogCommandMsg is None:
            return False, 'JogCommand message type unavailable'

        with self._status_lock:
            status = self._status
        if not status.bridge_active:
            return False, 'bridge not active'
        if not status.robot_ready or not status.servo_active:
            return False, 'robot not ready or servo inactive'
        if status.state not in {JogBridgeState.READY, JogBridgeState.ACTIVE}:
            return False, f'bridge state {status.state.value} not READY/ACTIVE'

        msg = JogCommandMsg()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.joint_index = int(joint_index)
        msg.direction = int(direction)
        msg.step_degrees = float(step_degrees)
        msg.velocity_scale = float(velocity_scale)
        msg.mode = str(mode)

        try:
            self._jog_pub.publish(msg)
            return True, 'published'
        except Exception:
            return False, 'publish exception'

    def _create_clients(self) -> None:
        if self._node is None:
            return
        if Trigger is not None:
            self._activate_client = self._node.create_client(
                Trigger, self._activate_service
            )
            self._deactivate_client = self._node.create_client(
                Trigger, self._deactivate_service
            )
        if JogCommandMsg is not None:
            self._jog_pub = self._node.create_publisher(
                JogCommandMsg, self._jog_command_topic, 10
            )

    def _create_subscriptions(self) -> None:
        if self._node is None or ServoBridgeStatusMsg is None:
            return
        self._node.create_subscription(
            ServoBridgeStatusMsg,
            self._status_topic,
            self._on_servo_bridge_status,
            10,
        )

    def _on_servo_bridge_status(self, msg: Any) -> None:
        state_str = str(getattr(msg, 'state', 'IDLE'))
        try:
            state = JogBridgeState(state_str)
        except ValueError:
            state = JogBridgeState.IDLE

        new_status = JogBridgeStatusView(
            state=state,
            points_queued=int(getattr(msg, 'points_queued', 0)),
            effective_hz=float(getattr(msg, 'effective_hz', 0.0)),
            robot_ready=bool(getattr(msg, 'robot_ready', False)),
            servo_active=bool(getattr(msg, 'servo_active', False)),
            bridge_active=bool(getattr(msg, 'bridge_active', False)),
            last_error=str(getattr(msg, 'last_error', '')),
            rejection_reason=str(getattr(msg, 'rejection_reason', '')),
        )

        with self._status_lock:
            self._status = new_status

        self._broadcast(_build_jog_status_event(new_status))

    def _spin(self) -> None:  # pragma: no cover - requires ROS runtime
        assert self._executor is not None
        assert self._context is not None
        while not self._stop_requested and self._context.ok():
            self._executor.spin_once(timeout_sec=0.2)
