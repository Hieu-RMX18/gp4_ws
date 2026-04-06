import os
import threading
import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from industrial_msgs.msg import RobotStatus
from industrial_msgs.msg import TriState
from interfaces.msg import RobotReadiness

from .command_validator import CommandValidator
from .workspace_guard import WorkspaceGuard
from .execution_gate import ExecutionGate


class SafetyManager(Node):
    """V4 H1/H2: Fail-closed safety manager with execution lock.

    Monitors robot status via industrial_msgs/RobotStatus.
    Blocks execution when:
    - Robot in_error
    - Robot e_stopped
    - Robot not ready (drives not powered, motion not possible)
    - No trusted joint state available (no status received)
    """

    def __init__(self):
        super().__init__('safety_manager')
        self._sim_mode = bool(self.declare_parameter('sim_mode', False).value)

        # V4 H2: Fail-closed — start in blocked state until robot proves ready
        self._lock = threading.Lock()
        self._robot_ready = False
        self._last_error_reason = (
            "no hw_adapter readiness received yet (sim mode)"
            if self._sim_mode else
            "no robot status received yet (fail-closed)"
        )
        self._status_received = False
        self._adapter_ready_received = False

        # Load safety rules from config
        try:
            pkg_share = get_package_share_directory('safety')
            yaml_path = os.path.join(pkg_share, 'config', 'safety_rules.yaml')
            with open(yaml_path, 'r') as f:
                self.safety_rules = yaml.safe_load(f)
            self.get_logger().info("Loaded safety rules successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load safety rules: {e}")
            self.safety_rules = {}

        # Initialize internal modules
        self.validator = CommandValidator(self.safety_rules)
        self.guard = WorkspaceGuard(self.safety_rules)
        self.gate = ExecutionGate(self, self.validator, self.guard, self)

        # Safety status publisher (structured status)
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)

        if self._sim_mode:
            self.adapter_ready_sub = self.create_subscription(
                RobotReadiness,
                '/hw_adapter/ready',
                self.adapter_ready_callback,
                10
            )
            self.get_logger().info(
                "safety running in SIM MODE: using /hw_adapter/ready for readiness gate"
            )
            self.get_logger().info(
                "SafetyManager started (fail-closed until hw_adapter readiness received)."
            )
        else:
            # V4 J5: Correct subscriber type: industrial_msgs/RobotStatus
            self.status_sub = self.create_subscription(
                RobotStatus,
                '/yaskawa/robot_status',
                self.status_callback,
                10
            )
            self.get_logger().info("SafetyManager started (fail-closed until robot status received).")

    @property
    def is_robot_ready(self) -> bool:
        """Thread-safe check: is the robot ready for execution?"""
        with self._lock:
            return self._robot_ready

    @property
    def last_error_reason(self) -> str:
        """Thread-safe: why is the robot not ready?"""
        with self._lock:
            return self._last_error_reason

    def status_callback(self, msg: RobotStatus):
        """V4 H1-Layer4: Update robot readiness from controller status."""
        if self._sim_mode:
            return

        with self._lock:
            self._status_received = True

            # Check safety-critical fields
            if msg.in_error.val == TriState.TRUE:
                self._robot_ready = False
                self._last_error_reason = "robot in_error via /yaskawa/robot_status"
                self.get_logger().error(
                    "SAFETY BLOCK: robot reported in_error")
                self.publish_status("ERROR: in_error active")
                return

            if msg.e_stopped.val == TriState.TRUE:
                self._robot_ready = False
                self._last_error_reason = "robot e_stopped via /yaskawa/robot_status"
                self.get_logger().error(
                    "SAFETY BLOCK: robot e_stopped")
                self.publish_status("ERROR: E-stop active")
                return

            if msg.drives_powered.val != TriState.TRUE:
                self._robot_ready = False
                self._last_error_reason = "drives not powered"
                self.publish_status("BLOCKED: drives not powered")
                return

            if msg.motion_possible.val != TriState.TRUE:
                self._robot_ready = False
                self._last_error_reason = "motion not possible"
                self.publish_status("BLOCKED: motion not possible")
                return

            # All checks passed — robot is ready
            self._robot_ready = True
            self._last_error_reason = ""
            self.publish_status("OK")

    def adapter_ready_callback(self, msg: RobotReadiness):
        """SIM MODE: update readiness from normalized hw_adapter readiness."""
        if not self._sim_mode:
            return

        with self._lock:
            self._adapter_ready_received = True

            if not msg.ready:
                self._robot_ready = False
                self._last_error_reason = (
                    msg.status_message
                    if msg.status_message else
                    "hw_adapter reported not ready (sim mode)"
                )
                self.publish_status(f"BLOCKED: {self._last_error_reason}")
                return

            self._robot_ready = True
            self._last_error_reason = ""
            self.publish_status("OK")

    def publish_status(self, status_msg: str):
        msg = String()
        msg.data = status_msg
        self.safety_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
