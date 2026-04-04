import json
from rclpy.node import Node
from interfaces.srv import ValidateCommand
from .command_validator import CommandValidator
from .workspace_guard import WorkspaceGuard


class ExecutionGate:
    """V4 H2: Fail-closed execution gate.

    Validates commands AND checks robot readiness before allowing execution.
    """

    def __init__(self, node: Node, validator: CommandValidator,
                 guard: WorkspaceGuard, safety_manager=None):
        self.node = node
        self.validator = validator
        self.guard = guard
        # Reference to SafetyManager for readiness checks
        self._safety_manager = safety_manager

        self.srv = self.node.create_service(
            ValidateCommand,
            '/validate_command',
            self.validate_callback
        )
        self.node.get_logger().info("ExecutionGate initialized: /validate_command service ready.")

    def validate_callback(self, request, response):
        cmd_json = request.command_json

        # V4 H2: Fail-closed readiness check BEFORE command validation.
        # If robot is not ready, reject ALL commands.
        if self._safety_manager is not None:
            if not self._safety_manager.is_robot_ready:
                reason = self._safety_manager.last_error_reason
                self.node.get_logger().warn(
                    f"Execution blocked (fail-closed): {reason}")
                response.valid = False
                response.reason = f"BLOCKED: {reason}"
                response.sanitized_json = ""
                return response

        # 1. Validate JSON and parameters
        is_valid_cmd, reason_cmd = self.validator.validate(cmd_json)
        if not is_valid_cmd:
            self.node.get_logger().warn(f"Validation failed: {reason_cmd}")
            response.valid = False
            response.reason = reason_cmd
            response.sanitized_json = ""
            return response

        # 2. Extract command data
        cmd_data = json.loads(cmd_json)
        prim_type = cmd_data.get("primitive_type", "")

        # 3. Check workspace and collisions if target_pose should be used
        # Skip pose check for: HOME, STOP, or PTP when joint_target is provided
        joint_target = cmd_data.get("joint_target")
        ptp_with_joints = (
            prim_type == "PTP" and
            isinstance(joint_target, list) and
            len(joint_target) > 0
        )
        if prim_type not in ["HOME", "STOP"] and not ptp_with_joints:
            is_valid_pose, reason_pose = self.guard.check_pose(request.target_pose)
            if not is_valid_pose:
                self.node.get_logger().warn(f"Pose validation failed: {reason_pose}")
                response.valid = False
                response.reason = reason_pose
                response.sanitized_json = ""
                return response

        # 4. Valid command
        response.valid = True
        response.reason = "OK"
        # Normalize and stringify JSON
        response.sanitized_json = json.dumps(cmd_data)

        self.node.get_logger().info(f"Command validated successfully: {prim_type}")
        return response
