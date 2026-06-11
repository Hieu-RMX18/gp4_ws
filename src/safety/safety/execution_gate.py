import json
import math
from rclpy.node import Node
from interfaces.srv import ValidateCommand
from .command_validator import CommandValidator
from .workspace_guard import WorkspaceGuard


# MOVE_REL delta safety limits — must match motion_core/move_rel_validator.hpp.
# Defense-in-depth: gate rejects bad deltas before they reach motion_core.
# Hardware conservative MOVE_REL cap — short nudges only.
_MOVE_REL_MAX_DELTA_NORM = 0.21
_MOVE_REL_ALLOWED_FRAMES = {"", "base_link"}

# BLENDED_SEQUENCE step primitive allowlist.
_BLENDED_SEQUENCE_STEP_PRIMITIVES = {"LIN", "PTP"}


class ExecutionGate:
    """V4 H2: Fail-closed execution gate.

    Validates commands AND checks robot readiness before allowing execution.

    MOVE_REL pose-check skip rationale
    ------------------------------------
    The execution_gate skips workspace-bound checking on target_pose for
    MOVE_REL because the absolute target does not exist yet — it is
    computed from (current_pose + delta) inside motion_core at planning
    time.  Fail-closed is maintained by three layers of defense:

      1. This gate validates delta magnitude (≤ 0.21 m) and reference
         frame (base_link only) right here — catching bad commands before
         they reach motion_core.
      2. motion_core validates the resolved target against the same
         workspace bounds and keepout zones used by WorkspaceGuard,
         immediately before planning (see move_rel_validator.hpp).
      3. MoveIt planning itself rejects unreachable / collision targets.

    If any layer rejects, the command does not execute.
    """

    def __init__(
        self,
        node: Node,
        validator: CommandValidator,
        guard: WorkspaceGuard,
        safety_manager=None,
    ):
        self.node = node
        self.validator = validator
        self.guard = guard
        motion_limits = self.validator.safety_rules.get("motion_limits", {})
        using_fallback_move_rel_limit = "max_move_rel_translation" not in motion_limits
        self._max_move_rel_delta_norm = float(
            motion_limits.get(
                "max_move_rel_translation",
                _MOVE_REL_MAX_DELTA_NORM,
            )
        )
        if using_fallback_move_rel_limit:
            self.node.get_logger().warn(
                "motion_limits.max_move_rel_translation missing; "
                f"using fallback {_MOVE_REL_MAX_DELTA_NORM:.2f} m"
            )
        # Reference to SafetyManager for readiness checks
        self._safety_manager = safety_manager

        from std_msgs.msg import String
        self._trace_pub = self.node.create_publisher(String, "/llm_debug", 10)

        self.srv = self.node.create_service(
            ValidateCommand, "/validate_command", self.validate_callback
        )
        self.node.get_logger().info(
            "ExecutionGate initialized: /validate_command service ready."
        )

    def _emit_trace(self, event: str, level: str = "INFO", summary: str = "", **details):
        import time
        from std_msgs.msg import String
        payload = json.dumps({
            "t": "command_trace",
            "ts": time.time(),
            "cmd_id": "",
            "layer": "safety",
            "phase": "validation",
            "event": event,
            "level": level,
            "summary": summary,
            "details": details if details else None,
            "error_why": details.get("error_why", ""),
            "error_where": details.get("error_where", ""),
            "error_next_action": details.get("error_next_action", "")
        })
        self._trace_pub.publish(String(data=payload))

    def validate_callback(self, request, response):
        self._emit_trace("validate_command_called", summary="Validation started")
        try:
            return self._validate_callback_inner(request, response)
        finally:
            if response.valid:
                self._emit_trace("validation_passed", level="INFO", summary="All safety checks passed", sanitized_json=response.sanitized_json)
            else:
                self._emit_trace("validation_failed", level="ERROR", summary=response.reason, error_why=response.reason, error_where="safety/validate_command")

    def _validate_callback_inner(self, request, response):
        cmd_json = request.command_json
        raw_primitive_type = ""
        try:
            raw_command = json.loads(cmd_json)
            if isinstance(raw_command, dict):
                raw_primitive_type = str(raw_command.get("primitive_type", ""))
        except json.JSONDecodeError:
            pass

        # V4 H2: Fail-closed readiness check BEFORE command validation.
        # If robot is not ready, reject ALL commands.
        if self._safety_manager is not None:
            if not self._safety_manager.is_robot_ready:
                if raw_primitive_type == "ALARM_RESET":
                    self.node.get_logger().warn(
                        "Bypassing readiness gate for ALARM_RESET recovery path."
                    )
                    self._emit_trace("readiness_bypass", summary="ALARM_RESET bypasses readiness gate")
                else:
                    reason = self._safety_manager.last_error_reason
                    self.node.get_logger().warn(
                        f"Execution blocked (fail-closed): {reason}"
                    )
                    self._emit_trace("readiness_blocked", level="ERROR",
                                     summary=f"Robot not ready: {reason}",
                                     error_why=reason, error_where="safety/readiness_check")
                    response.valid = False
                    response.reason = f"BLOCKED: {reason}"
                    response.sanitized_json = ""
                    return response
            else:
                self._emit_trace("readiness_ok", summary="Robot readiness check PASSED")

        # 1. Validate JSON and parameters
        self._emit_trace("param_validation_start", summary=f"Validating {raw_primitive_type} JSON parameters...")
        is_valid_cmd, reason_cmd = self.validator.validate(cmd_json)
        if not is_valid_cmd:
            self.node.get_logger().warn(f"Validation failed: {reason_cmd}")
            self._emit_trace("param_validation_failed", level="ERROR",
                             summary=f"Parameter check FAILED: {reason_cmd}",
                             error_why=reason_cmd, error_where="safety/command_validator")
            response.valid = False
            response.reason = reason_cmd
            response.sanitized_json = ""
            return response
        self._emit_trace("param_validation_passed", summary=f"{raw_primitive_type} parameter check PASSED")

        # 2. Extract command data
        cmd_data = json.loads(cmd_json)
        prim_type = cmd_data.get("primitive_type", "")

        # 3a. MOVE_REL: validate deltas and frame at the gate (defense-in-depth).
        #     Pose check is skipped because the absolute target is computed
        #     from current_pose + delta inside motion_core — see class docstring.
        if prim_type == "MOVE_REL":
            dx = cmd_data.get('delta_x', 0)
            dy = cmd_data.get('delta_y', 0)
            dz = cmd_data.get('delta_z', 0)
            self._emit_trace("move_rel_delta_check",
                             summary=f"Checking MOVE_REL deltas: dx={dx} dy={dy} dz={dz} frame={cmd_data.get('reference_frame','')}",
                             delta_x=str(dx), delta_y=str(dy), delta_z=str(dz),
                             max_norm=str(self._max_move_rel_delta_norm))
            move_rel_ok, move_rel_reason = self._validate_move_rel_deltas(cmd_data)
            if not move_rel_ok:
                self.node.get_logger().warn(
                    f"MOVE_REL delta validation failed: {move_rel_reason}"
                )
                self._emit_trace("move_rel_delta_rejected", level="ERROR",
                                 summary=move_rel_reason,
                                 error_why=move_rel_reason, error_where="safety/move_rel_deltas")
                response.valid = False
                response.reason = move_rel_reason
                response.sanitized_json = ""
                return response
            self._emit_trace("move_rel_delta_passed", summary="MOVE_REL delta magnitude and frame OK")

        # 3b. Check workspace and collisions if target_pose should be used.
        #     Skip for primitives that are non-pose commands.
        joint_target = cmd_data.get("joint_target")
        ptp_with_joints = (
            prim_type == "PTP"
            and isinstance(joint_target, list)
            and len(joint_target) > 0
        )
        pose_validation_skip_primitives = {
            "HOME",
            "STOP",
            "MOVE_REL",
            "SET_SPEED",
            "WAIT",
            "ALARM_RESET",
            "IO_SET",
            "MOVE_JOINT",
            "MOVE_JOINTS",
            "GET_POSE",
            # BLENDED_SEQUENCE validates every sequence_steps target below.
            "BLENDED_SEQUENCE",
            "FACTORY_TASK_RUNTIME",
        }
        if prim_type not in pose_validation_skip_primitives and not ptp_with_joints:
            p = request.target_pose.position
            self._emit_trace("workspace_check",
                             summary=f"Checking target pose in workspace: x={p.x:.4f} y={p.y:.4f} z={p.z:.4f}")
            is_valid_pose, reason_pose = self.guard.check_pose(request.target_pose)
            if not is_valid_pose:
                self.node.get_logger().warn(f"Pose validation failed: {reason_pose}")
                self._emit_trace("workspace_rejected", level="ERROR",
                                 summary=f"Target pose OUTSIDE workspace: {reason_pose}",
                                 error_why=reason_pose, error_where="safety/workspace_guard")
                response.valid = False
                response.reason = reason_pose
                response.sanitized_json = ""
                return response
            self._emit_trace("workspace_passed", summary="Target pose within workspace bounds")

        # 3c. CIRC: validate auxiliary waypoint[0] workspace bounds (fail-closed)
        if prim_type == "CIRC":
            waypoints = cmd_data.get("waypoints")
            if not isinstance(waypoints, list) or len(waypoints) != 1:
                response.valid = False
                response.reason = "CIRC requires exactly 1 auxiliary waypoint"
                response.sanitized_json = ""
                return response

            aux_wp = waypoints[0]
            if not isinstance(aux_wp, dict) or not isinstance(
                aux_wp.get("position"), dict
            ):
                response.valid = False
                response.reason = "CIRC auxiliary waypoint must include position{x,y,z}"
                response.sanitized_json = ""
                return response

            try:
                ax = float(aux_wp["position"].get("x", 0.0))
                ay = float(aux_wp["position"].get("y", 0.0))
                az = float(aux_wp["position"].get("z", 0.0))
            except (TypeError, ValueError) as e:
                self.node.get_logger().warn(f"CIRC auxiliary waypoint parse error: {e}")
                response.valid = False
                response.reason = "CIRC auxiliary waypoint: parse error"
                response.sanitized_json = ""
                return response

            if not (math.isfinite(ax) and math.isfinite(ay) and math.isfinite(az)):
                response.valid = False
                response.reason = "CIRC auxiliary waypoint position must be finite"
                response.sanitized_json = ""
                return response

            # Build a temporary pose for WorkspaceGuard
            aux_pose = request.target_pose.__class__()
            aux_pose.position.x = ax
            aux_pose.position.y = ay
            aux_pose.position.z = az
            aux_pose.orientation.x = 0.0
            aux_pose.orientation.y = 0.0
            aux_pose.orientation.z = 0.0
            aux_pose.orientation.w = 1.0

            aux_is_valid, aux_reason = self.guard.check_pose(aux_pose)
            if not aux_is_valid:
                self.node.get_logger().warn(
                    f"CIRC auxiliary waypoint validation failed: {aux_reason}"
                )
                response.valid = False
                response.reason = f"CIRC auxiliary pose: {aux_reason}"
                response.sanitized_json = ""
                return response

        # 3d. CARTESIAN_PATH: validate every waypoint against WorkspaceGuard.
        if prim_type == "CARTESIAN_PATH":
            cart_ok, cart_reason = self._validate_cartesian_path_waypoints(
                waypoints=cmd_data.get("waypoints"),
                pose_cls=request.target_pose.__class__,
            )
            if not cart_ok:
                self.node.get_logger().warn(
                    f"CARTESIAN_PATH waypoint validation failed: {cart_reason}"
                )
                response.valid = False
                response.reason = cart_reason
                response.sanitized_json = ""
                return response

        # 3e. BLENDED_SEQUENCE: comprehensive per-step validation (W4)
        if prim_type == "BLENDED_SEQUENCE":
            blended_ok, blended_reason = self._validate_blended_sequence(
                cmd_data, request
            )
            if not blended_ok:
                self.node.get_logger().warn(
                    f"BLENDED_SEQUENCE validation failed: {blended_reason}"
                )
                response.valid = False
                response.reason = blended_reason
                response.sanitized_json = ""
                return response

        # 4. Valid command
        response.valid = True
        response.reason = "OK"
        # Normalize and stringify JSON
        response.sanitized_json = json.dumps(cmd_data)

        self.node.get_logger().info(f"Command validated successfully: {prim_type}")
        return response

    def _validate_blended_sequence(
        self, cmd_data: dict, request
    ) -> tuple[bool, str]:
        """Fail-closed validation for BLENDED_SEQUENCE before planning.

        Checks:
        - At least 2 sequence_steps.
        - Every step blend_radius >= 0; last step blend_radius == 0.0.
        - Step primitive_type in {LIN, PTP}.
        - Per-step velocity/acceleration scale within policy caps.
        - Only pose-target steps are accepted because motion_core only plans
          BLENDED_SEQUENCE as pose-based MoveGroupSequence items.
        - POSE steps: workspace bounds via WorkspaceGuard.
        """
        steps = cmd_data.get("sequence_steps")
        if not isinstance(steps, list) or len(steps) < 2:
            return False, "BLENDED_SEQUENCE requires at least 2 sequence_steps"

        # Top-level velocity / acceleration caps (same as CommandValidator)
        max_velocity = self.validator.max_velocity
        max_acceleration = self.validator.max_acceleration

        for idx, step in enumerate(steps):
            if not isinstance(step, dict):
                return False, f"BLENDED_SEQUENCE step[{idx}] must be a dict"

            step_prim = str(step.get("primitive_type", "")).strip().upper()
            if step_prim and step_prim not in _BLENDED_SEQUENCE_STEP_PRIMITIVES:
                return (
                    False,
                    f"BLENDED_SEQUENCE step[{idx}] unsupported primitive_type "
                    f"'{step_prim}'; allowed: {_BLENDED_SEQUENCE_STEP_PRIMITIVES}",
                )

            # blend_radius checks
            blend_radius = step.get("blend_radius_m")
            if blend_radius is None:
                blend_radius = step.get("blend_radius")
            try:
                blend_radius = float(blend_radius) if blend_radius is not None else 0.0
            except (TypeError, ValueError):
                return (
                    False,
                    f"BLENDED_SEQUENCE step[{idx}] blend_radius must be numeric",
                )
            if blend_radius < 0.0:
                return (
                    False,
                    f"BLENDED_SEQUENCE step[{idx}] blend_radius must be >= 0.0",
                )
            if idx == len(steps) - 1 and blend_radius != 0.0:
                return (
                    False,
                    f"BLENDED_SEQUENCE last step blend_radius must be 0.0, "
                    f"got {blend_radius}",
                )

            # Per-step velocity / acceleration scale caps
            for scale_key, max_val, label in (
                ("velocity_scale", max_velocity, "velocity_scale"),
                ("acceleration_scale", max_acceleration, "acceleration_scale"),
            ):
                scale = step.get(scale_key)
                if scale is not None:
                    try:
                        scale = float(scale)
                    except (TypeError, ValueError):
                        return (
                            False,
                            f"BLENDED_SEQUENCE step[{idx}] {label} invalid",
                        )
                    if scale > max_val:
                        return (
                            False,
                            f"BLENDED_SEQUENCE step[{idx}] {label}={scale} "
                            f"exceeds max allowed {max_val}",
                        )
                    if scale <= 0.0:
                        return (
                            False,
                            f"BLENDED_SEQUENCE step[{idx}] {label} must be positive",
                        )

            # goal_type dispatch
            goal_type = int(step.get("goal_type", 0))

            if goal_type == 1:  # GOAL_JOINTS
                return (
                    False,
                    f"BLENDED_SEQUENCE step[{idx}] GOAL_JOINTS is not supported; "
                    "use a normal sequence so joint moves run step-by-step",
                )

            if goal_type == 2:  # GOAL_NAMED
                return (
                    False,
                    f"BLENDED_SEQUENCE step[{idx}] GOAL_NAMED is not supported; "
                    "use a normal sequence so named poses run step-by-step",
                )

            # Default / GOAL_POSE (0): workspace bounds check
            step_pose = step.get("target_pose")
            if not isinstance(step_pose, dict) or "position" not in step_pose:
                return (
                    False,
                    f"BLENDED_SEQUENCE step[{idx}] missing target_pose.position",
                )
            pos = step_pose["position"]
            try:
                wp_pose = request.target_pose.__class__()
                wp_pose.position.x = float(pos["x"])
                wp_pose.position.y = float(pos["y"])
                wp_pose.position.z = float(pos["z"])
            except (TypeError, ValueError, KeyError) as exc:
                return (
                    False,
                    f"BLENDED_SEQUENCE step[{idx}] position invalid: {exc}",
                )
            is_valid, reason = self.guard.check_pose(wp_pose)
            if not is_valid:
                return (
                    False,
                    f"BLENDED_SEQUENCE step[{idx}]: {reason}",
                )

        return True, ""

    def _validate_move_rel_deltas(self, cmd_data: dict) -> tuple:
        """Defense-in-depth: validate MOVE_REL deltas and frame at the safety gate.

        Returns (bool, str) — (True, "") on success, (False, reason) on failure.
        """
        ref_frame = cmd_data.get("reference_frame", "")
        if ref_frame not in _MOVE_REL_ALLOWED_FRAMES:
            return False, (
                f"MOVE_REL: unsupported reference_frame '{ref_frame}'; "
                "only 'base_link' is allowed"
            )

        try:
            dx = float(cmd_data.get("delta_x", 0.0))
            dy = float(cmd_data.get("delta_y", 0.0))
            dz = float(cmd_data.get("delta_z", 0.0))
        except (TypeError, ValueError):
            return False, "MOVE_REL: delta values must be numeric"

        if not (math.isfinite(dx) and math.isfinite(dy) and math.isfinite(dz)):
            return False, "MOVE_REL: delta values must be finite"

        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            return False, "MOVE_REL: all deltas are zero"

        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm > self._max_move_rel_delta_norm:
            return False, (
                f"MOVE_REL: delta norm {norm:.4f} m exceeds "
                f"limit {self._max_move_rel_delta_norm} m"
            )

        return True, ""

    def _validate_cartesian_path_waypoints(self, waypoints, pose_cls) -> tuple:
        if not isinstance(waypoints, list) or len(waypoints) == 0:
            return False, "CARTESIAN_PATH requires non-empty waypoints"

        for idx, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                return False, f"CARTESIAN_PATH waypoint[{idx}] must be an object"

            position = waypoint.get("position")
            if not isinstance(position, dict):
                return False, (
                    f"CARTESIAN_PATH waypoint[{idx}] must include position{{x,y,z}}"
                )

            for axis in ("x", "y", "z"):
                if axis not in position:
                    return False, (
                        f"CARTESIAN_PATH waypoint[{idx}] position.{axis} is required"
                    )

            try:
                x = float(position["x"])
                y = float(position["y"])
                z = float(position["z"])
            except (TypeError, ValueError):
                return False, (
                    f"CARTESIAN_PATH waypoint[{idx}] position must be numeric"
                )

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                return False, (
                    f"CARTESIAN_PATH waypoint[{idx}] position must be finite"
                )

            waypoint_pose = pose_cls()
            waypoint_pose.position.x = x
            waypoint_pose.position.y = y
            waypoint_pose.position.z = z
            waypoint_pose.orientation.x = 0.0
            waypoint_pose.orientation.y = 0.0
            waypoint_pose.orientation.z = 0.0
            waypoint_pose.orientation.w = 1.0

            is_valid, reason = self.guard.check_pose(waypoint_pose)
            if not is_valid:
                return False, f"CARTESIAN_PATH waypoint[{idx}]: {reason}"

        return True, ""
