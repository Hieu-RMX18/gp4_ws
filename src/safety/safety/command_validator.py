import json

from .policy_loader import _FAILSAFE_MOTION_LIMITS

_VELOCITY_BYPASS_PRIMITIVES = {
    "ALARM_RESET",
    "STOP",
    "WAIT",
    "IO_SET",
    "GET_POSE",
    "FACTORY_TASK_RUNTIME",
}

# Fail-safe only — active policy is loaded from safety_rules.yaml via constructor.
_DEFAULT_MAX_SCALE = _FAILSAFE_MOTION_LIMITS["max_velocity_scale"]


class CommandValidator:
    def __init__(self, safety_rules: dict):
        self.safety_rules = safety_rules
        motion_limits = self.safety_rules.get("motion_limits", {})
        legacy_limits = self.safety_rules.get("joint_limits_override", {})
        self.max_velocity = motion_limits.get(
            "max_velocity_scale",
            legacy_limits.get("max_velocity_scale", _DEFAULT_MAX_SCALE),
        )
        self.max_acceleration = motion_limits.get(
            "max_acceleration_scale",
            legacy_limits.get("max_acceleration_scale", _DEFAULT_MAX_SCALE),
        )

    def validate(self, command_json: str) -> tuple[bool, str]:
        if not command_json:
            return False, "Empty JSON"

        try:
            cmd = json.loads(command_json)
        except json.JSONDecodeError:
            return False, "Invalid JSON format"

        primitive_type = cmd.get("primitive_type")
        if not primitive_type:
            return False, "Missing primitive_type"

        if primitive_type in _VELOCITY_BYPASS_PRIMITIVES:
            return True, ""

        # W7: extended_mode precondition gate (before velocity check)
        extended_ok, extended_reason = self._check_extended_mode_preconditions(cmd)
        if not extended_ok:
            return False, extended_reason

        # Policy-aware defaulting: absent scale → use the safety policy ceiling.
        # This prevents a missing field from being silently treated as 100% speed.
        velocity_scale = cmd.get("velocity_scale")
        if velocity_scale is not None:
            try:
                velocity_scale = float(velocity_scale)
            except (ValueError, TypeError):
                return False, "Invalid velocity_scale format"

            if velocity_scale > self.max_velocity:
                return (
                    False,
                    f"velocity_scale {velocity_scale} exceeds max allowed {self.max_velocity}",
                )

            if velocity_scale <= 0.0:
                return False, "velocity_scale must be positive"
        # else: absent field is acceptable here; motion_core resolves the actual
        # default at execution time via TrajectoryPostProcessor::kDefaultVelocityScaling.
        # This validator only checks for out-of-bounds or invalid values.

        acceleration_scale = cmd.get("acceleration_scale")
        if acceleration_scale is not None:
            try:
                acceleration_scale = float(acceleration_scale)
            except (ValueError, TypeError):
                return False, "Invalid acceleration_scale format"

            if acceleration_scale > self.max_acceleration:
                return False, (
                    f"acceleration_scale {acceleration_scale} exceeds "
                    f"max allowed {self.max_acceleration}"
                )

            if acceleration_scale <= 0.0:
                return False, "acceleration_scale must be positive"
        # else: absent field is acceptable; motion_core resolves default at execution time.

        return True, ""

    def _check_extended_mode_preconditions(self, command: dict) -> tuple:
        """Validate extended_mode preconditions.

        Checks 1-4 below are pre-wired for the future ``joint_6_t.extended``
        config key. They are intentionally inert until the YAML operator
        activates that key; the early-exit guard above enforces the
        fail-closed posture by rejecting requests when extended mode is
        not enabled in ``safety_rules.yaml``.
        """
        if not command.get("extended_mode"):
            return True, ""
        cfg = self.safety_rules.get("operational_joint_limits", {}).get("joint_6_t")
        disabled_reason = "extended_mode disabled; use default joint_6_t envelope"
        if not isinstance(cfg, dict):
            return False, disabled_reason
        if "extended" not in cfg:
            return False, disabled_reason

        pre = cfg.get("extended_preconditions", {})

        # 1. Cable inspection sign-off
        if pre.get("cable_inspection_signed_off") and not command.get(
            "cable_inspection_signed_off_token"
        ):
            return (
                False,
                "extended_mode requires cable_inspection_signed_off_token; "
                "operator must certify",
            )

        # 2. Velocity cap
        cap = float(pre.get("max_velocity_scale", 0.10))
        vel = float(command.get("velocity_scale", 0.06))
        if vel > cap:
            return (
                False,
                f"extended_mode velocity_scale={vel} > cap {cap}",
            )

        # 3. Operator confirm token
        if pre.get("requires_operator_confirm") and not command.get(
            "operator_confirm_token"
        ):
            return (
                False,
                "extended_mode requires operator_confirm_token (single-command scope)",
            )

        # 4. Estimated duration cap
        estimated_s = float(command.get("estimated_duration_s", 0))
        max_duration = float(pre.get("max_continuous_extended_time_s", 30))
        if estimated_s > max_duration:
            return (
                False,
                f"extended_mode estimated_duration {estimated_s}s "
                f"> cap {max_duration}s",
            )

        return True, ""
