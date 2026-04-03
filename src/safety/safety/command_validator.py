import json

class CommandValidator:
    def __init__(self, safety_rules: dict):
        self.safety_rules = safety_rules
        self.max_velocity = self.safety_rules.get("joint_limits_override", {}).get("max_velocity_scale", 0.5)

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

        velocity_scale = cmd.get("velocity_scale", 1.0)
        try:
            velocity_scale = float(velocity_scale)
        except (ValueError, TypeError):
            return False, "Invalid velocity_scale format"

        if velocity_scale > self.max_velocity:
            return False, f"velocity_scale {velocity_scale} exceeds max allowed {self.max_velocity}"
            
        if velocity_scale <= 0.0:
            return False, "velocity_scale must be positive"

        return True, ""
