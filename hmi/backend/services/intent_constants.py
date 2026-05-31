"""Intent resolution constants: field allowlists, primitive sets, planner defaults."""

from __future__ import annotations

import re

UNIT_TO_METERS = {"mm": 0.001, "cm": 0.01, "m": 1.0}
DEFAULT_DRAW_TEXT_HEIGHT = 20.0
DEFAULT_DRAW_TEXT_UNITS = "mm"
ROUTED_DRAW_METADATA_FIELDS = {"plan_only", "chunk_index", "stroke_index"}
SUPPORTED_PRIMITIVES = {
    "HOME",
    "PTP",
    "LIN",
    "CIRC",
    "CARTESIAN_PATH",
    "MOVE_REL",
    "MOVE_JOINT",
    "MOVE_JOINTS",
    "WAIT",
    "STOP",
    "SET_SPEED",
    "IO_SET",
    "ALARM_RESET",
    "GET_POSE",
    "BLENDED_SEQUENCE",
}
HARDWARE_WHITELIST = set(SUPPORTED_PRIMITIVES)
PLANNER_DEFAULTS = {
    "HOME": "PILZ_PTP",
    "PTP": "PILZ_PTP",
    "LIN": "PILZ_LIN",
    "CIRC": "PILZ_CIRC",
    "CARTESIAN_PATH": "PILZ_LIN",
    "MOVE_REL": "PILZ_LIN",
    "MOVE_JOINT": "PILZ_PTP",
    "MOVE_JOINTS": "PILZ_PTP",
    "BLENDED_SEQUENCE": "PILZ_LIN",
}
MOTION_PRIMITIVES = {
    "HOME",
    "PTP",
    "LIN",
    "CIRC",
    "CARTESIAN_PATH",
    "MOVE_REL",
    "MOVE_JOINT",
    "MOVE_JOINTS",
    "BLENDED_SEQUENCE",
}
_OLD_ACTIONS = {
    "move_home": "HOME",
    "home": "HOME",
    "stop": "STOP",
    "move_rel": "MOVE_REL",
    "move_cartesian_delta": "MOVE_REL",
    "move_joint": "MOVE_JOINT",
    "move_joint_delta": "MOVE_JOINT",
    "move_joints": "MOVE_JOINTS",
    "wait": "WAIT",
    "set_speed": "SET_SPEED",
    "io_set": "IO_SET",
    "alarm_reset": "ALARM_RESET",
    "get_pose": "GET_POSE",
    "ptp": "PTP",
    "lin": "LIN",
    "circ": "CIRC",
    "cartesian_path": "CARTESIAN_PATH",
}
_ALLOWED_FIELDS_BY_PRIMITIVE = {
    "HOME": {"velocity_scale", "acceleration_scale", "planner_id", "reference_frame"},
    "PTP": {
        "target_pose",
        "joint_target",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "reference_frame",
    },
    "LIN": {
        "target_pose",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "reference_frame",
    },
    "CIRC": {
        "target_pose",
        "waypoints",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "reference_frame",
    },
    "CARTESIAN_PATH": {
        "waypoints",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "reference_frame",
    },
    "MOVE_REL": {
        "delta_x",
        "delta_y",
        "delta_z",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "reference_frame",
    },
    "MOVE_JOINT": {
        "joint_index",
        "joint_angle",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
    },
    "MOVE_JOINTS": {
        "joint_target",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
    },
    "WAIT": {"wait_duration_sec", "reference_frame"},
    "STOP": {"reference_frame"},
    "SET_SPEED": {"velocity_scale"},
    "IO_SET": {"io_address", "io_value", "reference_frame"},
    "ALARM_RESET": {"reference_frame"},
    "GET_POSE": {"reference_frame"},
    "BLENDED_SEQUENCE": {
        "sequence_steps",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "reference_frame",
    },
}
_CARTESIAN_DIRECTIONS = {
    "up": (0.0, 0.0, 1.0),
    "down": (0.0, 0.0, -1.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "forward": (1.0, 0.0, 0.0),
    "back": (-1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
}

_DRAW_TEXT_PREFIX_PATTERN = re.compile(
    r"^(?:write|ve\s+chu|vẽ\s+chữ|viet|viết)\s+(.+)$", re.IGNORECASE
)
