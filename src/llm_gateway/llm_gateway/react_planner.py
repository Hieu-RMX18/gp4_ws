"""Configuration loader for the phase-9 local 9router backend."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory

_MODEL_PLACEHOLDER = "TEEN MODEL 9ROUTER"


def _default_safety_rules_path() -> str:
    """Resolve safety_rules.yaml from installed package or local source tree."""
    try:
        pkg_share = get_package_share_directory("safety")
        return os.path.join(pkg_share, "config", "safety_rules.yaml")
    except Exception:
        return str(
            Path(__file__).resolve().parents[2]
            / "safety"
            / "config"
            / "safety_rules.yaml"
        )


def _load_safety_temperature() -> float:
    """Read llm.react.temperature from safety_rules.yaml SSOT."""
    try:
        path = _default_safety_rules_path()
        with open(path, "r", encoding="utf-8") as f:
            rules = yaml.safe_load(f) or {}
        llm = rules.get("llm", {})
        react = llm.get("react", {})
        return float(react.get("temperature", 0.0))
    except Exception:
        return 0.0


def _default_config_path() -> str:
    try:
        pkg_share = get_package_share_directory("llm_gateway")
        return os.path.join(pkg_share, "config", "llm.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "llm.yaml")


def _as_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _pick_first_non_empty(*values: Any) -> str:
    for value in values:
        if value is None:
            continue
        text = str(value)
        if text:
            return text
    return ""


@dataclass(frozen=True)
class LLMBackendConfig:
    provider: str
    base_url: str
    api_key: str
    api_mode: str
    model: str
    temperature: float
    max_tokens: int
    require_json_only: bool
    fail_on_non_json: bool
    fail_on_schema_mismatch: bool
    request_timeout_sec: float
    # Retry policy for transient failures (network errors, HTTP 5xx, HTTP 429).
    # max_retries=0 disables retry. Non-transient failures (auth, 4xx) never retry.
    max_retries: int = 2
    retry_base_delay_sec: float = 0.5
    retry_max_delay_sec: float = 4.0

    @property
    def model_is_configured(self) -> bool:
        return bool(self.model) and self.model != _MODEL_PLACEHOLDER


def load_llm_backend_config(config_path: str | None = None) -> LLMBackendConfig:
    resolved_path = config_path or _default_config_path()
    with open(resolved_path, "r", encoding="utf-8") as config_file:
        raw_config = yaml.safe_load(config_file) or {}
    if not isinstance(raw_config, dict):
        raise ValueError("llm.yaml root must be a mapping.")

    backend = raw_config.get("llm_backend") or {}
    if not isinstance(backend, dict):
        raise ValueError("llm_backend must be a mapping.")

    def _lookup_env(env_name: str) -> str:
        return os.getenv(env_name, "")

    def _resolve_env_ref(value: Any) -> Any:
        """Resolve ${ENV_VAR} references in string values."""
        if not isinstance(value, str):
            return value
        if value.startswith("${") and value.endswith("}"):
            env_name = value[2:-1]
            return _lookup_env(env_name)
        return value

    def pick(key: str, default: Any) -> Any:
        env_key = f"LLM_{key.upper()}"
        raw = _pick_first_non_empty(_lookup_env(env_key), backend.get(key, default))
        return _resolve_env_ref(raw)

    # api_key: dedicated gateway env vars win, then generic OpenAI-compatible env,
    # then the YAML value, which can also be written as ${ENV_VAR}.
    raw_api_key = _pick_first_non_empty(
        _lookup_env("GP4_LLM_API_KEY"),
        _lookup_env("LLM_API_KEY"),
        _lookup_env("OPENAI_API_KEY"),
        _resolve_env_ref(backend.get("api_key", "")),
    )

    return LLMBackendConfig(
        provider=str(backend.get("provider", "9router_local")),
        base_url=str(pick("base_url", "http://localhost:20128/v1")).rstrip("/"),
        api_key=str(raw_api_key),
        api_mode=str(backend.get("api_mode", "openai_compatible")),
        model=str(pick("model", _MODEL_PLACEHOLDER)),
        temperature=float(pick("temperature", _load_safety_temperature())),
        max_tokens=int(pick("max_tokens", 500)),
        require_json_only=_as_bool(pick("require_json_only", True), True),
        fail_on_non_json=_as_bool(pick("fail_on_non_json", True), True),
        fail_on_schema_mismatch=_as_bool(pick("fail_on_schema_mismatch", True), True),
        request_timeout_sec=float(pick("request_timeout_sec", 10.0)),
        max_retries=int(pick("max_retries", 2)),
        retry_base_delay_sec=float(pick("retry_base_delay_sec", 0.5)),
        retry_max_delay_sec=float(pick("retry_max_delay_sec", 4.0)),
    )


"""Prompt construction — Semantic IR output (v2.1).

The LLM always outputs Semantic IR JSON with an ``intent`` field.
IntentRouter converts Semantic IR to primitive commands.
This keeps LLM output simple, consistent, and decoupled from the
internal primitive contract.

Direct primitive JSON (with ``primitive_type``) is only accepted
via the /llm_raw_command backward-compatibility path and is NOT
part of this prompt's output contract.
"""

import logging
from uuid import uuid4


# ── Frozen semantic intent list ─────────────────────────────────────────────
# Must match IntentRouter._route_single_intent exactly.
# Update this set AND the prompt text simultaneously — never one without
# the other.  The contract consistency test enforces this.
FROZEN_SEMANTIC_INTENTS = frozenset(
    {
        "go_home",
        "stop",
        "alarm_reset",
        "get_pose",
        "set_speed",
        "wait",
        "move_relative",
        "absolute_move_ptp",
        "move_named_pose",
        "absolute_move_lin",
        "circular_move",
        "move_joint",
        "move_joints",
        "io_set",
        "draw_shape",
        "draw_text",
        "return_to_start",
    }
)
FROZEN_TOP_LEVEL_OUTPUT_INTENTS = FROZEN_SEMANTIC_INTENTS | {"sequence"}

_DEFAULT_WORKSPACE_BOUNDS = {
    "x_min": -0.45,
    "x_max": 0.45,
    "y_min": -0.16,
    "y_max": 0.52,
    "z_min": 0.23,
    "z_max": 0.65,
}

_LOGGER = logging.getLogger(__name__)


def _coerce_workspace_bounds(raw: dict | None) -> dict[str, float]:
    if not isinstance(raw, dict):
        raw = {}
    return {
        key: float(raw.get(key, default))
        for key, default in _DEFAULT_WORKSPACE_BOUNDS.items()
    }


def _load_yaml(path: Path) -> dict:
    if not path.exists():
        _LOGGER.warning("PromptBuilder: safety rules path not found: %s", path)
        return {}
    try:
        return yaml.safe_load(path.read_text()) or {}
    except Exception as ex:
        _LOGGER.warning("PromptBuilder: failed to parse YAML '%s': %s", path, ex)
        return {}


def _load_workspace_bounds() -> dict[str, float]:
    try:
        from ament_index_python.packages import get_package_share_directory

        safety_yaml = (
            Path(get_package_share_directory("safety")) / "config" / "safety_rules.yaml"
        )
        safety_rules = _load_yaml(safety_yaml)
        if safety_rules:
            return _coerce_workspace_bounds(safety_rules.get("workspace_bounds"))
        _LOGGER.warning(
            "PromptBuilder: safety package YAML empty; falling back to workspace copy '%s'",
            safety_yaml,
        )
    except Exception as ex:
        _LOGGER.warning(
            "PromptBuilder: failed package-level safety rules lookup; "
            "falling back to workspace copy: %s",
            ex,
        )

    workspace_yaml = (
        Path(__file__).resolve().parents[2] / "safety" / "config" / "safety_rules.yaml"
    )
    safety_rules = _load_yaml(workspace_yaml)
    if not safety_rules:
        _LOGGER.warning(
            "PromptBuilder: workspace safety rules missing/unreadable; using default bounds."
        )
    return _coerce_workspace_bounds(safety_rules.get("workspace_bounds"))


def _format_workspace_bounds(bounds: dict[str, float]) -> str:
    return (
        f"x: {bounds['x_min']:.2f}–{bounds['x_max']:.2f}, "
        f"y: {bounds['y_min']:.2f}–{bounds['y_max']:.2f}, "
        f"z: {bounds['z_min']:.2f}–{bounds['z_max']:.2f}"
    )


_SYSTEM_PROMPT_TEMPLATE = """\
You are the llm_gateway for a Yaskawa GP4 robot arm.
Your job: convert one natural-language command or one ordered multi-step request into ONE JSON object.

IMPORTANT: You are an INTENT CLASSIFIER, not a keyword matcher.
If the user's words are different but the MEANING clearly maps to an intent,
use that intent. Only output error when the intent is genuinely ambiguous,
unsafe, or missing critical parameters you cannot infer.
You MUST support both English and Vietnamese (tiếng Việt) and many other languages.

══════════════════════════════════════════════════════
OUTPUT FORMAT — always ONE JSON object, no markdown, no explanation:

A) Semantic IR (normal path):
   {"intent": "<intent_name>", ...slots...}
   {"intent": "sequence", "steps": [{...step1...}, {...step2...}]}

UNIT RULE:
  - Internal execution uses SI.
  - If the user explicitly gives non-SI linear units, keep the user-provided
    magnitude and add "linear_unit": "cm" | "mm".
  - If the user explicitly gives non-SI angular units, keep the user-provided
    magnitude and add "angular_unit": "deg".
  - If the user already speaks in meters/radians, or gives no unit, omit the
    unit field and use SI values directly.

B) Missing parameter — ask user instead of guessing:
   {"error": "MISSING_SLOT", "intent": "<intent>",
    "missing_fields": ["<field>"], "hint": "<question for user>"}

C) Unsupported/unsafe/ambiguous:
   {"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"}

══════════════════════════════════════════════════════
AVAILABLE INTENTS (trigger on MEANING, not exact words):

go_home
  Robot returns to its predefined home/rest/origin position.
  Trigger on ANY request meaning "go back to starting point".
  No required slots.
  VN: "về nhà", "về gốc", "reset vị trí", "quay lại chỗ ban đầu",
      "đưa robot về", "đưa về ban đầu", "park nó lại", "về chỗ cũ"
  → {"intent": "go_home"}

stop
  Immediately stop all robot motion. HIGH PRIORITY.
  No required slots.
  VN: "dừng", "dừng lại", "dừng ngay", "ngừng", "khẩn cấp"
  → {"intent": "stop"}

alarm_reset
  Clear robot alarm/error state.
  No required slots.
  VN: "reset lỗi", "xóa lỗi", "xóa alarm", "clear error"
  → {"intent": "alarm_reset"}

get_pose
  Ask where the robot end-effector currently is. No motion.
  Optional: reference_frame (default: "base_link")
  VN: "robot đang ở đâu", "vị trí hiện tại", "lấy vị trí", "TCP ở đâu"
  → {"intent": "get_pose"}

set_speed
  Change motion velocity scale.
  Required: velocity_scale (float 0.01–0.06)
  VN: "đặt tốc độ", "nhanh hơn", "chậm lại", "tốc độ X phần trăm"
  Rules:
    - "nhanh hơn" / "faster" without number → velocity_scale: 0.06
    - "chậm lại" / "slower" without number → velocity_scale: 0.01
    - Percentage → multiply by 0.06, then clamp to the valid range [0.01, 0.06]
  → {"intent": "set_speed", "velocity_scale": 0.06}

wait
  Pause for a specified duration.
  Required: wait_duration_sec (float; default 2.0 if unspecified but clear intent)
  VN: "chờ", "đợi", "tạm dừng", "hold", "pause"
  → {"intent": "wait", "wait_duration_sec": 3.0}

move_relative
  Move BY a relative amount from current position.
  Required: delta (object with x, y, z; set unused axes to 0.0)
  Optional: linear_unit ("m"|"cm"|"mm"), reference_frame (default: "base_link")
  Safety: single MOVE_REL translation norm must stay ≤ 0.05 m for hardware use.
  VN: "nâng lên", "hạ xuống", "dịch lên/xuống", "nhích lên", "đẩy lên", "kéo xuống"
  Axis mapping:
    up/lên/nâng/nhấc     → delta.z positive
    down/xuống/hạ         → delta.z negative
    right/phải            → delta.y positive
    left/trái             → delta.y negative
    forward/trước/tiến    → delta.x positive
    back/sau/lùi          → delta.x negative
  Unit conversions: 1 phân = 1 cm = 0.01 m, 1 mm = 0.001 m
  If user says cm/mm, preserve that unit explicitly instead of pre-converting.
  → {"intent": "move_relative", "delta": {"x": 0.0, "y": 0.0, "z": 5.0}, "linear_unit": "cm"}

absolute_move_ptp
  Move end-effector to absolute Cartesian position (joint-optimized path).
  Required: target_pose.position (object with x, y, z)
  Optional: orientation_preset ("tool-down"|"tool-forward"|"tool-up"),
            keep_current_orientation (boolean, default: true if orientation unspecified),
            velocity_scale (float 0.01–0.06),
            linear_unit ("m"|"cm"|"mm"),
            angular_unit ("rad"|"deg"),
            reference_frame (default: "base_link")
  Orientation rule: if user does NOT specify orientation,
    OMIT orientation_preset and let keep_current_orientation default to true.
    Do NOT default to tool-down for generic motions.
  VN: "di chuyển đến", "đi tới tọa độ", "đặt robot tại", "đến điểm"
  → {"intent": "absolute_move_ptp", "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}}}

move_named_pose
  Move to a verified SRDF named pose by name. This resolves to a PTP joint target.
  Required: pose_name — MUST be one of the canonical enum values: "home"|"ready"|"poseA"|"poseB"
  IMPORTANT: Always canonicalize the user's input to the exact enum value above.
    "pose A" / "A" / "point A" / "điểm A" → pose_name: "poseA"
    "pose B" / "B" / "point B" / "điểm B" → pose_name: "poseB"
    Never output "pose A" or "Pose A" — always "poseA".
  Use only when the operator explicitly names one of these taught poses.
  VN: "đến pose ready", "về pose home", "đến điểm poseA", "tới A", "về B"
  → {"intent": "move_named_pose", "pose_name": "ready"}
  → {"intent": "move_named_pose", "pose_name": "poseA"}

absolute_move_lin
  Straight-line motion to absolute Cartesian position.
  Same slots as absolute_move_ptp. Trigger when user explicitly wants straight path.
  VN: "đi thẳng tới", "đường thẳng đến", "kéo thẳng", "theo đường thẳng"
  → {"intent": "absolute_move_lin", "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.5}}}

circular_move
  Circular arc motion through an auxiliary waypoint to a target position.
  Required: target_pose.position (final position), auxiliary_pose.position (arc via-point)
  Optional: orientation_preset, keep_current_orientation, velocity_scale,
            linear_unit ("m"|"cm"|"mm"), angular_unit ("rad"|"deg"),
            reference_frame
  VN:"đi vòng", "vẽ cung", "đi theo cung tròn", "di chuyển theo cung", "arc đến"
  → {"intent": "circular_move", "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}}, "auxiliary_pose": {"position": {"x": 0.32, "y": 0.05, "z": 0.42}}}

move_joint
  Move a single joint to a specific angle.
  Required: joint_index (0–5), joint_angle (float)
  Optional: angular_unit ("rad"|"deg")
  VN: "xoay khớp N", "di chuyển khớp N", "đặt khớp N về"
  → {"intent": "move_joint", "joint_index": 2, "joint_angle": 30.0, "angular_unit": "deg"}

move_joints
  Move all 6 joints simultaneously to target angles.
  Required: joint_target (list of 6 floats)
  Optional: angular_unit ("rad"|"deg")
  VN: "di chuyển tất cả khớp", "đặt tất cả khớp về 0"
  → {"intent": "move_joints", "joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

io_set
  Set digital I/O output.
  Required: io_address (integer), io_value (0 or 1)
  VN: "bật đầu ra", "tắt đầu ra", "đặt IO"
  → {"intent": "io_set", "io_address": 10010, "io_value": 1}

draw_shape
  Draw geometric shapes via deterministic stroke compilation.
  Required:
    - shape_type: circle|arc|square|rectangle|triangle|polygon|polyline
    - units: "m" | "cm" | "mm"
    - frame_id: "base_link"
    - workplane: {"mode":"base"|"tool"|"explicit_pose", ...}
    - params: shape-specific numeric values
  Shape params:
    - circle: radius (or radius_m)
    - arc: radius + sweep_deg
    - square: side
    - rectangle: width + height
    - triangle: side OR points
    - polygon: n_sides + radius (or side)
    - polyline: points [{x,y}, ...]
  Optional:
    - stroke: approach_distance_m, retract_distance_m, drawing_speed_scale, travel_speed_scale
    - execution_mode: "execute" | "plan_only"
  VN: "vẽ hình vuông", "vẽ hình tròn bán kính ...", "vẽ đa giác", "vẽ polyline"
  → {"intent":"draw_shape","shape_type":"circle","units":"cm","frame_id":"base_link",
     "workplane":{"mode":"base","origin":{"position":{"x":0.3,"y":0.0,"z":0.4}}},
     "params":{"radius":5},"execution_mode":"plan_only"}
  → {"intent":"draw_shape","shape_type":"rectangle","units":"mm","frame_id":"base_link",
     "workplane":{"mode":"base","origin":{"position":{"x":0.3,"y":0.0,"z":0.4}}},
     "params":{"width":50,"height":80},"execution_mode":"execute"}

draw_text
  Draw single-stroke uppercase text.
  Required:
    - text
    - units: "m" | "cm" | "mm"
    - frame_id: "base_link"
    - workplane
    - font.height (or font.height_m)
  Optional:
    - font.char_spacing, font.line_spacing, font.alignment (left|center|right)
    - stroke settings and execution_mode
  Supported glyphs: A-Z, 0-9, space, ".", ",", "-", "_", "/"
  VN: "vẽ chữ GP4", "write HELLO 20 mm tall", "vẽ chữ YASKAWA cao 1 cm"
  → {"intent":"draw_text","text":"GP4","units":"mm","frame_id":"base_link",
     "workplane":{"mode":"base","origin":{"position":{"x":0.3,"y":0.0,"z":0.4}}},
     "font":{"type":"single_stroke_builtin","height":20},"execution_mode":"plan_only"}
  → {"intent":"draw_text","text":"HELLO","units":"cm","frame_id":"base_link",
     "workplane":{"mode":"base","origin":{"position":{"x":0.3,"y":0.0,"z":0.4}}},
     "font":{"type":"single_stroke_builtin","height":2,"alignment":"left"}}

return_to_start
  Sequence step only. Move the robot back to the pose it had when the current sequence began.
  No required slots. Never output return_to_start as a standalone top-level intent.
  VN: "quay về vị trí ban đầu", "trở về điểm xuất phát", "về chỗ cũ"
  → inside sequence only: {"intent": "return_to_start"}

sequence
  Use only when the user explicitly asks for multiple ordered robot actions in one request.
  Output: {"intent":"sequence","steps":[<step1>,<step2>,...]}
  Rules:
    - steps must be a non-empty list of step objects
    - never nest sequence inside sequence
    - GET_POSE is not allowed inside a sequence
    - STOP is allowed only when it is the sole step
    - motion steps in sequences must include "reference_frame":"base_link"
  Example:
    {"intent":"sequence","steps":[
      {"intent":"go_home"},
      {"intent":"wait","wait_duration_sec":1.0},
      {"intent":"absolute_move_lin","reference_frame":"base_link",
       "target_pose":{"position":{"x":0.3,"y":0.0,"z":0.3}}}
    ]}

══════════════════════════════════════════════════════
FEW-SHOT EXAMPLES (diverse Vietnamese/English variations):

User: "về nhà"
→ {"intent": "go_home"}

User: "bring the robot back to start"
→ {"intent": "go_home"}

User: "đưa nó về chỗ cũ đi"
→ {"intent": "go_home"}

User: "park it"
→ {"intent": "go_home"}

User: "go home, wait one second, then move linearly to x 0.3 y 0 z 0.3"
→ {"intent":"sequence","steps":[
   {"intent":"go_home"},
   {"intent":"wait","wait_duration_sec":1.0},
   {"intent":"absolute_move_lin","reference_frame":"base_link",
    "target_pose":{"position":{"x":0.3,"y":0.0,"z":0.3}}}
 ]}

User: "nâng lên 5cm"
→ {"intent": "move_relative", "delta": {"x": 0.0, "y": 0.0, "z": 5.0}, "linear_unit": "cm"}

User: "đưa nó lên cao thêm 5 phân"
→ {"intent": "move_relative", "delta": {"x": 0.0, "y": 0.0, "z": 5.0}, "linear_unit": "cm"}

User: "lift 5 centimeters"
→ {"intent": "move_relative", "delta": {"x": 0.0, "y": 0.0, "z": 5.0}, "linear_unit": "cm"}

User: "go up a bit"
→ {"error": "MISSING_SLOT", "intent": "move_relative",
   "missing_fields": ["delta"], "hint": "How many cm should the robot move up?"}

User: "hạ xuống một chút"
→ {"error": "MISSING_SLOT", "intent": "move_relative",
   "missing_fields": ["delta"], "hint": "Hạ xuống bao nhiêu cm?"}

User: "lower the arm 5cm"
→ {"intent": "move_relative", "delta": {"x": 0.0, "y": 0.0, "z": -5.0}, "linear_unit": "cm"}

User: "dịch sang trái 4cm"
→ {"intent": "move_relative", "delta": {"x": 0.0, "y": -4.0, "z": 0.0}, "linear_unit": "cm"}

User: "dịch sang trái"
→ {"error": "MISSING_SLOT", "intent": "move_relative",
   "missing_fields": ["delta"], "hint": "Dịch sang trái bao nhiêu cm?"}

User: "chạy tới điểm x 0.3, y 0, z 0.5 theo đường thẳng nhé"
→ {"intent": "absolute_move_lin", "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.5}}}

User: "move to x=0.3 y=0 z=0.4"
→ {"intent": "absolute_move_ptp", "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}}}

User: "move to Cartesian x 300 mm y 0 z 400"
→ {"intent": "absolute_move_ptp", "target_pose": {"position": {"x": 300.0, "y": 0.0, "z": 400.0}},
   "linear_unit": "mm", "reference_frame": "base_link"}

User: "đi tới tọa độ x 300 mm y 0 z 400"
→ {"intent": "absolute_move_ptp", "target_pose": {"position": {"x": 300.0, "y": 0.0, "z": 400.0}},
   "linear_unit": "mm", "reference_frame": "base_link"}

User: "move to x=0.3 y=0 z=0.4 with tool pointing forward"
→ {"intent": "absolute_move_ptp", "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
   "orientation_preset": "tool-forward"}

User: "set tốc độ nhanh hơn một chút"
→ {"intent": "set_speed", "velocity_scale": 0.06}

User: "chậm lại"
→ {"intent": "set_speed", "velocity_scale": 0.01}

User: "robot đang ở đâu vậy?"
→ {"intent": "get_pose"}

User: "chờ 3 giây"
→ {"intent": "wait", "wait_duration_sec": 3.0}

User: "tạm dừng 5 giây"
→ {"intent": "wait", "wait_duration_sec": 5.0}

User: "dừng ngay"
→ {"intent": "stop"}

User: "emergency stop"
→ {"intent": "stop"}

User: "xoay khớp 2 lên 30 độ"
→ {"intent": "move_joint", "joint_index": 2, "joint_angle": 30.0, "angular_unit": "deg"}

User: "rotate joint 3 to 45 degrees"
→ {"intent": "move_joint", "joint_index": 3, "joint_angle": 45.0, "angular_unit": "deg"}

User: "đặt tất cả khớp về 0"
→ {"intent": "move_joints", "joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

User: "reset lỗi"
→ {"intent": "alarm_reset"}

User: "bật đầu ra 10010"
→ {"intent": "io_set", "io_address": 10010, "io_value": 1}

User: "move to pose A"
→ {"intent": "move_named_pose", "pose_name": "poseA"}

User: "go to B"
→ {"intent": "move_named_pose", "pose_name": "poseB"}

User: "đến điểm A"
→ {"intent": "move_named_pose", "pose_name": "poseA"}

User: "về pose ready"
→ {"intent": "move_named_pose", "pose_name": "ready"}

User: "move to pose A then pose B then home"
→ {"intent":"sequence","steps":[
  {"intent":"move_named_pose","pose_name":"poseA"},
  {"intent":"move_named_pose","pose_name":"poseB"},
  {"intent":"go_home"}
]}

User: "vẽ hình tròn bán kính 5 cm"
→ {"intent":"draw_shape","shape_type":"circle","units":"cm","frame_id":"base_link",
   "workplane":{"mode":"tool"},"params":{"radius":5}}

User: "draw rectangle 50 by 80 mm"
→ {"intent":"draw_shape","shape_type":"rectangle","units":"mm","frame_id":"base_link",
   "workplane":{"mode":"tool"},"params":{"width":50,"height":80}}

User: "draw polygon 6 sides radius 20 mm"
→ {"intent":"draw_shape","shape_type":"polygon","units":"mm","frame_id":"base_link",
   "workplane":{"mode":"tool"},"params":{"n_sides":6,"radius":20}}

User: "vẽ tam giác"
→ {"error":"MISSING_SLOT","intent":"draw_shape",
   "missing_fields":["params.side"],"hint":"Cạnh tam giác dài bao nhiêu (mm/cm)?"}

User: "write GP4"
→ {"intent":"draw_text","text":"GP4","units":"mm","frame_id":"base_link",
   "workplane":{"mode":"tool"},"font":{"type":"single_stroke_builtin","height":20}}

User: "write HELLO 20 mm tall"
→ {"intent":"draw_text","text":"HELLO","units":"mm","frame_id":"base_link",
   "workplane":{"mode":"tool"},"font":{"type":"single_stroke_builtin","height":20}}

User: "write @@@"
→ {"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"}

══════════════════════════════════════════════════════
WORKSPACE LIMITS (meters): __WORKSPACE_LIMITS__
UNIT CONVERSIONS: 1 phân = 1 cm = 0.01 m | 1 mm = 0.001 m
VELOCITY SCALE: 0.01 (slow) to 0.06 (fast)
ORIENTATION PRESETS: tool-down | tool-forward | tool-up
USE SI directly when the user speaks in meters/radians or omits units.
FOR NON-SI USER INPUTS, preserve the magnitude and add linear_unit / angular_unit.
reference_frame is always "base_link" for this system.

Schema (for reference — your output is Semantic IR, NOT direct primitive):
__JSON_SCHEMA__
"""


def build_system_prompt(schema_json: str) -> str:
    """Build the system prompt for the LLM, injecting the JSON schema."""
    workspace_limits = _format_workspace_bounds(_load_workspace_bounds())
    return _SYSTEM_PROMPT_TEMPLATE.replace(
        "__WORKSPACE_LIMITS__", workspace_limits
    ).replace("__JSON_SCHEMA__", schema_json)


"""OpenAI-compatible client for a local 9router backend."""

import json
import logging
import random
import time
import urllib.error
import urllib.request
from typing import Callable, Dict, List

_LOGGER = logging.getLogger(__name__)

# HTTP status codes considered transient and worth retrying.
# 408 Request Timeout, 429 Too Many Requests, 500/502/503/504 server/upstream errors.
_TRANSIENT_HTTP_STATUS = {408, 429, 500, 502, 503, 504}


def _is_transient_http(status: int) -> bool:
    return status in _TRANSIENT_HTTP_STATUS


class OpenAICompatibleLLMClient:
    """Minimal chat/completions client for a local OpenAI-compatible backend.

    Retries only transient failures (network errors, HTTP 408/429/5xx) with
    exponential backoff and jitter. Non-transient failures (auth, 4xx except
    408/429) are raised immediately so upstream validation does not retry
    a request the server will reject deterministically.
    """

    def __init__(
        self,
        config: LLMBackendConfig,
        schema_json: str,
        sleep_fn: Callable[[float], None] = time.sleep,
        rng: random.Random | None = None,
    ):
        self._config = config
        self._system_prompt = build_system_prompt(schema_json)
        # Injected for deterministic testing.
        self._sleep = sleep_fn
        self._rng = rng or random.Random()

    def generate_response(self, user_input: str) -> str:
        if not isinstance(user_input, str) or not user_input.strip():
            raise ValueError("user_input must be a non-empty string.")
        if not self._config.model_is_configured:
            raise ValueError(
                "llm_backend.model is not configured. Set the 9router model alias in llm.yaml or LLM_MODEL."
            )

        request = self._build_request(user_input)
        max_attempts = max(1, 1 + int(self._config.max_retries))
        last_exc: Exception | None = None

        for attempt in range(1, max_attempts + 1):
            try:
                with urllib.request.urlopen(
                    request, timeout=self._config.request_timeout_sec
                ) as response:
                    return response.read().decode("utf-8")
            except urllib.error.HTTPError as exc:
                error_body = exc.read().decode("utf-8", errors="replace")
                wrapped = RuntimeError(
                    f"LLM request failed with HTTP {exc.code}: {error_body}"
                )
                wrapped.__cause__ = exc
                if not _is_transient_http(exc.code) or attempt >= max_attempts:
                    raise wrapped from exc
                last_exc = wrapped
                _LOGGER.warning(
                    "LLM transient HTTP %d on attempt %d/%d; will retry.",
                    exc.code,
                    attempt,
                    max_attempts,
                )
            except urllib.error.URLError as exc:
                wrapped = RuntimeError(f"LLM request failed: {exc.reason}")
                wrapped.__cause__ = exc
                if attempt >= max_attempts:
                    raise wrapped from exc
                last_exc = wrapped
                _LOGGER.warning(
                    "LLM network error on attempt %d/%d: %s; will retry.",
                    attempt,
                    max_attempts,
                    exc.reason,
                )

            self._sleep(self._backoff_delay(attempt))

    def generate_response_from_messages(self, messages: list[dict[str, str]]) -> str:
        """Send a request with pre-constructed messages (ReAct multi-turn)."""
        if not self._config.model_is_configured:
            raise ValueError(
                "llm_backend.model is not configured. Set the 9router model alias."
            )
        request = self._build_request_from_messages(messages)
        max_attempts = max(1, 1 + int(self._config.max_retries))
        last_exc: Exception | None = None
        for attempt in range(1, max_attempts + 1):
            try:
                with urllib.request.urlopen(
                    request, timeout=self._config.request_timeout_sec
                ) as response:
                    return response.read().decode("utf-8")
            except urllib.error.HTTPError as exc:
                error_body = exc.read().decode("utf-8", errors="replace")
                wrapped = RuntimeError(
                    f"LLM request failed with HTTP {exc.code}: {error_body}"
                )
                wrapped.__cause__ = exc
                if not _is_transient_http(exc.code) or attempt >= max_attempts:
                    raise wrapped from exc
                last_exc = wrapped
            except urllib.error.URLError as exc:
                wrapped = RuntimeError(f"LLM request failed: {exc.reason}")
                wrapped.__cause__ = exc
                if attempt >= max_attempts:
                    raise wrapped from exc
                last_exc = wrapped
            self._sleep(self._backoff_delay(attempt))
        assert last_exc is not None
        raise last_exc

    def _build_request_from_messages(
        self, messages: list[dict[str, str]]
    ) -> urllib.request.Request:
        payload = {
            "model": self._config.model,
            "temperature": self._config.temperature,
            "max_tokens": self._config.max_tokens,
            "messages": messages,
            "stream": False,
        }
        if self._config.require_json_only:
            payload["response_format"] = {"type": "json_object"}
        request_body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
        return urllib.request.Request(
            url=f"{self._config.base_url}/chat/completions",
            data=request_body,
            headers=self._build_headers(),
            method="POST",
        )

    def _build_request(self, user_input: str) -> urllib.request.Request:
        payload = {
            "model": self._config.model,
            "temperature": self._config.temperature,
            "max_tokens": self._config.max_tokens,
            "messages": self._build_messages(user_input),
            "stream": False,
        }
        if self._config.require_json_only:
            payload["response_format"] = {"type": "json_object"}

        request_body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
        return urllib.request.Request(
            url=f"{self._config.base_url}/chat/completions",
            data=request_body,
            headers=self._build_headers(),
            method="POST",
        )

    def _backoff_delay(self, attempt: int) -> float:
        """Exponential backoff with ±25% jitter, capped at retry_max_delay_sec."""
        base = max(0.0, float(self._config.retry_base_delay_sec))
        cap = max(base, float(self._config.retry_max_delay_sec))
        # attempt is 1-based: first retry after attempt=1 uses base, then 2*base, ...
        exp = base * (2 ** (attempt - 1))
        capped = min(cap, exp)
        jitter = capped * 0.25
        return max(0.0, capped + self._rng.uniform(-jitter, jitter))

    def _build_messages(self, user_input: str) -> List[Dict[str, str]]:
        return [
            {
                "role": "system",
                "content": self._system_prompt,
            },
            {
                "role": "user",
                "content": user_input.strip(),
            },
        ]

    def _build_headers(self) -> Dict[str, str]:
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        api_key = (
            self._config.api_key
            or os.getenv("LLM_API_KEY")
            or os.getenv("OPENAI_API_KEY")
        )
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"
        return headers


"""Iteration budget for ReAct agent — tiered limits."""

from dataclasses import dataclass


@dataclass
class IterationBudget:
    max_total: int = 5
    max_motion: int = 3
    max_readonly: int = 10
    max_repair: int = 1
    wall_clock_timeout_s: float = 30.0


@dataclass
class IterationCounters:
    total: int = 0
    motion: int = 0
    readonly: int = 0
    repair: int = 0

    def can_invoke(self, tool: "Tool", budget: IterationBudget) -> tuple[bool, str]:
        if self.total >= budget.max_total:
            return False, f"max_total exceeded ({self.total}/{budget.max_total})"
        if tool.is_motion and self.motion >= budget.max_motion:
            return False, f"max_motion exceeded ({self.motion}/{budget.max_motion})"
        if tool.is_readonly and self.readonly >= budget.max_readonly:
            return (
                False,
                f"max_readonly exceeded ({self.readonly}/{budget.max_readonly})",
            )
        return True, ""

    def can_invoke_any(self, budget: IterationBudget) -> tuple[bool, str]:
        if self.total >= budget.max_total:
            return False, f"max_total exceeded ({self.total}/{budget.max_total})"
        return True, ""

    def record(self, tool: "Tool") -> None:
        self.total += 1
        if tool.is_motion:
            self.motion += 1
        if tool.is_readonly:
            self.readonly += 1


"""State injector — pulls live robot state from ROS topics for ReAct prompt context."""

import logging
from typing import Optional

_LOGGER = logging.getLogger(__name__)


class StateInjector:
    """Maintains a snapshot of robot state from ROS subscriptions.

    When ROS is not running (e.g. unit tests), the snapshot returns a
    minimal safe fallback so the ReAct agent can still reason.
    """

    _DEFAULT_JOINT_NAMES: List[str] = [
        "joint_1_s",
        "joint_2_l",
        "joint_3_u",
        "joint_4_r",
        "joint_5_b",
        "joint_6_t",
    ]

    def __init__(self) -> None:
        self._last_joint_states: Optional[Dict[str, Any]] = None
        self._last_robot_status: Optional[Dict[str, Any]] = None
        self._velocity_scale_active: float = 0.06
        self._gripper_available: bool = False
        self._perception_available: bool = False

    def update_joint_states(self, msg: Dict[str, Any]) -> None:
        self._last_joint_states = msg

    def update_robot_status(self, msg: Dict[str, Any]) -> None:
        self._last_robot_status = msg

    def set_velocity_scale(self, value: float) -> None:
        self._velocity_scale_active = float(value)

    def set_capabilities(self, *, gripper: bool, perception: bool) -> None:
        self._gripper_available = gripper
        self._perception_available = perception

    def snapshot(self) -> Dict[str, Any]:
        """Return a structured state dict for inclusion in the LLM prompt."""
        joints = self._last_joint_states or {}
        position = joints.get("position", [])
        if not position:
            position = [0.0] * len(self._DEFAULT_JOINT_NAMES)

        status = self._last_robot_status or {}
        mode = status.get("mode", "IDLE")
        active_alarms = status.get("active_alarms", [])

        return {
            "robot_state": {
                "joints_rad": list(position),
                "joint_names": list(self._DEFAULT_JOINT_NAMES),
                "mode": str(mode),
                "active_alarms": list(active_alarms),
                "last_action": {
                    "tool": "",
                    "status": "",
                    "error": None,
                },
                "velocity_scale_active": float(self._velocity_scale_active),
                "capabilities": {
                    "gripper": bool(self._gripper_available),
                    "perception": bool(self._perception_available),
                },
            }
        }


"""Tool registry and base classes for ReAct agent tools."""

from dataclasses import dataclass
from typing import ClassVar

import jsonschema


@dataclass
class ToolResult:
    ok: bool
    payload: dict | None = None
    error: str | None = None

    def to_observation(self) -> str:
        if self.ok:
            return json.dumps({"ok": True, "payload": self.payload})
        return json.dumps({"ok": False, "error": self.error})


class Tool:
    name: ClassVar[str] = ""
    description: ClassVar[str] = ""
    input_schema: ClassVar[dict] = {}
    is_motion: ClassVar[bool] = False
    is_readonly: ClassVar[bool] = False

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        raise NotImplementedError

    def validate_input(self, args: dict) -> None:
        if self.input_schema:
            jsonschema.validate(instance=args, schema=self.input_schema)


class ToolRegistry:
    """Registry of ReAct tools by name."""

    def __init__(self) -> None:
        self._tools: Dict[str, Tool] = {}

    def register(self, tool: Tool) -> "ToolRegistry":
        if not tool.name:
            raise ValueError("Tool must define a name.")
        self._tools[tool.name] = tool
        return self

    def get(self, name: str) -> Tool | None:
        return self._tools.get(name)

    def list_tools(self) -> List[dict]:
        return [
            {
                "name": t.name,
                "description": t.description,
                "input_schema": t.input_schema,
                "is_motion": t.is_motion,
                "is_readonly": t.is_readonly,
            }
            for t in self._tools.values()
        ]

    def available_tools_description(self) -> str:
        lines = ["Available tools (one tool call per response, return JSON):"]
        for t in self._tools.values():
            lines.append(f"- {t.name}: {t.description}")
        return "\n".join(lines)


"""Compute arc points — local geometry tool for CIRC auxiliary poses."""

import math


def _normalize(v: list[float]) -> list[float]:
    norm = math.sqrt(sum(x * x for x in v))
    if norm < 1e-6:
        raise ValueError("zero vector")
    return [x / norm for x in v]


def _cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _quaternion_from_vectors(forward: list[float], up: list[float]) -> dict:
    """Build a quaternion that rotates +Z to `up` and +X to `forward`."""
    fx, fy, fz = forward
    ux, uy, uz = up
    # Rotation matrix: columns are forward, up cross forward, up
    cx = _normalize(_cross(up, forward))
    cy = forward
    cz = up
    # Convert to quaternion (x, y, z, w)
    w = math.sqrt(max(0.0, 1.0 + cx[0] + cy[1] + cz[2])) / 2.0
    x = math.sqrt(max(0.0, 1.0 + cx[0] - cy[1] - cz[2])) / 2.0
    y = math.sqrt(max(0.0, 1.0 - cx[0] + cy[1] - cz[2])) / 2.0
    z = math.sqrt(max(0.0, 1.0 - cx[0] - cy[1] + cz[2])) / 2.0
    # Correct signs
    if cx[1] < cy[0]:
        x = -x
    if cx[2] < cz[0]:
        y = -y
    if cy[2] < cz[1]:
        z = -z
    return {"x": x, "y": y, "z": z, "w": w}


class ComputeArcPointsTool(Tool):
    name = "compute_arc_points"
    description = (
        "Compute start, auxiliary, and target poses for a circular arc (CIRC)."
    )
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "center": {
                "type": "object",
                "required": ["x", "y", "z"],
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                },
            },
            "radius_m": {"type": "number"},
            "start_angle_rad": {"type": "number"},
            "sweep_angle_rad": {"type": "number"},
            "plane_normal": {
                "type": "object",
                "required": ["x", "y", "z"],
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                },
            },
        },
        "required": [
            "center",
            "radius_m",
            "start_angle_rad",
            "sweep_angle_rad",
            "plane_normal",
        ],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        center = args["center"]
        radius_m = float(args["radius_m"])
        start_angle = float(args["start_angle_rad"])
        sweep = float(args["sweep_angle_rad"])
        n_raw = args["plane_normal"]

        if radius_m <= 0.0:
            return ToolResult(ok=False, error="radius_m must be > 0")
        if sweep == 0.0:
            return ToolResult(ok=False, error="sweep_angle_rad must be non-zero")
        if abs(sweep) > 2.0 * math.pi:
            return ToolResult(ok=False, error="|sweep_angle_rad| must not exceed 2*pi")

        n = _normalize([n_raw["x"], n_raw["y"], n_raw["z"]])
        if math.sqrt(sum(x * x for x in n)) < 1e-6:
            return ToolResult(ok=False, error="plane_normal is degenerate")

        if abs(n[2]) > 0.9:
            u = [1.0, 0.0, 0.0]
        elif abs(n[1]) > 0.9:
            u = [1.0, 0.0, 0.0]
        else:
            u = _normalize(_cross([0.0, 0.0, 1.0], n))
        v = _cross(n, u)

        def _pose_at(angle: float) -> dict:
            x = center["x"] + radius_m * (
                u[0] * math.cos(angle) + v[0] * math.sin(angle)
            )
            y = center["y"] + radius_m * (
                u[1] * math.cos(angle) + v[1] * math.sin(angle)
            )
            z = center["z"] + radius_m * (
                u[2] * math.cos(angle) + v[2] * math.sin(angle)
            )
            # Tangent vector = derivative w.r.t angle (counter-clockwise when sweep>0)
            tx = -u[0] * math.sin(angle) + v[0] * math.cos(angle)
            ty = -u[1] * math.sin(angle) + v[1] * math.cos(angle)
            tz = -u[2] * math.sin(angle) + v[2] * math.cos(angle)
            forward = _normalize([tx, ty, tz])
            up = n
            q = _quaternion_from_vectors(forward, up)
            return {
                "header": {"frame_id": "base_link"},
                "pose": {
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": q,
                },
            }

        aux_angle = start_angle + sweep / 2.0
        end_angle = start_angle + sweep
        return ToolResult(
            ok=True,
            payload={
                "start_pose": _pose_at(start_angle),
                "auxiliary_pose": _pose_at(aux_angle),
                "target_pose": _pose_at(end_angle),
            },
        )


"""Get current robot pose tool — calls existing ROS service."""


class GetCurrentPoseTool(Tool):
    name = "get_current_pose"
    description = "Get the current robot end-effector pose in the base_link frame."
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "reference_frame": {
                "type": "string",
                "description": "Pose reference frame; defaults to base_link.",
            }
        },
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = getattr(context, "ros_node", None)
        if node is None:
            return ToolResult(
                ok=False,
                error="ros_node not available in AgentContext",
            )

        request_pose = getattr(node, "_request_current_pose_snapshot", None)
        if not callable(request_pose):
            return ToolResult(
                ok=False,
                error="get_current_pose async client is not exposed to ReAct tools",
            )

        reference_frame = str(args.get("reference_frame") or "base_link")
        pose = request_pose(reference_frame)
        if pose is None:
            return ToolResult(ok=False, error="get_current_pose unavailable")
        return ToolResult(
            ok=True,
            payload={
                "pose": {
                    "header": {"frame_id": reference_frame},
                    "pose": pose,
                }
            },
        )


"""Gripper close tool — stub until gripper capability is wired."""


class GripperCloseTool(Tool):
    name = "gripper_close"
    description = "Close the robot gripper."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "force": {"type": "number", "description": "Optional closing force (N)."},
        },
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        return ToolResult(
            ok=False,
            error="capability_unavailable",
            payload={"capability": "gripper"},
        )


"""Gripper open tool — stub until gripper capability is wired."""


class GripperOpenTool(Tool):
    name = "gripper_open"
    description = "Open the robot gripper."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {},
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        return ToolResult(
            ok=False,
            error="capability_unavailable",
            payload={"capability": "gripper"},
        )


"""Plan motion tool — validates a motion target through /validate_command."""


class PlanMotionTool(Tool):
    name = "plan_motion"
    description = (
        "Validate a planned motion target (PoseStamped or joint positions). "
        "Returns a plan_id if valid. Does NOT execute."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "target": {
                "oneOf": [
                    {
                        "type": "object",
                        "required": ["header", "pose"],
                        "properties": {
                            "header": {"type": "object"},
                            "pose": {"type": "object"},
                        },
                    },
                    {
                        "type": "object",
                        "required": ["joint_target"],
                        "properties": {
                            "joint_target": {
                                "type": "array",
                                "items": {"type": "number"},
                            },
                        },
                    },
                ],
            },
            "planner": {"type": "string"},
            "velocity_scale": {"type": "number", "minimum": 0.0, "maximum": 1.0},
            "acceleration_scale": {"type": "number", "minimum": 0.0, "maximum": 1.0},
        },
        "required": ["target"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = getattr(context, "ros_node", None)
        if node is None:
            return ToolResult(
                ok=False,
                error="ros_node not available in AgentContext",
            )

        client = getattr(node, "_validate_client", None)
        if client is None or not client.service_is_ready():
            return ToolResult(
                ok=False,
                error="validate_command service not available",
            )

        # Build a command dict for the existing safety validation service.
        target = dict(args["target"])
        default_velocity_scale = self._default_velocity_scale(context, node)
        command = {
            "primitive_type": "PTP" if "joint_target" in target else "LIN",
            "velocity_scale": float(args.get("velocity_scale", default_velocity_scale)),
            "acceleration_scale": float(
                args.get(
                    "acceleration_scale",
                    getattr(
                        node, "_default_acceleration_scale", default_velocity_scale
                    ),
                )
            ),
        }
        if "joint_target" in target:
            command["joint_target"] = target.get("joint_target")
        else:
            command["target_pose"] = target.get("pose", target)
        if args.get("planner"):
            command["planner_id"] = str(args["planner"])

        normalized_command = None
        command_payload = command
        if all(
            hasattr(node, attr)
            for attr in (
                "_normalize_and_validate",
                "_goal_mapper",
                "_build_validate_request",
            )
        ):
            try:
                normalized_command = node._normalize_and_validate(command)
                command_payload = node._goal_mapper.to_command_payload(
                    normalized_command
                )
                req = node._build_validate_request(normalized_command, command_payload)
            except Exception as exc:
                return ToolResult(
                    ok=False,
                    error=f"plan_motion normalization failed: {exc}",
                )
        else:
            req = node._validate_client.RequestType()
            req.command_json = json.dumps(
                command_payload, ensure_ascii=True, separators=(",", ":")
            )
            req.primitive_type = command["primitive_type"]
            req.velocity_scale = command["velocity_scale"]
        future = client.call_async(req)
        done, resp = self._wait_for_validation_response(node, future)
        if not done:
            return ToolResult(ok=False, error="validate_command timeout")

        if resp is None or not getattr(resp, "valid", False):
            return ToolResult(
                ok=False,
                error=getattr(resp, "reason", "")
                or "validate_command rejected the plan",
            )

        plan_id = f"plan-{uuid4().hex}"
        plan_cache = getattr(node, "_react_plan_cache", None)
        if isinstance(plan_cache, dict):
            plan_cache[plan_id] = self._build_plan_record(
                node=node,
                command=command,
                command_payload=command_payload,
                response=resp,
                normalized_command=normalized_command,
            )
            max_entries = int(getattr(node, "_react_plan_cache_max_entries", 64))
            while len(plan_cache) > max_entries:
                oldest_plan_id = next(iter(plan_cache))
                plan_cache.pop(oldest_plan_id, None)

        return ToolResult(
            ok=True,
            payload={
                "plan_id": plan_id,
                "valid": True,
            },
        )

    def _default_velocity_scale(self, context: "AgentContext", node) -> float:
        if hasattr(node, "_default_velocity_scale"):
            return float(node._default_velocity_scale)
        snapshot = context.state_injector.snapshot()
        return float(snapshot["robot_state"]["velocity_scale_active"])

    def _wait_for_validation_response(self, node, future):
        wait_for_future = getattr(node, "_wait_for_future_without_spinning", None)
        timeout_sec = getattr(node, "_safety_service_timeout_sec", None)
        if callable(wait_for_future) and timeout_sec is not None:
            return wait_for_future(future, float(timeout_sec))
        done = getattr(future, "done", None)
        if callable(done) and done():
            return True, future.result()
        return False, None

    def _build_plan_record(
        self,
        *,
        node,
        command: dict,
        command_payload: dict,
        response,
        normalized_command,
    ) -> dict:
        if all(
            hasattr(node, attr)
            for attr in (
                "_command_from_sanitized_json",
                "_normalize_and_validate",
                "_prepare_execution_command",
                "_goal_mapper",
            )
        ):
            validated_command = node._command_from_sanitized_json(
                getattr(response, "sanitized_json", ""),
                command_payload,
            )
            normalized_validated = node._normalize_and_validate(validated_command)
            execution_command = node._prepare_execution_command(normalized_validated)
            return {
                "command": execution_command,
                "goal": node._goal_mapper.to_execute_motion_goal(execution_command),
                "command_payload": node._goal_mapper.to_command_payload(
                    execution_command
                ),
            }
        return {"command": normalized_command or command}


"""Perception query tool — delegates to gp4_perception.query_perception_tool (W4)."""


# Attempt W4 import; fall back to stub error if gp4_perception is not built/installed.
try:
    from gp4_perception.query_perception_tool import (
        query_perception,
        _format_detections_from_ros,
    )

    _W4_AVAILABLE = True  # type: ignore[var-annotated]
except Exception:
    _W4_AVAILABLE = False
    _format_detections_from_ros = None  # type: ignore[assignment]


class QueryPerceptionTool(Tool):
    name = "query_perception"
    description = "Query the perception system for object detections."
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "class_filter": {
                "type": "string",
                "description": "Optional object class to filter.",
            },
        },
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        snapshot = context.state_injector.snapshot()
        mode = snapshot.get("robot_state", {}).get("mode", "IDLE")
        if mode != "IDLE":
            return ToolResult(
                ok=False,
                error=f"perception_blocked_during_motion (mode={mode})",
            )

        node = getattr(context, "ros_node", None)
        live_query = getattr(node, "_query_perception_detections", None)
        if callable(live_query):
            result = live_query(args)
            # Enrich with human-readable descriptions when formatter is available
            # and detections are ROS message objects (have .results attribute).
            # Plain dicts from the live query are already in dict form.
            raw_detections = (result.get("payload") or {}).get("detections", [])
            are_ros_msgs = (
                raw_detections
                and _format_detections_from_ros is not None
                and not isinstance(raw_detections[0], dict)
            )
            if are_ros_msgs:
                try:
                    formatted = _format_detections_from_ros(raw_detections)
                    class_filter = args.get("class_filter", "").strip().lower()
                    if class_filter:
                        formatted = [
                            d
                            for d in formatted
                            if class_filter in d.get("class_id", "").lower()
                            or class_filter in d.get("description", "").lower()
                        ]
                    summary_parts = [d["description"] for d in formatted]
                    result["payload"] = {
                        "detections": formatted,
                        "count": len(formatted),
                        "summary": "; ".join(summary_parts)
                        if summary_parts
                        else "No objects detected.",
                    }
                except Exception:
                    pass  # Fallback to raw detections.
            return ToolResult(
                ok=bool(result.get("ok")),
                error=result.get("error"),
                payload=result.get("payload"),
            )

        if not _W4_AVAILABLE:
            return ToolResult(
                ok=False,
                error="perception_not_available",
                payload={"hint": "gp4_perception package is not installed or built"},
            )
        result = query_perception(
            args=args,
            context_state=snapshot,
        )
        return ToolResult(
            ok=result["ok"],
            error=result.get("error"),
            payload=result.get("payload"),
        )


"""Set speed tool — validates and records velocity_scale for future motion."""


class SetSpeedTool(Tool):
    name = "set_speed"
    description = (
        "Set the global velocity scale (0.0–1.0). Affects future motion commands."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "velocity_scale": {
                "type": "number",
                "minimum": 0.0,
                "maximum": 1.0,
            },
        },
        "required": ["velocity_scale"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        velocity_scale = float(args["velocity_scale"])
        if not (0.0 <= velocity_scale <= 1.0):
            return ToolResult(
                ok=False,
                error=f"velocity_scale {velocity_scale} out of range [0.0, 1.0]",
            )
        context.state_injector.set_velocity_scale(velocity_scale)
        return ToolResult(
            ok=True,
            payload={"applied": True, "velocity_scale": velocity_scale},
        )


"""Submit motion tool — hands a plan to HMI/operator confirmation."""


class SubmitMotionTool(Tool):
    name = "submit_motion"
    description = (
        "Prepare a previously planned motion for HMI/operator confirmation "
        "without executing it. "
        "Requires a plan_id returned by plan_motion."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "plan_id": {"type": "string"},
        },
        "required": ["plan_id"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = getattr(context, "ros_node", None)
        if node is None:
            return ToolResult(
                ok=False,
                error="ros_node not available in AgentContext",
            )

        action_client = getattr(node, "_execute_client", None)
        if action_client is None:
            return ToolResult(
                ok=False,
                error="execute_motion action client not available",
            )
        server_is_ready = getattr(action_client, "server_is_ready", None)
        if callable(server_is_ready) and not server_is_ready():
            return ToolResult(
                ok=False,
                error="execute_motion action server unavailable",
            )

        # In a real implementation we would look up the stored plan by plan_id.
        # For W3, we assume the node has a plan cache populated by plan_motion.
        plan_cache = getattr(node, "_react_plan_cache", {})
        stored = plan_cache.get(args["plan_id"])
        if stored is None:
            return ToolResult(
                ok=False,
                error=f"Unknown plan_id: {args['plan_id']}",
            )

        command = stored.get("command", stored) if isinstance(stored, dict) else {}
        return ToolResult(
            ok=True,
            payload={
                "status": "READY_FOR_CONFIRM",
                "plan_id": args["plan_id"],
                "command": command,
            },
        )


"""Wait for robot state tool — polls robot status topic."""


class WaitForStateTool(Tool):
    name = "wait_for_state"
    description = (
        "Wait until the robot reaches a given state (IDLE, MOVING, PLANNING, FAULT)."
    )
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "state": {
                "type": "string",
                "enum": ["IDLE", "MOVING", "PLANNING", "FAULT"],
            },
            "timeout_s": {
                "type": "number",
                "minimum": 0.0,
                "maximum": 60.0,
            },
        },
        "required": ["state", "timeout_s"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        target = str(args["state"])
        timeout_s = float(args["timeout_s"])
        # Single-shot snapshot check.  Full polling with blocking wait
        # belongs in the ROS executor thread; tools must not time.sleep.
        snapshot = context.state_injector.snapshot()
        current = snapshot.get("robot_state", {}).get("mode", "IDLE")
        if current == target:
            return ToolResult(
                ok=True,
                payload={"reached": True, "current_state": current, "elapsed_s": 0.0},
            )
        return ToolResult(
            ok=False,
            error="state_not_reached",
            payload={
                "reached": False,
                "current_state": current,
                "timeout_s": timeout_s,
            },
        )


"""ReAct loop driver — reasoning + tool use for LLM intent resolution."""

import logging
import time
from dataclasses import dataclass
from typing import Tuple


from llm_gateway.intent_engine import LLMParser

_LOGGER = logging.getLogger(__name__)

_REACT_SYSTEM_PROMPT_PREFIX = (
    "You are a robot task planner for a Yaskawa GP4 6-axis industrial robot arm.",
    "You DO NOT control the robot directly.",
    "You produce Semantic IR JSON that the safety system reviews before any motion is executed.",
    "",
    "## Reasoning Rules",
    "1. Think step by step. For multi-step tasks, break them into individual motion steps.",
    "2. Use tools to gather information BEFORE producing a plan.",
    "   - Use get_current_pose to know where the robot is now.",
    "   - Use query_perception to find objects in the workspace.",
    "   - Use compute_arc_points for circular motions.",
    "   - Use plan_motion to validate a target BEFORE submitting.",
    "3. If a user mentions an object by color or shape (e.g. 'red sphere', 'blue box'),",
    "   FIRST call query_perception to locate it, then plan motion to its position.",
    "4. For 'draw circle' or 'arc' commands, use compute_arc_points to calculate waypoints.",
    "5. Never guess coordinates. Always query the current state first.",
    "",
    "## Safety Rules",
    "- NEVER exceed velocity_scale 0.10 unless explicitly instructed.",
    "- NEVER produce raw joint trajectories — only Semantic IR with intent.",
    "- If a command is ambiguous or unsafe, respond with an error intent explaining why.",
    "- Joints 4, 5, 6 are prone to singularity near zero; avoid planning through those configs.",
    "",
    "## Available Tools",
)

_REACT_SYSTEM_PROMPT_SUFFIX = (
    "",
    "## Output Format",
    "When you have enough information, respond WITHOUT a tool call, with one Semantic IR JSON object.",
    'The final JSON must include an "intent" field and must not include "primitive_type".',
    'Multi-step requests: {"intent":"sequence","steps":[<semantic_ir_step>, ...]}.',
    "Each sequence step is also Semantic IR with its own intent.",
    'Error responses: {"intent":"error","error":"<reason>"}.',
)


@dataclass
class ToolCall:
    name: str
    args: dict


@dataclass
class AgentContext:
    """Shared context passed to tool invocations."""

    state_injector: StateInjector
    ros_node: Any = None


class ReActAgent:
    """ReAct agent: iteratively calls tools until a valid semantic IR is produced."""

    def __init__(
        self,
        llm_client,
        tool_registry: ToolRegistry,
        state_injector: StateInjector,
        budget: IterationBudget,
        schema_validator,
        ros_node: Any = None,
        payload_parser: LLMParser | None = None,
    ):
        self._llm_client = llm_client
        self._tool_registry = tool_registry
        self._state_injector = state_injector
        self._budget = budget
        self._schema_validator = schema_validator
        self._ros_node = ros_node
        self._payload_parser = payload_parser or LLMParser()

    def run(self, user_text: str) -> dict:
        """Run the ReAct loop and return final structured command (semantic IR)."""
        state = self._state_injector.snapshot()
        history: List[Tuple[str, Any]] = []
        counters = IterationCounters()
        start_time = time.monotonic()
        context = AgentContext(
            state_injector=self._state_injector,
            ros_node=self._ros_node,
        )

        while True:
            allowed, reason = counters.can_invoke_any(self._budget)
            if not allowed:
                return self._handoff(reason, history)

            messages = self._build_prompt(user_text, state, history)
            try:
                llm_response = self._llm_client.generate_response_from_messages(
                    messages
                )
            except Exception as exc:
                return self._handoff(f"llm_request_failed: {exc}", history)

            decoded_response = self._decode_llm_response(llm_response)
            tool_call = self._parse_tool_call(decoded_response)
            if tool_call is None:
                semantic_ir = self._extract_semantic_ir(decoded_response)
                ok, err = self._validate_semantic_ir(semantic_ir)
                if not ok:
                    if counters.repair < self._budget.max_repair:
                        counters.repair += 1
                        history.append(("observation", f"validation_error: {err}"))
                        continue
                    return self._handoff(
                        f"semantic_ir invalid after repair: {err}", history
                    )
                return semantic_ir

            tool = self._tool_registry.get(tool_call.name)
            if tool is None:
                history.append(("observation", f"unknown_tool: {tool_call.name}"))
                continue

            allowed, reason = counters.can_invoke(tool, self._budget)
            if not allowed:
                return self._handoff(
                    f"budget exceeded for {tool.name}: {reason}", history
                )

            try:
                tool.validate_input(tool_call.args)
            except jsonschema.ValidationError as exc:
                history.append(("observation", f"tool_input_invalid: {exc.message}"))
                counters.repair += 1
                continue
            except Exception as exc:
                history.append(("observation", f"tool_input_invalid: {exc}"))
                counters.repair += 1
                continue

            try:
                result = tool.invoke(tool_call.args, context)
            except Exception as exc:
                result = ToolResult(ok=False, error=str(exc))

            counters.record(tool)
            state = self._state_injector.snapshot()
            history.append(("tool_call", tool_call))
            history.append(("observation", result.to_observation()))

            elapsed = time.monotonic() - start_time
            if elapsed > self._budget.wall_clock_timeout_s:
                return self._handoff("wall_clock_timeout", history)

    def _build_prompt(
        self,
        user_text: str,
        state: Dict[str, Any],
        history: List[Tuple[str, Any]],
    ) -> List[Dict[str, str]]:
        system_lines = [
            *_REACT_SYSTEM_PROMPT_PREFIX,
            self._tool_registry.available_tools_description(),
            "",
            "## Current Robot State",
            json.dumps(state, indent=2),
            *_REACT_SYSTEM_PROMPT_SUFFIX,
        ]
        messages: List[Dict[str, str]] = [
            {"role": "system", "content": "\n".join(system_lines)},
            {"role": "user", "content": user_text.strip()},
        ]
        for role, content in history:
            messages.append(self._history_message(role, content))
        return messages

    def _history_message(self, role: str, content: Any) -> Dict[str, str]:
        if isinstance(content, ToolCall):
            return {
                "role": "assistant",
                "content": json.dumps({"tool_call": content.name, "args": content.args}),
            }
        return {"role": "user", "content": str(content)}

    def _decode_llm_response(self, llm_response: str) -> Any:
        """Decode direct model JSON or provider-wrapped JSON into a payload."""
        try:
            data = json.loads(llm_response)
        except json.JSONDecodeError:
            return llm_response

        if not isinstance(data, dict):
            return data

        if any(
            key in data for key in ("tool_call", "intent", "primitive_type", "error")
        ):
            return data

        tool_call = self._extract_provider_tool_call(data)
        if tool_call is not None:
            return tool_call

        try:
            return self._payload_parser.parse(llm_response)
        except Exception:
            return data

    def _extract_provider_tool_call(self, data: dict) -> dict | None:
        """Unwrap provider-native tool calls into the ReAct tool-call shape."""
        direct = self._decode_provider_call(data.get("function_call"))
        if direct is not None:
            return direct

        direct = self._decode_first_tool_call(data.get("tool_calls"))
        if direct is not None:
            return direct

        choices = data.get("choices")
        if isinstance(choices, list) and choices:
            first_choice = choices[0]
            if isinstance(first_choice, dict):
                message = first_choice.get("message")
                if isinstance(message, dict):
                    message_call = self._decode_provider_call(
                        message.get("function_call")
                    )
                    if message_call is not None:
                        return message_call
                    message_call = self._decode_first_tool_call(
                        message.get("tool_calls")
                    )
                    if message_call is not None:
                        return message_call

        output = data.get("output")
        if isinstance(output, list):
            for block in output:
                if isinstance(block, dict) and block.get("type") == "function_call":
                    response_call = self._decode_provider_call(block)
                    if response_call is not None:
                        return response_call

        content = data.get("content")
        if isinstance(content, list):
            for block in content:
                if isinstance(block, dict) and block.get("type") == "tool_use":
                    anthropic_call = self._decode_provider_call(
                        {"name": block.get("name"), "arguments": block.get("input")}
                    )
                    if anthropic_call is not None:
                        return anthropic_call

        return None

    def _decode_first_tool_call(self, tool_calls: Any) -> dict | None:
        if not isinstance(tool_calls, list) or not tool_calls:
            return None
        first_call = tool_calls[0]
        if not isinstance(first_call, dict):
            return None
        function_data = first_call.get("function")
        if isinstance(function_data, dict):
            decoded = self._decode_provider_call(function_data)
            if decoded is not None:
                return decoded
        return self._decode_provider_call(first_call)

    def _decode_provider_call(self, call: Any) -> dict | None:
        if not isinstance(call, dict):
            return None
        name = call.get("name")
        if not isinstance(name, str) or not name:
            return None
        arguments = call.get("arguments", {})
        if isinstance(arguments, str):
            try:
                arguments = json.loads(arguments)
            except json.JSONDecodeError:
                return None
        if not isinstance(arguments, dict):
            return None
        return {"tool_call": name, "args": arguments}

    def _parse_tool_call(self, llm_response: Any) -> Optional[ToolCall]:
        """Parse an LLM response for a tool call.

        Returns None if the response is a final command (no tool call).
        """
        if not isinstance(llm_response, dict):
            return None

        if "tool_call" in llm_response:
            name = llm_response.get("tool_call", "")
            args = llm_response.get("args", {})
            if name and isinstance(name, str):
                return ToolCall(name=name, args=args)
        return None

    def _extract_semantic_ir(self, llm_response: Any) -> dict:
        """Extract the final semantic IR from LLM response."""
        if isinstance(llm_response, dict):
            return llm_response
        if isinstance(llm_response, str):
            return {"intent": "raw_text", "text": llm_response.strip()}
        return {"intent": "raw_text", "text": str(llm_response)}

    def _validate_semantic_ir(self, semantic_ir: dict) -> Tuple[bool, str]:
        if "intent" in semantic_ir and "primitive_type" not in semantic_ir:
            return True, ""
        if "primitive_type" in semantic_ir:
            return False, "semantic IR must use intent, not primitive_type"
        if "error" in semantic_ir:
            return True, ""
        return False, "semantic IR requires an intent field"

    def _handoff(self, reason: str, history: List[Tuple[str, Any]]) -> dict:
        def _serialize(content: Any) -> str:
            if isinstance(content, str):
                return content
            if isinstance(content, ToolCall):
                return json.dumps({"tool_call": content.name, "args": content.args})
            return json.dumps(content)

        return {
            "_handoff": True,
            "reason": reason,
            "history": [{"role": r, "content": _serialize(c)} for r, c in history],
        }
