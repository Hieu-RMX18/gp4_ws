"""Single-shot LLM→FactoryTask planner without ReAct loop.

Phase 2 of llm_gateway pipeline refactor (spec: docs/superpowers/specs/
2026-06-10-llm-gateway-factory-pipeline-design.md §3). This module replaces
the multi-iteration ReAct agent with a single-shot planner: one LLM call
produces a FactoryTask or error dict, no tool loop.

IMPORTANT: This module does NOT import rclpy or any ROS execution layer.
It uses ament_index_python only for finding installed package config paths.
"""

from __future__ import annotations

import json
import logging
import os
import random
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

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


# ── Frozen semantic intent list ─────────────────────────────────────────────
# Must match IntentRouter._route_single_intent exactly.
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
        "move_joint_delta",
        "move_joints",
        "io_set",
        "draw_shape",
        "draw_text",
        "return_to_start",
    }
)
FROZEN_TOP_LEVEL_OUTPUT_INTENTS = FROZEN_SEMANTIC_INTENTS | {"sequence"}


# ── Workspace bounds and system prompt ──────────────────────────────────────

_DEFAULT_WORKSPACE_BOUNDS = {
    "x_min": -0.45,
    "x_max": 0.45,
    "y_min": -0.16,
    "y_max": 0.52,
    "z_min": 0.15,
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
You are the llm_gateway task planner for a Yaskawa GP4 robot arm.
Your job: convert one natural-language command or ordered factory task into ONE FactoryTask JSON object.
You support English and Vietnamese (tiếng Việt). Classify by meaning, not exact words.

The FactoryTask does not execute motion. The gateway reviews it into a FactoryTask runtime payload; TaskRuntime later resolves each skill into guarded internal command artifacts behind supervisor validation, collision checks, runtime freshness checks, operator confirmation, and the hardware execution gate.

══════════════════════════════════════════════════════
OUTPUT FORMAT — always ONE JSON object, no markdown, no explanation:

A) FactoryTask normal path:
{
  "task_type": "factory_task",
  "version": "1.0",
  "task_id": "short_stable_id",
  "mode": "supervised_hardware",
  "operator_summary": "short operator-facing summary",
  "limits": {"velocity_scale": 0.06, "acceleration_scale": 0.06},
  "replan_policy": {"max_replans": 1, "on_world_change": "replan_before_motion"},
  "root": {"type": "skill", "name": "go_home", "args": {}}
}

B) Missing parameter — ask user instead of guessing:
{"error": "MISSING_SLOT", "missing_fields": ["field"], "hint": "question for the operator"}

C) Unsupported, unsafe, or ambiguous:
{"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND", "hint": "short reason"}

Do not output final Semantic IR. Do not output direct primitive commands or raw trajectories.

══════════════════════════════════════════════════════
FACTORY TASK NODE TYPES

sequence: ordered children. Use for multi-step work.
skill: a single guarded task skill with name and args.
repeat: runtime loop with positive count; never expand loops into long static sequences.
for_each: iterate over a WorldModel collection such as visible_objects.
until: repeat until a condition evaluator says done.
if: conditional branch evaluated by runtime policy.
retry: retry child nodes at runtime, preserving observations and failure reasons.
fallback: try alternate child branches after failure.
observe: read the WorldModel/perception state without motion.
wait_until: wait for a runtime predicate without blocking executor callbacks.

Allowed control nodes keep loop/retry/fallback/replan behavior visible to TaskRuntime and HMI. Prefer them over prebuilding many repeated steps.

══════════════════════════════════════════════════════
FACTORY TASK SKILLS

go_home args: {}
stop args: {}
alarm_reset args: {}
get_pose args: {"reference_frame": "base_link"}
set_speed args: {"velocity_scale": 0.01..0.06}
wait args: {"wait_duration_sec": float, default 2.0 when clearly requested}
move_relative args: {"delta": {"x": m, "y": m, "z": m}, "reference_frame": "base_link"}
move_named_pose args: {"pose_name": "home"|"ready"|"poseA"|"poseB"}
move_to_region args: {"region": "semantic_region_name", "approach": "safe_top"}
move_to_object args: {"object_ref": "world_model_object", "pose": "approach"}
move_cartesian args: {"target_pose": {"position": {"x": n, "y": n, "z": n}}, "reference_frame": "base_link", "keep_current_orientation": true, "orientation_preset": "tool-down"|"tool-forward"|"tool-up" when explicitly requested}
move_joint args: {"joint_index": 0..5, "joint_angle": float, "angular_unit": "rad"|"deg"}
move_joint_delta args: {"joint_index": 0..5, "delta_angle": float, "angular_unit": "rad"|"deg"}
move_joints args: {"joint_target": [six floats], "angular_unit": "rad"|"deg"}
pick_object args: {"object": "world_model_object"}
place_object args: {"object": "world_model_object", "destination": "region_or_pose"}
place_relative args: {"object": "world_model_object", "reference": "current_pose", "delta": {"x": m, "y": m, "z": m}}
verify_scene args: {"object": "world_model_object", "expected": "held|placed|visible"}
draw_shape args: {"shape_type": "circle|arc|square|rectangle|triangle|polygon|polyline", "units": "m|cm|mm", "frame_id": "base_link", "workplane": {"mode": "tool|base|explicit_pose"}, "params": {}}
draw_text args: {"text": "A-Z 0-9 text", "units": "m|cm|mm", "frame_id": "base_link", "workplane": {"mode": "tool|base|explicit_pose"}, "font": {"type": "single_stroke_builtin", "height": n}}

Unknown object poses, stale perception, missing calibration, missing frame, or unknown region: ALWAYS generate a FactoryTask with an observe step before the motion step. TaskRuntime will query perception and grounding at execution time. Only return MISSING_SLOT if the operator's command is fundamentally incomplete, such as "move" with no target. Never guess transforms, units, object locations, or robot state.

══════════════════════════════════════════════════════
GROUNDING AND SAFETY RULES

- The WorldModel owns object, region, collection, freshness, and calibration facts.
- TaskRuntime resolves skills at execution time; missing world facts fail closed there.
- PolicyEngine decisions must remain visible through task metadata and HMI planSummary.
- TaskRuntime owns loops, retry, fallback, and replan. Use replan_policy for tasks where objects can move.
- Motion remains behind supervisor validation, collision checking, operator confirmation, and execution gating.
- Hardware-adjacent velocity_scale and acceleration_scale must stay at or below 0.06 unless the safety rules are changed.

UNIT RULES
- Internal execution uses SI.
- Convert relative move distances to meters inside args.delta.
- For absolute/cartesian positions where the user explicitly gives non-SI linear units, keep the magnitude and include linear_unit: "cm" or "mm".
- For non-SI joint angles, keep the magnitude and include angular_unit: "deg".
- If user already speaks in meters/radians or gives no unit, use SI values directly.

WORKSPACE LIMITS (meters): __WORKSPACE_LIMITS__
UNIT CONVERSIONS: 1 phân = 1 cm = 0.01 m | 1 mm = 0.001 m
VELOCITY SCALE: 0.01 (slow) to 0.06 (fast)
ORIENTATION PRESETS: tool-down | tool-forward | tool-up
reference_frame is always "base_link" unless the user asks a read-only pose query in another frame.

══════════════════════════════════════════════════════
EXAMPLES

User: "về nhà"
→ {"task_type": "factory_task", "version": "1.0", "task_id": "go-home", "root": {"type": "skill", "name": "go_home", "args": {}}}

User: "move to pose A"
→ {"task_type": "factory_task", "version": "1.0", "task_id": "move-pose-a", "root": {"type": "skill", "name": "move_named_pose", "args": {"pose_name": "poseA"}}}

User: "go home then wait one second"
→ {"task_type": "factory_task", "version": "1.0", "task_id": "home-wait", "root": {"type": "sequence", "children": [{"type": "skill", "name": "go_home", "args": {}}, {"type": "skill", "name": "wait", "args": {"wait_duration_sec": 1.0}}]}}

User: "move to Cartesian x 300 mm y 0 z 400"
→ {"task_type": "factory_task", "version": "1.0", "task_id": "cartesian-move", "root": {"type": "skill", "name": "move_cartesian", "args": {"target_pose": {"position": {"x": 300.0, "y": 0.0, "z": 400.0}}, "linear_unit": "mm", "reference_frame": "base_link", "keep_current_orientation": true}}}

User: "nhặt quả táo, thả lên 10cm, kiểm tra rồi nhặt lại"
→ {"task_type": "factory_task", "version": "1.0", "task_id": "apple-drop-repick", "replan_policy": {"max_replans": 1, "on_world_change": "replan_before_motion"}, "root": {"type": "sequence", "children": [{"type": "skill", "name": "pick_object", "args": {"object": "apple"}}, {"type": "skill", "name": "place_relative", "args": {"object": "apple", "reference": "current_pose", "delta": {"x": 0.0, "y": 0.0, "z": 0.10}}}, {"type": "skill", "name": "verify_scene", "args": {"object": "apple", "expected": "visible"}}, {"type": "skill", "name": "pick_object", "args": {"object": "apple"}}]}}

User: "inspect every visible object"
→ {"task_type": "factory_task", "version": "1.0", "task_id": "inspect-visible", "root": {"type": "sequence", "children": [{"type": "observe", "name": "observe_station", "args": {"region": "station"}}, {"type": "for_each", "collection": "visible_objects", "item_name": "object", "children": [{"type": "skill", "name": "move_to_object", "args": {"object_ref": "$object", "pose": "approach"}}, {"type": "skill", "name": "verify_scene", "args": {"object": "$object", "expected": "visible"}}]}]}}

User: "try placing the apple twice, otherwise go home"
→ {"task_type": "factory_task", "version": "1.0", "task_id": "place-with-fallback", "root": {"type": "fallback", "children": [{"type": "retry", "count": 2, "children": [{"type": "skill", "name": "place_object", "args": {"object": "apple", "destination": "bin_a"}}]}, {"type": "skill", "name": "go_home", "args": {}}]}}

User: "hạ xuống một chút"
→ {"error": "MISSING_SLOT", "missing_fields": ["distance"], "hint": "relative move requires direction and distance."}

Schema reference for downstream validation and compatibility:
__JSON_SCHEMA__
"""


def build_system_prompt(schema_json: str) -> str:
    """Build the system prompt for the LLM, injecting the JSON schema."""
    workspace_limits = _format_workspace_bounds(_load_workspace_bounds())
    return _SYSTEM_PROMPT_TEMPLATE.replace(
        "__WORKSPACE_LIMITS__", workspace_limits
    ).replace("__JSON_SCHEMA__", schema_json)


# ── OpenAI-compatible client ────────────────────────────────────────────────

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

        assert last_exc is not None
        raise last_exc

    def generate_response_from_messages(self, messages: list[dict[str, str]]) -> str:
        """Send a request with pre-constructed messages."""
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
        exp = base * (2 ** (attempt - 1))
        capped = min(cap, exp)
        jitter = capped * 0.25
        return max(0.0, capped + self._rng.uniform(-jitter, jitter))

    def _build_messages(self, user_input: str) -> List[Dict[str, str]]:
        return [
            {"role": "system", "content": self._system_prompt},
            {"role": "user", "content": user_input.strip()},
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


# ── State snapshot ──────────────────────────────────────────────────────────

class StateInjector:
    """Maintains a planner-visible snapshot of robot state.

    The class is pure Python. ROS subscribers in llm_gateway_node feed it plain
    dictionaries; tests can update it directly.
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
        self._available_named_poses: List[str] = []
        self._semantic_map: Dict[str, Any] = {}

    def update_joint_states(self, msg: Dict[str, Any]) -> None:
        self._last_joint_states = msg

    def update_robot_status(self, msg: Dict[str, Any]) -> None:
        self._last_robot_status = msg

    def set_velocity_scale(self, value: float) -> None:
        self._velocity_scale_active = float(value)

    def set_capabilities(self, *, gripper: bool, perception: bool) -> None:
        self._gripper_available = gripper
        self._perception_available = perception

    def set_available_named_poses(self, poses: List[str]) -> None:
        self._available_named_poses = sorted(poses)

    def set_semantic_map(self, semantic_map: Dict[str, Any]) -> None:
        self._semantic_map = semantic_map

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
                "available_named_poses": list(self._available_named_poses),
            },
            "station_semantic_map": self._semantic_map,
        }


# ── Single-shot planner ─────────────────────────────────────────────────────

class TaskPlanner:
    """Single-shot natural language to FactoryTask planner."""

    def __init__(
        self,
        llm_client: Any,
        state_injector: StateInjector,
        schema_validator: Any,
        payload_parser: Any | None = None,
        max_repair: int = 1,
    ) -> None:
        from llm_gateway.intent_engine import LLMParser

        self._llm_client = llm_client
        self._state_injector = state_injector
        self._schema_validator = schema_validator
        self._payload_parser = payload_parser or LLMParser()
        self._max_repair = max(0, int(max_repair))
        self._system_prompt = self._resolve_system_prompt(schema_validator)

    def plan(self, user_text: str) -> Dict[str, Any]:
        """Return one FactoryTask JSON object or an explicit error payload."""
        if not isinstance(user_text, str) or not user_text.strip():
            return {
                "error": "MISSING_SLOT",
                "missing_fields": ["intent_text"],
                "hint": "Command text is required.",
            }

        messages = self._build_messages(user_text)
        last_error = "planner returned invalid JSON"
        for attempt in range(self._max_repair + 1):
            try:
                raw = self._call_llm(messages)
                parsed = self._payload_parser.parse(raw)
            except Exception as exc:
                last_error = str(exc)
                messages = self._repair_messages(messages, last_error)
                continue

            if self._is_accepted_payload(parsed):
                return parsed

            last_error = "LLM final payload must be FactoryTask or an error object."
            messages = self._repair_messages(messages, last_error)

        return {
            "error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND",
            "message": last_error,
            "hint": "Rephrase the command with clearer task steps or required parameters.",
        }

    def _call_llm(self, messages: list[dict[str, str]]) -> str:
        sender = getattr(self._llm_client, "generate_response_from_messages", None)
        if callable(sender):
            return str(sender(messages))
        fallback = getattr(self._llm_client, "generate_response", None)
        if callable(fallback):
            return str(fallback(messages[-1]["content"]))
        raise TypeError("llm_client must provide generate_response_from_messages or generate_response")

    def _build_messages(self, user_text: str) -> list[dict[str, str]]:
        state_context = json.dumps(
            self._state_injector.snapshot(), ensure_ascii=False, sort_keys=True
        )
        return [
            {"role": "system", "content": self._system_prompt},
            {
                "role": "user",
                "content": (
                    "Current robot/world state JSON:\n"
                    f"{state_context}\n\n"
                    "Operator command:\n"
                    f"{user_text.strip()}"
                ),
            },
        ]

    def _repair_messages(
        self, messages: list[dict[str, str]], reason: str
    ) -> list[dict[str, str]]:
        repaired = list(messages)
        repaired.append(
            {
                "role": "user",
                "content": (
                    "Repair the previous response. Return exactly one valid "
                    "FactoryTask JSON object or one explicit error JSON object. "
                    f"Reason: {reason}"
                ),
            }
        )
        return repaired

    def _resolve_system_prompt(self, schema_validator: Any) -> str:
        schema_json = self._schema_json(schema_validator)
        client_prompt = getattr(self._llm_client, "_system_prompt", None)
        if isinstance(client_prompt, str) and client_prompt.strip():
            return client_prompt
        return build_system_prompt(schema_json)

    @staticmethod
    def _schema_json(schema_validator: Any) -> str:
        for attr in ("schema_json", "_schema_json"):
            value = getattr(schema_validator, attr, None)
            if isinstance(value, str) and value.strip():
                return value
        schema = getattr(schema_validator, "schema", None)
        if isinstance(schema, dict):
            return json.dumps(schema, ensure_ascii=False)
        return "{}"

    @staticmethod
    def _is_accepted_payload(payload: Any) -> bool:
        from llm_gateway.factory_task import is_factory_task

        return bool(
            isinstance(payload, dict)
            and (is_factory_task(payload) or isinstance(payload.get("error"), str))
        )
