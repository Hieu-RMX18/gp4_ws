#!/usr/bin/env python3
"""GP4 Real-Time Pipeline Logger — continuous streaming diagnostic console.

Subscribes to every ROS2 topic in the GP4 pipeline and prints a continuous,
real-time, line-by-line log of everything happening — from the moment a
prompt is received through LLM reasoning, safety validation, MoveIt planning,
trajectory dispatch, and physical robot execution.

This console NEVER clears the screen. It streams like `ros2 topic echo`.

Usage:
  source /opt/ros/humble/setup.bash
  source ~/gp4_ws/install/setup.bash
  python3 tools/runtime_console.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from typing import Any

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from industrial_msgs.msg import RobotStatus
    from diagnostic_msgs.msg import DiagnosticStatus
    from action_msgs.msg import GoalStatusArray
    from rcl_interfaces.msg import Log
except Exception as exc:
    print(f"[FATAL] ROS2 environment not sourced: {exc}")
    sys.exit(1)

# Try to import ExecuteMotion feedback for trajectory progress
try:
    from interfaces.action import ExecuteMotion
    _HAS_EXECUTE_MOTION = True
except Exception:
    _HAS_EXECUTE_MOTION = False

# ── ANSI colours ──────────────────────────────────────────────────────
R = "\033[0m"     # reset
B = "\033[1m"     # bold
DIM = "\033[2m"
RED = "\033[31m"
GRN = "\033[32m"
YLW = "\033[33m"
BLU = "\033[34m"
MAG = "\033[35m"
CYN = "\033[36m"
WHT = "\033[37m"
BG_RED = "\033[41m"
BG_GRN = "\033[42m"
BG_BLU = "\033[44m"
BG_YLW = "\033[43m"
BG_MAG = "\033[45m"
BG_CYN = "\033[46m"

# ── Icons for each layer ─────────────────────────────────────────────
ICON = {
    "prompt":      "📝",
    "llm":         "🧠",
    "react":       "🔄",
    "parse":       "📋",
    "validate":    "🛡️",
    "safety":      "🔒",
    "plan":        "📐",
    "execute":     "⚡",
    "dispatch":    "🚀",
    "motion":      "🤖",
    "joint":       "🔧",
    "pose":        "📍",
    "vision":      "👁️",
    "error":       "❌",
    "success":     "✅",
    "warn":        "⚠️",
    "info":        "ℹ️",
    "supervisor":  "👮",
    "status":      "📊",
    "sequence":    "📦",
    "feedback":    "📶",
    "time":        "⏱️",
}


def _ts() -> str:
    """Current timestamp with milliseconds."""
    now = time.time()
    t = time.strftime("%H:%M:%S", time.localtime(now))
    ms = int((now % 1) * 1000)
    return f"{DIM}{t}.{ms:03d}{R}"


def _deg(rad: float) -> str:
    return f"{math.degrees(rad):.2f}°"


def _mm(m: float) -> str:
    return f"{m*1000:.1f}mm"


def _quat_to_rpy(qx, qy, qz, qw):
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _log(icon: str, color: str, tag: str, msg: str, detail: str = ""):
    """Print one continuous log line. Never clears screen."""
    line = f"{_ts()} {icon} {color}{B}[{tag}]{R} {msg}"
    print(line, flush=True)
    if detail:
        # indent detail lines under the main line
        for dl in detail.strip().split("\n"):
            print(f"                    {DIM}  └─ {dl}{R}", flush=True)


def _log_kv(icon: str, color: str, tag: str, msg: str, kvs: dict):
    """Log with key-value pairs on subsequent lines."""
    _log(icon, color, tag, msg)
    for k, v in kvs.items():
        if v is not None and v != "" and v != {} and v != []:
            val_str = str(v)
            if len(val_str) > 200:
                val_str = val_str[:200] + "..."
            print(f"                    {DIM}  │ {CYN}{k}{R}: {val_str}", flush=True)


class GP4StreamLogger(Node):
    """Real-time streaming pipeline logger for GP4."""

    def __init__(self):
        super().__init__("gp4_stream_logger")

        # State tracking
        self._cmd_start_time: float = 0.0
        self._last_pose: PoseStamped | None = None
        self._last_joints: dict[str, float] = {}
        self._robot_mode = "UNKNOWN"
        self._in_error = False
        self._pose_print_counter = 0
        self._joint_print_counter = 0

        # ── Subscribe to EVERYTHING ──────────────────────────────────
        # 1) LLM debug channel — main pipeline trace bus
        self.create_subscription(String, "/llm_debug", self._on_llm_debug, 10)

        # 2) Gateway status transitions
        self.create_subscription(String, "/gateway_status", self._on_gateway_status, 10)

        # 3) Robot hardware status
        self.create_subscription(RobotStatus, "/yaskawa/robot_status",
                                 self._on_robot_status, qos_profile_sensor_data)

        # 4) Live TCP pose
        self.create_subscription(PoseStamped, "/yaskawa/current_pose",
                                 self._on_pose, qos_profile_sensor_data)

        # 5) Live joint states
        self.create_subscription(JointState, "/yaskawa/joint_states",
                                 self._on_joints, qos_profile_sensor_data)

        # 6) Supervisor alerts
        self.create_subscription(DiagnosticStatus, "/supervisor/alerts",
                                 self._on_alert, 10)

        # 7) ExecuteMotion action goal status
        self.create_subscription(GoalStatusArray,
                                 "/execute_motion/_action/status",
                                 self._on_goal_status, 10)

        # 8) ExecuteMotion action feedback (trajectory progress)
        if _HAS_EXECUTE_MOTION:
            from rclpy.action import ActionClient
            # Subscribe to feedback topic directly
            self.create_subscription(
                ExecuteMotion.Impl.FeedbackMessage,
                "/execute_motion/_action/feedback",
                self._on_motion_feedback, 10)

        # 9) /rosout — captures ALL internal C++ and Python logs
        #    This gives us IK solve results, FK, planner selection,
        #    waypoints count, ruckig/totg, trajectory points, etc.
        #    from motion_core, safety, llm_gateway without modifying C++.
        self._ROSOUT_NODES = {
            "motion_core_node", "safety_node", "llm_gateway_node",
            "hw_adapter_node", "supervisor_node",
        }
        self.create_subscription(Log, "/rosout", self._on_rosout, 10)

        # 10) Periodic heartbeat — shows the console is alive
        self._heartbeat_timer = self.create_timer(10.0, self._heartbeat)

        self._print_banner()

    # ── Banner ────────────────────────────────────────────────────────
    def _print_banner(self):
        print()
        print(f"{B}{BG_BLU}{WHT} ╔══════════════════════════════════════════════════════════════╗ {R}")
        print(f"{B}{BG_BLU}{WHT} ║       GP4 REAL-TIME PIPELINE LOGGER — STREAMING MODE        ║ {R}")
        print(f"{B}{BG_BLU}{WHT} ╚══════════════════════════════════════════════════════════════╝ {R}")
        print(f"  {DIM}ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID', '0')} | Ctrl+C to stop{R}")
        print(f"  {DIM}Listening: /llm_debug /gateway_status /yaskawa/* /supervisor/alerts /rosout{R}")
        print(f"  {DIM}IK/FK/Planner/Ruckig/Waypoints from /rosout (motion_core internal logs){R}")
        print(f"  {DIM}Logs stream continuously below ↓{R}")
        print(f"{DIM}{'─' * 72}{R}")
        print(flush=True)

    def _heartbeat(self):
        mode_c = GRN if self._robot_mode == "READY" else (RED if self._in_error else YLW)
        _log(ICON["status"], DIM, "HEARTBEAT",
             f"Console alive | Robot: {mode_c}{self._robot_mode}{R} | "
             f"Pose: {'OK' if self._last_pose else 'N/A'} | "
             f"Joints: {len(self._last_joints)} axes")

    # ══════════════════════════════════════════════════════════════════
    # 1) /llm_debug — THE MAIN PIPELINE EVENT BUS
    # ══════════════════════════════════════════════════════════════════
    def _on_llm_debug(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            _log(ICON["info"], DIM, "DEBUG-RAW", msg.data[:120])
            return

        # Route based on message type
        if data.get("t") == "command_trace":
            self._handle_trace(data)
        elif "status" in data and "stage" in data:
            self._handle_debug_payload(data)
        else:
            _log(ICON["info"], CYN, "DEBUG", json.dumps(data)[:160])

    def _handle_trace(self, t: dict):
        """Handle structured command_trace events — THE core diagnostic feed."""
        event = t.get("event", "")
        layer = t.get("layer", "unknown")
        phase = t.get("phase", "")
        level = t.get("level", "INFO")
        summary = t.get("summary", "")
        details = t.get("details") or {}
        source = t.get("source", "")
        cmd_id = t.get("cmd_id", "")[:8]

        # Pick colour/icon by layer+event
        if level == "ERROR":
            color, icon = RED, ICON["error"]
        elif level == "WARN":
            color, icon = YLW, ICON["warn"]
        elif layer == "llm_gateway":
            color, icon = YLW, ICON["llm"]
        elif layer == "safety":
            color, icon = MAG, ICON["safety"]
        elif layer == "motion_core":
            color, icon = BLU, ICON["motion"]
        elif layer == "supervisor":
            color, icon = CYN, ICON["supervisor"]
        else:
            color, icon = WHT, ICON["info"]

        # Specialize icon based on event
        if "react" in event:
            icon = ICON["react"]
        elif "prompt" in event:
            icon = ICON["prompt"]
            self._cmd_start_time = time.time()
        elif "parsed" in event or "parse" in event:
            icon = ICON["parse"]
        elif "valid" in event or "safety" in event:
            icon = ICON["validate"]
        elif "plan" in event:
            icon = ICON["plan"]
        elif "dispatch" in event:
            icon = ICON["dispatch"]
        elif "execution" in event or "goal" in event:
            icon = ICON["execute"]

        # Build tag
        tag = f"{layer}/{phase}" if phase else layer
        if cmd_id:
            tag = f"{tag} #{cmd_id}"

        # Build main message
        main_msg = f"{event}"
        if summary:
            main_msg += f" — {summary}"

        # Print the event
        _log(icon, color, tag, main_msg)

        # Print details
        if source:
            print(f"                    {DIM}  │ source: {source}{R}", flush=True)
        if details:
            for k, v in details.items():
                if v is not None and v != "":
                    val_s = str(v)
                    if len(val_s) > 300:
                        val_s = val_s[:300] + "..."
                    print(f"                    {DIM}  │ {CYN}{k}{R}: {val_s}", flush=True)

        # Print error context
        error_why = t.get("error_why") or details.get("error_why", "")
        error_where = t.get("error_where") or details.get("error_where", "")
        error_next = t.get("error_next_action") or details.get("error_next_action", "")
        if error_why:
            print(f"                    {RED}{B}  ├─ WHY: {error_why}{R}", flush=True)
        if error_where:
            print(f"                    {RED}{B}  ├─ WHERE: {error_where}{R}", flush=True)
        if error_next:
            print(f"                    {YLW}{B}  └─ FIX: {error_next}{R}", flush=True)

    def _handle_debug_payload(self, data: dict):
        """Handle legacy debug payloads from _publish_debug()."""
        status = data.get("status", "")
        stage = data.get("stage", "")
        intent = data.get("intent", "")

        if status == "rejected":
            reason = data.get("reason", "unknown")
            hint = data.get("hint", "")
            _log(ICON["error"], RED, f"REJECTED/{stage}",
                 f"{reason}")
            if intent:
                print(f"                    {DIM}  │ intent: {intent[:80]}{R}", flush=True)
            if data.get("raw_llm_output"):
                raw = str(data["raw_llm_output"])[:200]
                print(f"                    {DIM}  │ raw_llm: {raw}{R}", flush=True)
            if data.get("validated_command"):
                vc = json.dumps(data["validated_command"])[:200]
                print(f"                    {DIM}  │ command: {vc}{R}", flush=True)
            if hint:
                print(f"                    {YLW}  └─ HINT: {hint}{R}", flush=True)
            return

        if status == "validated":
            vc = data.get("validated_command", {})
            prim = vc.get("primitive_type", "?")
            vel = vc.get("velocity_scale", "?")
            acc = vc.get("acceleration_scale", "?")
            _log_kv(ICON["validate"], GRN, f"VALIDATED/{stage}",
                    f"Command approved → primitive={prim} vel={vel} acc={acc}",
                    {k: v for k, v in vc.items()
                     if k in ("target_pose", "delta_x", "delta_y", "delta_z",
                              "joint_target", "waypoints", "named_pose",
                              "reference_frame", "planner_id")})
            return

        if status == "succeeded":
            elapsed = time.time() - self._cmd_start_time if self._cmd_start_time else 0
            _log(ICON["success"], GRN, "SUCCEEDED",
                 f"{data.get('message', 'done')} | "
                 f"{ICON['time']} total={elapsed:.2f}s")
            print(f"{B}{GRN}{'━' * 72}{R}", flush=True)
            return

        if status == "sequence_valid":
            _log_kv(ICON["sequence"], MAG, "SEQUENCE_VALID",
                    f"steps={data.get('step_count')} "
                    f"frame={data.get('validated_reference_frame', '?')} "
                    f"distance={data.get('cumulative_move_rel_distance_m', '?')}m "
                    f"est_time≥{data.get('estimated_duration_lower_bound_sec', '?')}s",
                    {"io_side_effects": data.get("has_io_side_effects"),
                     "manual_recovery": data.get("manual_recovery_required_on_failure"),
                     "diagnostics": data.get("diagnostics")})
            return

        if status == "sequence_succeeded":
            elapsed = time.time() - self._cmd_start_time if self._cmd_start_time else 0
            _log(ICON["success"], GRN, "SEQUENCE_DONE",
                 f"All {data.get('step_count', '?')} steps completed | "
                 f"{ICON['time']} total={elapsed:.2f}s")
            print(f"{B}{GRN}{'━' * 72}{R}", flush=True)
            return

        if status == "query_result":
            pose = data.get("current_pose", {})
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            _log_kv(ICON["pose"], CYN, "QUERY/get_pose",
                    f"x={pos.get('x', '?')} y={pos.get('y', '?')} z={pos.get('z', '?')}",
                    {"orientation": ori, "message": data.get("message")})
            return

        if "plan_precheck" in status:
            _log(ICON["plan"], BLU, "PLAN_ONLY",
                 f"Plan precheck {status} (no execution)")
            return

        # Generic fallback
        _log(ICON["info"], CYN, f"DEBUG/{stage}",
             f"status={status} | {json.dumps(data)[:140]}")

    # ══════════════════════════════════════════════════════════════════
    # 2) /gateway_status — status transitions
    # ══════════════════════════════════════════════════════════════════
    def _on_gateway_status(self, msg: String):
        status = msg.data.strip()

        # Track command start time
        if status == "received":
            self._cmd_start_time = time.time()

        # Map status to descriptive message
        STATUS_MAP = {
            "received": ("📝 Prompt received by LLM Gateway", YLW),
            "llm_response_received": ("🧠 LLM response received, parsing...", YLW),
            "parsed": ("📋 LLM output parsed into semantic IR", YLW),
            "schema_valid": ("📋 Schema validation passed", YLW),
            "semantic_valid": ("🔍 Semantic validation passed (units, frames, limits OK)", GRN),
            "routed": ("🔀 Intent routed to primitive handler", YLW),
            "safety_validation_requested": ("🔒 Sent to SafetyGate /validate_command...", MAG),
            "safety_approved": ("🛡️ SafetyGate APPROVED — command is safe to execute", GRN),
            "dispatched": ("🚀 Goal sent to ExecuteMotion action server", BLU),
            "succeeded": ("✅ Execution completed successfully", GRN),
            "get_pose_requested": ("📍 GET_POSE query sent to motion_core", CYN),
            "query_succeeded": ("📍 GET_POSE query returned successfully", GRN),
            "ready_for_confirm": ("⏸️ Waiting for operator confirmation", YLW),
            "sequence_valid": ("📦 Sequence validated OK", GRN),
            "sequence_succeeded": ("📦 Full sequence execution done", GRN),
        }

        if status.startswith("rejected:"):
            reason = status.split(":", 1)[1] if ":" in status else status
            _log(ICON["error"], RED, "GATEWAY", f"REJECTED → {reason}")
            return

        if status.startswith("sequence_step:"):
            step_info = status.split(":", 1)[1]
            _log(ICON["sequence"], MAG, "SEQUENCE",
                 f"Executing step {step_info}...")
            return

        desc, color = STATUS_MAP.get(status, (f"status={status}", DIM))
        _log(ICON["status"], color, "GATEWAY", desc)

    # ══════════════════════════════════════════════════════════════════
    # 3) /yaskawa/robot_status
    # ══════════════════════════════════════════════════════════════════
    def _on_robot_status(self, msg: RobotStatus):
        old_mode = self._robot_mode

        if msg.e_stopped == 1 or msg.in_error == 1:
            self._robot_mode = "FAULT/ESTOP"
            self._in_error = True
        elif msg.drives_powered == 1 and msg.motion_possible == 1:
            if msg.in_motion == 1:
                self._robot_mode = "MOVING"
            else:
                self._robot_mode = "READY"
            self._in_error = False
        elif msg.drives_powered == 1:
            self._robot_mode = "IDLE"
            self._in_error = False
        else:
            self._robot_mode = "OFF"

        # Only print when state changes
        if self._robot_mode != old_mode:
            color = GRN if "READY" in self._robot_mode else (
                RED if self._in_error else (BLU if "MOVING" in self._robot_mode else YLW))
            details = (f"drives={msg.drives_powered} motion_possible={msg.motion_possible} "
                       f"in_motion={msg.in_motion} e_stop={msg.e_stopped} in_error={msg.in_error}")
            _log(ICON["motion"], color, "ROBOT_STATUS",
                 f"State changed → {B}{self._robot_mode}{R}",
                 details)

    # ══════════════════════════════════════════════════════════════════
    # 4) /yaskawa/current_pose — print every 50th update + on change
    # ══════════════════════════════════════════════════════════════════
    def _on_pose(self, msg: PoseStamped):
        self._last_pose = msg
        self._pose_print_counter += 1
        # Print every 50th message to avoid flood, but keep it live
        if self._pose_print_counter % 50 == 1:
            p = msg.pose.position
            o = msg.pose.orientation
            r, pit, y = _quat_to_rpy(o.x, o.y, o.z, o.w)
            _log(ICON["pose"], CYN, "TCP_POSE",
                 f"x={_mm(p.x)} y={_mm(p.y)} z={_mm(p.z)} | "
                 f"R={_deg(r)} P={_deg(pit)} Y={_deg(y)}")

    # ══════════════════════════════════════════════════════════════════
    # 5) /yaskawa/joint_states — print every 50th
    # ══════════════════════════════════════════════════════════════════
    def _on_joints(self, msg: JointState):
        self._last_joints = {n: float(p) for n, p in zip(msg.name, msg.position)}
        self._joint_print_counter += 1
        if self._joint_print_counter % 50 == 1:
            jstr = " ".join(f"{n}={_deg(p)}" for n, p in
                            sorted(self._last_joints.items()))
            _log(ICON["joint"], MAG, "JOINTS", jstr)

    # ══════════════════════════════════════════════════════════════════
    # 6) /supervisor/alerts
    # ══════════════════════════════════════════════════════════════════
    def _on_alert(self, msg: DiagnosticStatus):
        if "heartbeat" in msg.message.lower() or msg.message.lower() == "idle":
            return

        level_val = msg.level[0] if isinstance(msg.level, bytes) else msg.level
        if level_val >= 2:
            color, icon = RED, ICON["error"]
        elif level_val == 1:
            color, icon = YLW, ICON["warn"]
        else:
            color, icon = CYN, ICON["info"]

        kvs = {kv.key: kv.value for kv in msg.values}
        _log_kv(icon, color, "SUPERVISOR", msg.message, kvs)

    # ══════════════════════════════════════════════════════════════════
    # 7) /execute_motion/_action/status — goal lifecycle
    # ══════════════════════════════════════════════════════════════════
    def _on_goal_status(self, msg: GoalStatusArray):
        STATUS_NAMES = {
            1: "ACCEPTED", 2: "EXECUTING", 4: "SUCCEEDED",
            5: "CANCELED", 6: "ABORTED",
        }
        for s in msg.status_list:
            gid = "".join(f"{b:02x}" for b in s.goal_info.goal_id.uuid)[:8]
            name = STATUS_NAMES.get(s.status, f"status={s.status}")

            if s.status == 2:  # EXECUTING
                _log(ICON["execute"], BLU, f"MOTION #{gid}",
                     f"🤖 Robot is MOVING — trajectory executing...")
            elif s.status == 4:  # SUCCEEDED
                elapsed = time.time() - self._cmd_start_time if self._cmd_start_time else 0
                _log(ICON["success"], GRN, f"MOTION #{gid}",
                     f"Motion COMPLETED ✅ | execution_time={elapsed:.2f}s")
                print(f"{B}{GRN}{'━' * 72}{R}", flush=True)
            elif s.status in (5, 6):
                label = "CANCELED ⛔" if s.status == 5 else "ABORTED ❌"
                _log(ICON["error"], RED, f"MOTION #{gid}",
                     f"Motion {label}")
            elif s.status == 1:
                _log(ICON["dispatch"], BLU, f"MOTION #{gid}",
                     f"Goal ACCEPTED by motion_core — planning will start")

    # ══════════════════════════════════════════════════════════════════
    # 8) ExecuteMotion feedback (trajectory progress %)
    # ══════════════════════════════════════════════════════════════════
    def _on_motion_feedback(self, msg):
        fb = msg.feedback
        progress = getattr(fb, "progress", None)
        status_str = getattr(fb, "status", "")
        if progress is not None:
            pct = int(progress * 100)
            bar = "█" * (pct // 5) + "░" * (20 - pct // 5)
            _log(ICON["feedback"], BLU, "TRAJECTORY",
                 f"[{bar}] {pct}% | {status_str}")
        elif status_str:
            _log(ICON["feedback"], BLU, "TRAJECTORY", status_str)

    # ══════════════════════════════════════════════════════════════════
    # 9) /rosout — internal C++/Python logs from pipeline nodes
    #    This captures IK solve, FK, planner selection, waypoints,
    #    ruckig/totg smoothing, trajectory points, MOVE_REL resolution,
    #    branch preservation, orientation filter — ALL internal details.
    # ══════════════════════════════════════════════════════════════════
    def _on_rosout(self, msg: Log):
        node_name = msg.name or ""
        # Only show logs from pipeline nodes
        if node_name not in self._ROSOUT_NODES:
            return

        text = msg.msg or ""
        # Skip noisy/empty messages
        if not text.strip() or len(text) < 5:
            return

        # Map ROS2 log levels
        # Log.DEBUG=10, Log.INFO=20, Log.WARN=30, Log.ERROR=40, Log.FATAL=50
        level = msg.level
        if level >= 40:
            color, icon = RED, ICON["error"]
        elif level >= 30:
            color, icon = YLW, ICON["warn"]
        else:
            color, icon = DIM, "📋"

        # Highlight important keywords for quick visual scanning
        HIGHLIGHTS = {
            "IK": ("🎯", CYN),
            "ik_solution": ("🎯", CYN),
            "IK-derived": ("🎯", CYN),
            "FK": ("📐", CYN),
            "planner_selected": ("📐", GRN),
            "planning": ("📐", BLU),
            "fraction=": ("📊", GRN),
            "points=": ("📊", GRN),
            "waypoint": ("📍", CYN),
            "MOVE_REL resolved": ("🔀", MAG),
            "delta=": ("🔀", MAG),
            "ruckig": ("⚙️", YLW),
            "TOTG": ("⚙️", YLW),
            "time_parameterization": ("⚙️", YLW),
            "branch-preserved": ("🔀", CYN),
            "orientation": ("🧭", CYN),
            "Execution succeeded": ("✅", GRN),
            "Execution failed": ("❌", RED),
            "dispatching trajectory": ("🚀", BLU),
            "goal_seq=": ("⚡", BLU),
            "CIRC": ("⭕", MAG),
            "CARTESIAN_PATH": ("📏", MAG),
            "BLENDED_SEQUENCE": ("📦", MAG),
            "HOME": ("🏠", GRN),
            "ALARM_RESET": ("🔔", YLW),
            "collision": ("💥", RED),
            "joint_limit": ("🚧", RED),
            "seed": ("🌱", CYN),
            "manipulability": ("📉", YLW),
        }

        # Find the most specific highlight
        matched_icon, matched_color = icon, color
        for keyword, (kw_icon, kw_color) in HIGHLIGHTS.items():
            if keyword.lower() in text.lower():
                matched_icon = kw_icon
                matched_color = kw_color
                break  # first match wins

        # For INFO level, only show if it contains useful pipeline info
        if level < 30:
            # Skip generic lifecycle/heartbeat noise
            skip_patterns = [
                "waiting for", "service ready", "initialized",
                "timer", "callback", "subscriber", "publisher",
            ]
            text_lower = text.lower()
            if any(pat in text_lower for pat in skip_patterns):
                return

        short_name = node_name.replace("_node", "")
        _log(matched_icon, matched_color, f"INTERNAL/{short_name}", text)


def main():
    rclpy.init()
    node = GP4StreamLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{DIM}Console stopped by user.{R}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
