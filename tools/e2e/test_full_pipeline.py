#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SAFETY_RULES = REPO_ROOT / "src" / "safety" / "config" / "safety_rules.yaml"
SIM_READY_TIMEOUT_SEC = 90.0
ACTION_TIMEOUT_SEC = 120.0
SERVICE_TIMEOUT_SEC = 30.0
LAUNCH_SHUTDOWN_TIMEOUT_SEC = 50.0
SPIN_STEP_SEC = 0.1
QUATERNION_TOLERANCE = 1e-3
LAUNCH_CHILD_EXIT_RE = re.compile(
    r"\[(?P<process>[^\]]+)\]: process has died .* exit code (?P<exit_code>-?\d+),"
)
LAUNCH_SHUTDOWN_MARKER = "user interrupted with ctrl-c"
EXPECTED_SIGINT_EXIT_CODE = -signal.SIGINT
EXPECTED_SIGTERM_EXIT_CODE = -signal.SIGTERM
EXPECTED_SIGKILL_EXIT_CODE = -signal.SIGKILL
EXPECTED_POST_SUCCESS_TEARDOWN_EXIT_PREFIXES = (
    "move_group-",
    "ros2_control_node-",
)
EXPECTED_POST_SUCCESS_TEARDOWN_EXIT_CODES = {
    -11,
    EXPECTED_SIGTERM_EXIT_CODE,
    EXPECTED_SIGKILL_EXIT_CODE,
}
# HOME is intentionally close to the conservative J5 lower bound. Keep the
# smoke-test delta small so the E2E probe verifies dispatch without challenging
# the safety envelope that separate guard tests cover directly.
MOVE_REL_DELTA_DIVISOR = 50.0
GP4_JOINT_NAMES = (
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t",
)
HOME_JOINT_TARGET = (
    1.5477395698141883,
    -0.15883329466662804,
    -0.15854787143360877,
    0.0,
    -1.6017466450445892,
    0.05361262853660316,
)
# Software-only staging target for this E2E probe. It reuses SRDF poseB, an
# operator-captured commissioning pose inside the configured workspace.
SOFTWARE_STAGING_JOINT_TARGET = (
    1.1122617820609437,
    0.3984751821963182,
    -0.2633406732574536,
    -0.1233275436378059,
    -1.0694327531072734,
    -0.1383267175475877,
)


def _require_ros_environment() -> None:
    if not os.environ.get("AMENT_PREFIX_PATH"):
        raise RuntimeError(
            "ROS environment not sourced. Run: source install/setup.bash"
        )


def _load_safety_rules() -> dict:
    with open(SAFETY_RULES) as f:
        return yaml.safe_load(f) or {}


def _wait_until(predicate: Callable[[], bool], timeout_sec: float, label: str) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(SPIN_STEP_SEC)
    raise TimeoutError(f"timed out waiting for {label}")


def _assert_launch_child_exit_records_clean(
    launch_log: Path,
    *,
    scenario_completed: bool = False,
) -> None:
    bad_exit_records = []
    if not launch_log.exists():
        return

    shutdown_started = False
    for line in launch_log.read_text(errors="replace").splitlines():
        if LAUNCH_SHUTDOWN_MARKER in line:
            shutdown_started = True
        match = LAUNCH_CHILD_EXIT_RE.search(line)
        if not match:
            continue
        exit_code = int(match.group("exit_code"))
        process = match.group("process")
        if exit_code == EXPECTED_SIGINT_EXIT_CODE:
            continue
        if (
            process.startswith("move_group-")
            and exit_code == EXPECTED_SIGTERM_EXIT_CODE
        ):
            continue
        if (
            scenario_completed
            and shutdown_started
            and process.startswith(EXPECTED_POST_SUCCESS_TEARDOWN_EXIT_PREFIXES)
            and exit_code in EXPECTED_POST_SUCCESS_TEARDOWN_EXIT_CODES
        ):
            continue
        bad_exit_records.append(f"{process} exit code {exit_code}")

    if bad_exit_records:
        raise RuntimeError(
            "launch child process crashed during teardown: "
            + "; ".join(bad_exit_records)
        )


def _snapshot_launch_logs() -> set[Path]:
    ros_log_dir = Path(os.environ.get("ROS_LOG_DIR", Path.home() / ".ros" / "log"))
    return set(ros_log_dir.glob("*/launch.log"))


def _newest_launch_log_after(before: set[Path]) -> Path | None:
    ros_log_dir = Path(os.environ.get("ROS_LOG_DIR", Path.home() / ".ros" / "log"))
    launch_logs = [
        path for path in ros_log_dir.glob("*/launch.log") if path not in before
    ]
    if not launch_logs:
        return None
    return max(launch_logs, key=lambda path: path.stat().st_mtime)


def _shutdown_launch_process(launch_process: subprocess.Popen) -> None:
    launch_process.send_signal(signal.SIGINT)
    try:
        launch_process.wait(timeout=LAUNCH_SHUTDOWN_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        os.killpg(launch_process.pid, signal.SIGTERM)
        launch_process.wait(timeout=10.0)


def _spin_until(node, future, timeout_sec: float, label: str):
    import rclpy

    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=SPIN_STEP_SEC)
        if future.done():
            return future.result()
    raise TimeoutError(f"timed out waiting for {label}")


def _valid_pose(pose) -> bool:
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    q_norm = math.sqrt(
        pose.orientation.x * pose.orientation.x
        + pose.orientation.y * pose.orientation.y
        + pose.orientation.z * pose.orientation.z
        + pose.orientation.w * pose.orientation.w
    )
    return abs(q_norm - 1.0) <= QUATERNION_TOLERANCE


def _move_rel_delta(current_pose, safety_rules: dict) -> tuple[float, float, float]:
    bounds = safety_rules["workspace_bounds"]
    max_delta = safety_rules["motion_limits"]["max_move_rel_translation"]
    candidate = max_delta / MOVE_REL_DELTA_DIVISOR
    if current_pose.position.z < bounds["z_min"]:
        return (
            0.0,
            0.0,
            min(max_delta, bounds["z_min"] - current_pose.position.z + candidate),
        )
    if current_pose.position.z > bounds["z_max"]:
        return (
            0.0,
            0.0,
            -min(max_delta, current_pose.position.z - bounds["z_max"] + candidate),
        )
    z_margin_down = current_pose.position.z - bounds["z_min"]
    if z_margin_down > candidate:
        return 0.0, 0.0, -candidate
    z_margin_up = bounds["z_max"] - current_pose.position.z
    if z_margin_up > candidate:
        return 0.0, 0.0, candidate
    return 0.0, 0.0, 0.0


def _make_goal(primitive: str, velocity_scale: float, acceleration_scale: float):
    from interfaces.action import ExecuteMotion

    goal = ExecuteMotion.Goal()
    goal.primitive_type = primitive
    goal.velocity_scale = velocity_scale
    goal.acceleration_scale = acceleration_scale
    return goal


def _make_ptp_joint_goal(
    joint_target: tuple[float, ...],
    velocity_scale: float,
    acceleration_scale: float,
):
    goal = _make_goal("PTP", velocity_scale, acceleration_scale)
    goal.joint_target = list(joint_target)
    return goal


def main() -> int:
    _require_ros_environment()
    safety_rules = _load_safety_rules()
    velocity_scale = safety_rules["motion_limits"]["max_velocity_scale"]
    acceleration_scale = safety_rules["motion_limits"]["max_acceleration_scale"]
    launch_logs_before = _snapshot_launch_logs()
    scenario_completed = False

    launch_process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "gp4_bringup",
            "sim.launch.py",
            "use_rviz:=false",
        ],
        cwd=str(REPO_ROOT),
        start_new_session=True,
    )

    try:
        import rclpy
        from interfaces.action import ExecuteMotion
        from interfaces.srv import GetCurrentPose
        from rclpy.action import ActionClient

        rclpy.init()
        node = rclpy.create_node("gp4_full_pipeline_e2e")
        action_client = ActionClient(node, ExecuteMotion, "execute_motion")
        pose_client = node.create_client(GetCurrentPose, "/get_current_pose")

        _wait_until(
            action_client.wait_for_server,
            SIM_READY_TIMEOUT_SEC,
            "execute_motion action server",
        )
        _wait_until(
            pose_client.wait_for_service,
            SIM_READY_TIMEOUT_SEC,
            "/get_current_pose service",
        )

        def send_motion(goal, label: str) -> None:
            send_future = action_client.send_goal_async(goal)
            goal_handle = _spin_until(
                node, send_future, SERVICE_TIMEOUT_SEC, f"{label} goal acceptance"
            )
            if not goal_handle.accepted:
                raise RuntimeError(f"{label} goal rejected")
            result_future = goal_handle.get_result_async()
            result = _spin_until(
                node, result_future, ACTION_TIMEOUT_SEC, f"{label} result"
            ).result
            if not result.success:
                raise RuntimeError(f"{label} failed: {result.message}")
            print(f"PASS {label}: {result.message}")

        def get_pose(label: str):
            request = GetCurrentPose.Request()
            request.reference_frame = "base_link"
            response = _spin_until(
                node,
                pose_client.call_async(request),
                SERVICE_TIMEOUT_SEC,
                label,
            )
            if not response.success:
                raise RuntimeError(f"{label} failed: {response.message}")
            if not _valid_pose(response.current_pose):
                raise RuntimeError(f"{label} returned invalid pose")
            print(f"PASS {label}: {response.message}")
            return response.current_pose

        send_motion(
            _make_goal("HOME", velocity_scale, acceleration_scale),
            "HOME",
        )
        get_pose("GET_POSE after HOME")

        send_motion(
            _make_ptp_joint_goal(
                SOFTWARE_STAGING_JOINT_TARGET,
                velocity_scale,
                acceleration_scale,
            ),
            "PTP software staging",
        )
        current_pose = get_pose("GET_POSE after PTP software staging")

        move_rel_goal = _make_goal("MOVE_REL", velocity_scale, acceleration_scale)
        dx, dy, dz = _move_rel_delta(current_pose, safety_rules)
        move_rel_goal.delta_x = dx
        move_rel_goal.delta_y = dy
        move_rel_goal.delta_z = dz
        move_rel_goal.reference_frame = "base_link"
        send_motion(move_rel_goal, "MOVE_REL")

        ptp_pose = get_pose("GET_POSE after MOVE_REL")
        ptp_goal = _make_goal("PTP", velocity_scale, acceleration_scale)
        ptp_goal.target_pose = ptp_pose
        send_motion(ptp_goal, "PTP current pose")

        print("full pipeline E2E: all checks passed")
        node.destroy_node()
        rclpy.shutdown()
        scenario_completed = True
        return 0
    finally:
        _shutdown_launch_process(launch_process)
        launch_log = _newest_launch_log_after(launch_logs_before)
        if launch_log is not None:
            _assert_launch_child_exit_records_clean(
                launch_log,
                scenario_completed=scenario_completed,
            )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"full pipeline E2E FAILED: {exc}", file=sys.stderr)
        raise SystemExit(1)
