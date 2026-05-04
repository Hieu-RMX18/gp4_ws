"""CLI tools for natural-language, raw-command, and direct-action robot tests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any, Dict, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import yaml

from llm_gateway.normalizer import Normalizer
from llm_gateway.schema_validator import SchemaValidator
from llm_gateway.semantic_validator import SemanticValidator


DEFAULT_TEXT_TOPIC = "/llm_text_input"
DEFAULT_RAW_TOPIC = "/llm_raw_command"
DEFAULT_ACTION_NAME = "/execute_motion"


def build_text_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish natural-language commands to the llm_gateway text topic."
    )
    parser.add_argument(
        "text",
        nargs="*",
        help="Optional one-shot text command. If omitted, starts an interactive prompt.",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TEXT_TOPIC,
        help=f"Text input topic for llm_gateway. Default: {DEFAULT_TEXT_TOPIC}",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for a subscriber before publishing. Default: 2.0",
    )
    return parser


def build_argument_parser() -> argparse.ArgumentParser:
    """Backward-compatible alias for the original text CLI parser."""
    return build_text_argument_parser()


def _wait_for_subscriber(node: Node, topic: str, timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(timeout_sec, 0.0)
    while rclpy.ok() and time.monotonic() < deadline:
        if node.count_subscribers(topic) > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.count_subscribers(topic) > 0


def _publish_text(node: Node, publisher, text: str) -> None:
    publisher.publish(String(data=text))
    rclpy.spin_once(node, timeout_sec=0.1)


def _publish_string_message(
    *,
    topic: str,
    payload_text: str,
    node_name: str,
    timeout_sec: float,
) -> None:
    node = rclpy.create_node(node_name)
    publisher = node.create_publisher(String, topic, 10)
    try:
        if not _wait_for_subscriber(node, topic, timeout_sec):
            print(
                f"WARN: no active subscribers detected on {topic}; publishing anyway.",
                file=sys.stderr,
            )
        _publish_text(node, publisher, payload_text)
    finally:
        node.destroy_node()


def main_text(args: Sequence[str] | None = None) -> None:
    parsed = build_text_argument_parser().parse_args(args=args)

    rclpy.init(args=None)
    node = None
    try:
        if parsed.text:
            _publish_string_message(
                topic=parsed.topic,
                payload_text=" ".join(parsed.text).strip(),
                node_name="llm_text_cli",
                timeout_sec=parsed.wait_timeout,
            )
            return

        node = rclpy.create_node("llm_text_cli")
        publisher = node.create_publisher(String, parsed.topic, 10)
        while rclpy.ok():
            try:
                line = input(">> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print()
                break

            if not line:
                continue

            _publish_text(node, publisher, line)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _add_publish_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--transport",
        choices=("raw", "action"),
        default="raw",
        help="raw = publish to /llm_raw_command, action = send direct /execute_motion goal.",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_RAW_TOPIC,
        help=f"Raw command topic. Only used with --transport raw. Default: {DEFAULT_RAW_TOPIC}",
    )
    parser.add_argument(
        "--action-name",
        default=DEFAULT_ACTION_NAME,
        help=f"ExecuteMotion action name. Only used with --transport action. Default: {DEFAULT_ACTION_NAME}",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for a subscriber/server before sending.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print the command without sending it.",
    )
    parser.add_argument(
        "--no-validate",
        action="store_true",
        help="Skip local schema/semantic validation and send as-is.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Do not pretty-print the command before sending.",
    )


def _add_motion_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--speed",
        dest="velocity_scale",
        type=float,
        help="Velocity scaling factor. Typical hardware regression value: 0.05.",
    )
    parser.add_argument(
        "--accel",
        dest="acceleration_scale",
        type=float,
        help="Acceleration scaling factor. Typical hardware regression value: 0.05.",
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Request planning only; downstream still enforces approval policy.",
    )


def _add_pose_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--xyz",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        required=True,
        help="Target position in meters.",
    )
    orientation_group = parser.add_mutually_exclusive_group()
    orientation_group.add_argument(
        "--quat",
        nargs=4,
        type=float,
        metavar=("QX", "QY", "QZ", "QW"),
        help="Quaternion orientation.",
    )
    orientation_group.add_argument(
        "--rpy",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW"),
        help="Orientation as roll/pitch/yaw. Degrees are accepted if magnitudes exceed 2*pi.",
    )


def build_command_argument_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    _add_publish_options(common)

    motion = argparse.ArgumentParser(add_help=False)
    _add_motion_options(motion)

    parser = argparse.ArgumentParser(
        description="Readable CLI for raw topic publishing or direct ExecuteMotion action goals.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command_name", required=True)

    subparsers.add_parser("home", parents=[common, motion], help="Send HOME primitive.")
    subparsers.add_parser("stop", parents=[common], help="Send STOP primitive.")
    subparsers.add_parser("get-pose", parents=[common], help="Send GET_POSE primitive.")
    subparsers.add_parser(
        "alarm-reset", parents=[common], help="Send ALARM_RESET primitive."
    )

    wait_parser = subparsers.add_parser(
        "wait", parents=[common], help="Send WAIT primitive."
    )
    wait_parser.add_argument("seconds", type=float, help="Wait duration in seconds.")

    set_speed_parser = subparsers.add_parser(
        "set-speed",
        parents=[common],
        help="Send SET_SPEED primitive.",
    )
    set_speed_parser.add_argument(
        "speed",
        type=float,
        help="Velocity scale to request. Hardware-conservative range is 0.01 to 0.06.",
    )

    move_rel_parser = subparsers.add_parser(
        "move-rel",
        parents=[common, motion],
        help="Send MOVE_REL primitive.",
    )
    move_rel_parser.add_argument(
        "--x", type=float, default=0.0, help="delta_x in meters."
    )
    move_rel_parser.add_argument(
        "--y", type=float, default=0.0, help="delta_y in meters."
    )
    move_rel_parser.add_argument(
        "--z", type=float, default=0.0, help="delta_z in meters."
    )
    move_rel_parser.add_argument(
        "--frame",
        default="base_link",
        choices=["base_link"],
        help="Reference frame for MOVE_REL.",
    )

    move_joint_parser = subparsers.add_parser(
        "move-joint",
        parents=[common, motion],
        help="Send MOVE_JOINT primitive.",
    )
    move_joint_parser.add_argument(
        "--index", type=int, required=True, help="Zero-based joint index [0..5]."
    )
    move_joint_parser.add_argument(
        "--angle",
        type=float,
        required=True,
        help="Joint target angle in radians or degrees.",
    )

    move_joints_parser = subparsers.add_parser(
        "move-joints",
        parents=[common, motion],
        help="Send MOVE_JOINTS primitive.",
    )
    move_joints_parser.add_argument(
        "joints",
        nargs=6,
        type=float,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Six target joint values in radians or degrees.",
    )

    ptp_parser = subparsers.add_parser(
        "ptp", parents=[common, motion], help="Send PTP primitive."
    )
    _add_pose_options(ptp_parser)

    lin_parser = subparsers.add_parser(
        "lin", parents=[common, motion], help="Send LIN primitive."
    )
    _add_pose_options(lin_parser)

    circ_parser = subparsers.add_parser(
        "circ", parents=[common, motion], help="Send CIRC primitive."
    )
    _add_pose_options(circ_parser)
    circ_parser.add_argument(
        "--mid",
        nargs=3,
        type=float,
        metavar=("MX", "MY", "MZ"),
        required=True,
        help="Auxiliary waypoint position for CIRC.",
    )

    from_file_parser = subparsers.add_parser(
        "from-file",
        parents=[common],
        help="Load a raw command from YAML/JSON file, or '-' for stdin.",
    )
    from_file_parser.add_argument(
        "path", help="Path to YAML/JSON file, or '-' for stdin."
    )

    return parser


def _pose_from_args(parsed: argparse.Namespace) -> Dict[str, Any]:
    x, y, z = parsed.xyz
    pose: Dict[str, Any] = {"position": {"x": x, "y": y, "z": z}}
    if parsed.quat is not None:
        qx, qy, qz, qw = parsed.quat
        pose["orientation"] = {"x": qx, "y": qy, "z": qz, "w": qw}
    elif parsed.rpy is not None:
        roll, pitch, yaw = parsed.rpy
        pose["orientation"] = {"roll": roll, "pitch": pitch, "yaw": yaw}
    return pose


def _inject_motion_options(
    command: Dict[str, Any], parsed: argparse.Namespace
) -> Dict[str, Any]:
    if getattr(parsed, "velocity_scale", None) is not None:
        command["velocity_scale"] = parsed.velocity_scale
    if getattr(parsed, "acceleration_scale", None) is not None:
        command["acceleration_scale"] = parsed.acceleration_scale
    if getattr(parsed, "plan_only", False):
        command["plan_only"] = True
    return command


def _load_payload_from_file(path_str: str) -> Dict[str, Any]:
    content = (
        sys.stdin.read()
        if path_str == "-"
        else Path(path_str).read_text(encoding="utf-8")
    )
    loaded = yaml.safe_load(content)
    if not isinstance(loaded, dict):
        raise ValueError("Command file must decode to a mapping/object.")
    return loaded


def build_command_from_args(parsed: argparse.Namespace) -> Dict[str, Any]:
    command_name = parsed.command_name
    if command_name == "home":
        return _inject_motion_options({"primitive_type": "HOME"}, parsed)
    if command_name == "stop":
        return {"primitive_type": "STOP"}
    if command_name == "get-pose":
        return {"primitive_type": "GET_POSE"}
    if command_name == "alarm-reset":
        return {"primitive_type": "ALARM_RESET"}
    if command_name == "wait":
        return {"primitive_type": "WAIT", "wait_duration_sec": parsed.seconds}
    if command_name == "set-speed":
        return {"primitive_type": "SET_SPEED", "velocity_scale": parsed.speed}
    if command_name == "move-rel":
        return _inject_motion_options(
            {
                "primitive_type": "MOVE_REL",
                "delta_x": parsed.x,
                "delta_y": parsed.y,
                "delta_z": parsed.z,
                "reference_frame": parsed.frame,
            },
            parsed,
        )
    if command_name == "move-joint":
        return _inject_motion_options(
            {
                "primitive_type": "MOVE_JOINT",
                "joint_index": parsed.index,
                "joint_angle": parsed.angle,
            },
            parsed,
        )
    if command_name == "move-joints":
        return _inject_motion_options(
            {
                "primitive_type": "MOVE_JOINTS",
                "joint_target": list(parsed.joints),
            },
            parsed,
        )
    if command_name == "ptp":
        return _inject_motion_options(
            {"primitive_type": "PTP", "target_pose": _pose_from_args(parsed)}, parsed
        )
    if command_name == "lin":
        return _inject_motion_options(
            {"primitive_type": "LIN", "target_pose": _pose_from_args(parsed)}, parsed
        )
    if command_name == "circ":
        return _inject_motion_options(
            {
                "primitive_type": "CIRC",
                "target_pose": _pose_from_args(parsed),
                "waypoints": [
                    {
                        "position": {
                            "x": parsed.mid[0],
                            "y": parsed.mid[1],
                            "z": parsed.mid[2],
                        }
                    }
                ],
            },
            parsed,
        )
    if command_name == "from-file":
        return _load_payload_from_file(parsed.path)
    raise ValueError(f"Unsupported command_name: {command_name}")


def validate_command_payload(command: Dict[str, Any]) -> Dict[str, Any]:
    schema_validator = SchemaValidator()
    valid, error = schema_validator.validate_against_schema(command)
    if not valid:
        raise ValueError(f"Schema validation failed: {error}")
    normalized = Normalizer().normalize(command)
    SemanticValidator().validate(normalized)
    return normalized


def format_command_payload(command: Dict[str, Any]) -> str:
    return json.dumps(command, ensure_ascii=True, indent=2)


def _wait_for_action_server(client, timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(timeout_sec, 0.0)
    while time.monotonic() < deadline:
        if client.wait_for_server(timeout_sec=0.1):
            return True
    return client.server_is_ready()


def _send_action_goal(
    action_name: str, normalized_command: Dict[str, Any], timeout_sec: float
) -> None:
    from rclpy.action import ActionClient
    from interfaces.action import ExecuteMotion
    from llm_gateway.goal_mapper import GoalMapper

    node = rclpy.create_node("gp4_cmd_action_cli")
    client = ActionClient(node, ExecuteMotion, action_name)
    try:
        if not _wait_for_action_server(client, timeout_sec):
            raise RuntimeError(
                f"ExecuteMotion action server unavailable: {action_name}"
            )

        goal = GoalMapper().to_execute_motion_goal(normalized_command)
        send_future = client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("ExecuteMotion action rejected the goal.")

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        wrapped_result = result_future.result()
        result = wrapped_result.result if wrapped_result else None
        success = bool(result and result.success)
        message = result.message if result else "no result"
        execution_time_sec = result.execution_time_sec if result else 0.0
        print(
            json.dumps(
                {
                    "success": success,
                    "message": message,
                    "execution_time_sec": execution_time_sec,
                },
                ensure_ascii=True,
                indent=2,
            )
        )
        if not success:
            raise SystemExit(1)
    finally:
        client.destroy()
        node.destroy_node()


def main_raw(args: Sequence[str] | None = None) -> None:
    parsed = build_command_argument_parser().parse_args(args=args)
    rclpy.init(args=None)
    try:
        command = build_command_from_args(parsed)
        normalized_command = None
        if not parsed.no_validate:
            normalized_command = validate_command_payload(command)
    except Exception as exc:
        if rclpy.ok():
            rclpy.shutdown()
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc

    try:
        if not parsed.quiet:
            print(format_command_payload(command))

        if parsed.dry_run:
            return

        if parsed.transport == "action" and command.get("primitive_type") == "GET_POSE":
            raise ValueError(
                "GET_POSE is query-only; use llm_text_cli/raw gateway path instead of direct action."
            )

        if parsed.transport == "raw":
            _publish_string_message(
                topic=parsed.topic,
                payload_text=json.dumps(command, ensure_ascii=True),
                node_name="gp4_cmd_raw_cli",
                timeout_sec=parsed.wait_timeout,
            )
            return

        if normalized_command is None:
            normalized_command = validate_command_payload(command)
        _send_action_goal(parsed.action_name, normalized_command, parsed.wait_timeout)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main_text()
