"""Interactive CLI publisher for natural-language robot intents."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def build_argument_parser() -> argparse.ArgumentParser:
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
        default="/llm_text_input",
        help="Text input topic for llm_gateway. Default: /llm_text_input",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for a subscriber before publishing. Default: 2.0",
    )
    return parser


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


def main(args: Sequence[str] | None = None) -> None:
    parsed = build_argument_parser().parse_args(args=args)

    rclpy.init(args=None)
    node = rclpy.create_node("llm_text_cli")
    publisher = node.create_publisher(String, parsed.topic, 10)

    try:
        if not _wait_for_subscriber(node, parsed.topic, parsed.wait_timeout):
            print(
                f"WARN: no active subscribers detected on {parsed.topic}; publishing anyway.",
                file=sys.stderr,
            )

        if parsed.text:
            _publish_text(node, publisher, " ".join(parsed.text).strip())
            return

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
