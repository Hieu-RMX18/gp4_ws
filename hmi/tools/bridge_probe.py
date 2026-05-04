#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import urllib.parse
import urllib.request
from typing import Any

import websockets


SUPPORTED_SCHEMA_VERSION = "telemetry.v1"


def _http_get_json(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=5.0) as response:
        return json.loads(response.read().decode("utf-8"))


def _assert_schema(payload: dict[str, Any], context: str) -> None:
    if payload.get("schemaVersion") != SUPPORTED_SCHEMA_VERSION:
        raise RuntimeError(
            f"{context} schema mismatch: expected {SUPPORTED_SCHEMA_VERSION}, "
            f"got {payload.get('schemaVersion')!r}"
        )


def _build_base_url(base_url: str) -> str:
    return base_url.rstrip("/")


def _snapshot_url(base_url: str, session_id: str, operator_id: str) -> str:
    query = urllib.parse.urlencode(
        {
            "session_id": session_id,
            "operator_id": operator_id,
        }
    )
    return f"{_build_base_url(base_url)}/api/hmi/snapshot?{query}"


def _stream_url(base_url: str, session_id: str, operator_id: str) -> str:
    parsed = urllib.parse.urlparse(_build_base_url(base_url))
    scheme = "wss" if parsed.scheme == "https" else "ws"
    query = urllib.parse.urlencode(
        {
            "session_id": session_id,
            "operator_id": operator_id,
        }
    )
    return urllib.parse.urlunparse(
        (
            scheme,
            parsed.netloc,
            "/api/hmi/stream",
            "",
            query,
            "",
        )
    )


def _read_path(payload: dict[str, Any], path: str) -> Any:
    current: Any = payload
    for segment in path.split("."):
        if not isinstance(current, dict) or segment not in current:
            raise KeyError(path)
        current = current[segment]
    return current


def _parse_expected(value: str) -> Any:
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered == "null":
        return None
    return value


def _apply_expectations(payload: dict[str, Any], expectations: list[str]) -> None:
    for item in expectations:
        if "=" not in item:
            raise ValueError(f"Invalid expectation {item!r}. Use field.path=value.")
        field_path, expected_raw = item.split("=", 1)
        actual = _read_path(payload, field_path)
        expected = _parse_expected(expected_raw)
        if actual != expected:
            raise AssertionError(
                f"Expectation failed for {field_path}: expected {expected!r}, got {actual!r}"
            )


def _snapshot_summary(payload: dict[str, Any]) -> dict[str, Any]:
    active_sources = [
        source["name"]
        for source in payload.get("telemetrySources", [])
        if source.get("active")
    ]
    active_joint_source = next(
        (name for name in active_sources if name.startswith("joint_states_")),
        None,
    )
    return {
        "schemaVersion": payload["schemaVersion"],
        "generatedAt": payload["generatedAt"],
        "transportState": payload["transportState"],
        "telemetryState": payload["telemetryState"],
        "runtimeSystemState": payload["runtime"]["systemState"],
        "runtimeBlocking": payload["runtime"]["blocking"],
        "runtimeStatusText": payload["runtime"]["statusText"],
        "activeSources": active_sources,
        "activeJointSource": active_joint_source,
        "sourceFreshness": {
            source["name"]: source["freshnessState"]
            for source in payload.get("telemetrySources", [])
        },
    }


def run_snapshot(args: argparse.Namespace) -> int:
    payload = _http_get_json(
        _snapshot_url(
            base_url=args.base_url,
            session_id=args.session_id,
            operator_id=args.operator_id,
        )
    )
    _assert_schema(payload, "snapshot")
    _apply_expectations(payload, args.expect)
    summary = _snapshot_summary(payload)
    _apply_expectations(
        {"summary": summary}, [f"summary.{item}" for item in args.expect_summary]
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


async def run_stream_async(args: argparse.Namespace) -> int:
    uri = _stream_url(
        base_url=args.base_url,
        session_id=args.session_id,
        operator_id=args.operator_id,
    )
    timeout = max(args.duration_sec, 1.0)
    summary: dict[str, Any] = {
        "uri": uri,
        "durationSec": timeout,
        "snapshotCount": 0,
        "heartbeatCount": 0,
        "lastSnapshot": None,
        "lastHeartbeat": None,
    }

    async with websockets.connect(uri, max_size=4 * 1024 * 1024) as websocket:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while True:
            remaining = deadline - loop.time()
            if remaining <= 0.0:
                break
            try:
                raw = await asyncio.wait_for(websocket.recv(), timeout=remaining)
            except asyncio.TimeoutError:
                break
            event = json.loads(raw)
            if event.get("type") == "snapshot":
                snapshot = event["snapshot"]
                _assert_schema(snapshot, "stream snapshot")
                summary["snapshotCount"] += 1
                summary["lastSnapshot"] = _snapshot_summary(snapshot)
                if args.print_events:
                    print(
                        json.dumps(
                            {
                                "type": "snapshot",
                                **summary["lastSnapshot"],
                            },
                            sort_keys=True,
                        )
                    )
            elif event.get("type") == "heartbeat":
                _assert_schema(event, "stream heartbeat")
                summary["heartbeatCount"] += 1
                summary["lastHeartbeat"] = {
                    "generatedAt": event["generatedAt"],
                    "transportState": event["transportState"],
                    "telemetryState": event["telemetryState"],
                }
                if args.print_events:
                    print(
                        json.dumps(
                            {"type": "heartbeat", **summary["lastHeartbeat"]},
                            sort_keys=True,
                        )
                    )
            else:
                raise RuntimeError(
                    f"Unexpected stream event type: {event.get('type')!r}"
                )

    if args.expect_last_snapshot:
        if not isinstance(summary["lastSnapshot"], dict):
            raise AssertionError(
                "Expected at least one snapshot event, but none arrived."
            )
        _apply_expectations(
            {"snapshot": summary["lastSnapshot"]},
            [f"snapshot.{item}" for item in args.expect_last_snapshot],
        )
    if summary["snapshotCount"] < args.min_snapshots:
        raise AssertionError(
            f"Expected at least {args.min_snapshots} snapshot events, got {summary['snapshotCount']}."
        )
    if (
        args.max_heartbeats is not None
        and summary["heartbeatCount"] > args.max_heartbeats
    ):
        raise AssertionError(
            f"Expected at most {args.max_heartbeats} heartbeat events, got {summary['heartbeatCount']}."
        )

    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read-only telemetry bridge probe.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    snapshot_parser = subparsers.add_parser(
        "snapshot", help="Fetch and summarize GET /api/hmi/snapshot."
    )
    snapshot_parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    snapshot_parser.add_argument("--session-id", default="probe-session")
    snapshot_parser.add_argument("--operator-id", default="probe-operator")
    snapshot_parser.add_argument(
        "--expect",
        action="append",
        default=[],
        help="Expectation in field.path=value form, for example runtime.systemState=NORMAL.",
    )
    snapshot_parser.add_argument(
        "--expect-summary",
        action="append",
        default=[],
        help="Expectation against derived summary fields, for example activeJointSource=joint_states_fallback.",
    )
    snapshot_parser.set_defaults(func=run_snapshot)

    stream_parser = subparsers.add_parser("stream", help="Observe WS /api/hmi/stream.")
    stream_parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    stream_parser.add_argument("--session-id", default="probe-session")
    stream_parser.add_argument("--operator-id", default="probe-operator")
    stream_parser.add_argument("--duration-sec", type=float, default=8.0)
    stream_parser.add_argument("--print-events", action="store_true")
    stream_parser.add_argument("--min-snapshots", type=int, default=1)
    stream_parser.add_argument("--max-heartbeats", type=int)
    stream_parser.add_argument(
        "--expect-last-snapshot",
        action="append",
        default=[],
        help="Expectation applied to the last snapshot summary, for example transportState=connected.",
    )
    stream_parser.set_defaults(func=lambda args: asyncio.run(run_stream_async(args)))

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return args.func(args)
    except Exception as exc:
        print(f"bridge_probe error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
