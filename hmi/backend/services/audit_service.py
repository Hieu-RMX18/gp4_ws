from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

from ..domain.models import CommandRecord, CommandLifecycleState, SystemRuntimeState


def utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_blob(payload: dict[str, Any] | None) -> str | None:
    if payload is None:
        return None
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=True)


class AuditService:
    def __init__(
        self,
        db_path: str | Path = "hmi/data/gp4_hmi_audit.sqlite3",
        *,
        max_telemetry_snapshots: int = 50_000,
        telemetry_retention_days: int = 7,
    ) -> None:
        self._db_path = Path(db_path)
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._max_telemetry_snapshots = max_telemetry_snapshots
        self._telemetry_retention_days = telemetry_retention_days
        self._initialize_schema()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self._db_path)
        connection.row_factory = sqlite3.Row
        return connection

    def _initialize_schema(self) -> None:
        with self._connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS commands (
                    command_id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL,
                    operator_id TEXT NOT NULL,
                    raw_text TEXT NOT NULL,
                    parsed_intent_json TEXT,
                    validation_result_json TEXT,
                    structured_intent_json TEXT,
                    reject_reason TEXT,
                    plan_summary_json TEXT,
                    summary_label TEXT,
                    lifecycle_state TEXT,
                    confirm_at TEXT,
                    review_expires_at TEXT,
                    execute_at TEXT,
                    final_state TEXT,
                    plan_fingerprint TEXT,
                    correlation_id TEXT,
                    risk_level TEXT,
                    execution_result_json TEXT,
                    mode TEXT NOT NULL,
                    frame_used TEXT,
                    planner_used TEXT,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS command_events (
                    event_id TEXT PRIMARY KEY,
                    command_id TEXT NOT NULL,
                    session_id TEXT NOT NULL,
                    operator_id TEXT NOT NULL,
                    from_state TEXT,
                    to_state TEXT,
                    runtime_state TEXT,
                    reason TEXT,
                    payload_json TEXT,
                    created_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS runtime_events (
                    event_id TEXT PRIMARY KEY,
                    system_state TEXT NOT NULL,
                    session_id TEXT,
                    operator_id TEXT,
                    command_id TEXT,
                    message TEXT NOT NULL,
                    payload_json TEXT,
                    created_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS telemetry_snapshots (
                    snapshot_id TEXT PRIMARY KEY,
                    transport_state TEXT NOT NULL,
                    runtime_state TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS state_transitions (
                    event_id TEXT PRIMARY KEY,
                    channel TEXT NOT NULL,
                    previous_value TEXT,
                    next_value TEXT NOT NULL,
                    payload_json TEXT,
                    created_at TEXT NOT NULL
                );

                CREATE INDEX IF NOT EXISTS idx_telemetry_snapshots_created_at
                ON telemetry_snapshots(created_at);
                """
            )
            self._ensure_column(connection, "commands", "structured_intent_json", "TEXT")
            self._ensure_column(connection, "commands", "summary_label", "TEXT")
            self._ensure_column(connection, "commands", "lifecycle_state", "TEXT")
            self._ensure_column(connection, "commands", "review_expires_at", "TEXT")
            self._ensure_column(connection, "commands", "plan_fingerprint", "TEXT")
            self._ensure_column(connection, "commands", "correlation_id", "TEXT")
            self._ensure_column(connection, "commands", "risk_level", "TEXT")
            self._ensure_column(connection, "commands", "execution_result_json", "TEXT")

    def _ensure_column(
        self,
        connection: sqlite3.Connection,
        table_name: str,
        column_name: str,
        column_sql: str,
    ) -> None:
        existing_columns = {
            row["name"]
            for row in connection.execute(f"PRAGMA table_info({table_name})").fetchall()
        }
        if column_name in existing_columns:
            return
        connection.execute(
            f"ALTER TABLE {table_name} ADD COLUMN {column_name} {column_sql}"
        )

    def upsert_command(self, record: CommandRecord) -> None:
        now = utcnow_iso()
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO commands (
                    command_id,
                    session_id,
                    operator_id,
                    raw_text,
                    parsed_intent_json,
                    validation_result_json,
                    structured_intent_json,
                    reject_reason,
                    plan_summary_json,
                    summary_label,
                    lifecycle_state,
                    confirm_at,
                    review_expires_at,
                    execute_at,
                    final_state,
                    plan_fingerprint,
                    correlation_id,
                    risk_level,
                    execution_result_json,
                    mode,
                    frame_used,
                    planner_used,
                    created_at,
                    updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(command_id) DO UPDATE SET
                    session_id=excluded.session_id,
                    operator_id=excluded.operator_id,
                    raw_text=excluded.raw_text,
                    parsed_intent_json=excluded.parsed_intent_json,
                    validation_result_json=excluded.validation_result_json,
                    structured_intent_json=excluded.structured_intent_json,
                    reject_reason=excluded.reject_reason,
                    plan_summary_json=excluded.plan_summary_json,
                    summary_label=excluded.summary_label,
                    lifecycle_state=excluded.lifecycle_state,
                    confirm_at=excluded.confirm_at,
                    review_expires_at=excluded.review_expires_at,
                    execute_at=excluded.execute_at,
                    final_state=excluded.final_state,
                    plan_fingerprint=excluded.plan_fingerprint,
                    correlation_id=excluded.correlation_id,
                    risk_level=excluded.risk_level,
                    execution_result_json=excluded.execution_result_json,
                    mode=excluded.mode,
                    frame_used=excluded.frame_used,
                    planner_used=excluded.planner_used,
                    updated_at=excluded.updated_at
                """,
                (
                    record.command_id,
                    record.session_id,
                    record.operator_id,
                    record.raw_text,
                    _json_blob(record.parsed_intent),
                    _json_blob(record.validation_result),
                    _json_blob(record.structured_intent),
                    record.reject_reason,
                    _json_blob(record.plan_summary),
                    record.summary_label,
                    record.lifecycle_state.value,
                    record.confirm_at.isoformat() if record.confirm_at else None,
                    record.confirmation_expires_at.isoformat() if record.confirmation_expires_at else None,
                    record.execute_at.isoformat() if record.execute_at else None,
                    record.final_state.value if record.final_state else None,
                    record.plan_fingerprint,
                    record.correlation_id,
                    record.risk_level.value if record.risk_level else None,
                    _json_blob(record.execution_result),
                    record.mode.value,
                    record.frame_used,
                    record.planner_used,
                    record.created_at.isoformat(),
                    now,
                ),
            )

    def record_transition(
        self,
        *,
        command_id: str,
        session_id: str,
        operator_id: str,
        from_state: CommandLifecycleState | None,
        to_state: CommandLifecycleState | None,
        runtime_state: SystemRuntimeState | None,
        reason: str | None,
        payload: dict[str, Any] | None,
    ) -> None:
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO command_events (
                    event_id,
                    command_id,
                    session_id,
                    operator_id,
                    from_state,
                    to_state,
                    runtime_state,
                    reason,
                    payload_json,
                    created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    str(uuid4()),
                    command_id,
                    session_id,
                    operator_id,
                    from_state.value if from_state else None,
                    to_state.value if to_state else None,
                    runtime_state.value if runtime_state else None,
                    reason,
                    _json_blob(payload),
                    utcnow_iso(),
                ),
            )

    def record_runtime_event(
        self,
        *,
        system_state: SystemRuntimeState,
        message: str,
        session_id: str | None = None,
        operator_id: str | None = None,
        command_id: str | None = None,
        payload: dict[str, Any] | None = None,
    ) -> None:
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO runtime_events (
                    event_id,
                    system_state,
                    session_id,
                    operator_id,
                    command_id,
                    message,
                    payload_json,
                    created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    str(uuid4()),
                    system_state.value,
                    session_id,
                    operator_id,
                    command_id,
                    message,
                    _json_blob(payload),
                    utcnow_iso(),
                ),
            )

    def record_telemetry_snapshot(
        self,
        *,
        transport_state: str,
        runtime_state: SystemRuntimeState,
        payload: dict[str, Any],
    ) -> None:
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO telemetry_snapshots (
                    snapshot_id,
                    transport_state,
                    runtime_state,
                    payload_json,
                    created_at
                ) VALUES (?, ?, ?, ?, ?)
                """,
                (
                    str(uuid4()),
                    transport_state,
                    runtime_state.value,
                    json.dumps(payload, separators=(",", ":"), ensure_ascii=True),
                    utcnow_iso(),
                ),
            )
        self._prune_telemetry_snapshots()

    def record_state_transition(
        self,
        *,
        channel: str,
        previous_value: str | None,
        next_value: str,
        payload: dict[str, Any] | None = None,
    ) -> None:
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO state_transitions (
                    event_id,
                    channel,
                    previous_value,
                    next_value,
                    payload_json,
                    created_at
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    str(uuid4()),
                    channel,
                    previous_value,
                    next_value,
                    _json_blob(payload),
                    utcnow_iso(),
                ),
            )

    def telemetry_retention_policy(self) -> dict[str, int]:
        return {
            "maxTelemetrySnapshots": self._max_telemetry_snapshots,
            "telemetryRetentionDays": self._telemetry_retention_days,
        }

    def _prune_telemetry_snapshots(self) -> None:
        cutoff = datetime.now(timezone.utc) - timedelta(days=self._telemetry_retention_days)
        with self._connect() as connection:
            connection.execute(
                """
                DELETE FROM telemetry_snapshots
                WHERE created_at < ?
                """,
                (cutoff.isoformat(),),
            )
            connection.execute(
                """
                DELETE FROM telemetry_snapshots
                WHERE snapshot_id IN (
                    SELECT snapshot_id
                    FROM telemetry_snapshots
                    ORDER BY created_at DESC
                    LIMIT -1 OFFSET ?
                )
                """,
                (self._max_telemetry_snapshots,),
            )

    def list_commands(
        self,
        *,
        session_id: str | None = None,
        operator_id: str | None = None,
        final_state: str | None = None,
        created_from: str | None = None,
        created_to: str | None = None,
        limit: int = 50,
    ) -> list[dict[str, Any]]:
        predicates: list[str] = []
        values: list[Any] = []

        if session_id:
            predicates.append("session_id = ?")
            values.append(session_id)
        if operator_id:
            predicates.append("operator_id = ?")
            values.append(operator_id)
        if final_state:
            predicates.append("final_state = ?")
            values.append(final_state)
        if created_from:
            predicates.append("created_at >= ?")
            values.append(created_from)
        if created_to:
            predicates.append("created_at <= ?")
            values.append(created_to)

        where_clause = f"WHERE {' AND '.join(predicates)}" if predicates else ""
        values.append(max(1, min(limit, 200)))

        query = f"""
            SELECT *
            FROM commands
            {where_clause}
            ORDER BY created_at DESC
            LIMIT ?
        """
        with self._connect() as connection:
            rows = connection.execute(query, values).fetchall()
        return [dict(row) for row in rows]

    def get_command_detail(self, command_id: str) -> dict[str, Any] | None:
        with self._connect() as connection:
            command_row = connection.execute(
                "SELECT * FROM commands WHERE command_id = ?",
                (command_id,),
            ).fetchone()
            if command_row is None:
                return None

            timeline_rows = connection.execute(
                """
                SELECT *
                FROM command_events
                WHERE command_id = ?
                ORDER BY created_at ASC
                """,
                (command_id,),
            ).fetchall()
            runtime_rows = connection.execute(
                """
                SELECT *
                FROM runtime_events
                WHERE command_id = ?
                ORDER BY created_at ASC
                """,
                (command_id,),
            ).fetchall()

        return {
            "command": dict(command_row),
            "timeline": [dict(row) for row in timeline_rows],
            "runtime_events": [dict(row) for row in runtime_rows],
        }
