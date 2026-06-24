from __future__ import annotations
from typing import Any, Dict, Optional
import json
import time
import datetime
from std_msgs.msg import String

class TelemetryMixin:
    def _emit_trace(
        self,
        event: str,
        phase: str,
        level: str = "INFO",
        summary: str = "",
        command_id: str = "",
        source: str = "",
        **details,
    ) -> None:
        """Publish structured command trace event to /llm_debug."""
        trace_payload = {
            "t": "command_trace",
            "ts": time.time(),
            "cmd_id": command_id or getattr(self, "_active_command_id", ""),
            "layer": "llm_gateway",
            "phase": phase,
            "event": event,
            "level": level,
            "summary": summary,
            "details": details if details else None,
        }
        if source:
            trace_payload["source"] = source
        payload = json.dumps(trace_payload)
        pub = getattr(self, "_llm_debug_publisher", None)
        if pub is not None:
            pub.publish(String(data=payload))

    def _emit_task_event(self, category: str, event: str, detail: str, *, level: str = "INFO", source: str = "gateway", data: Optional[Dict[str, Any]] = None) -> None:
        sink = getattr(self, "_runtime_event_sink", None)
        if sink is not None:
            sink({
                "ts": datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
                "level": level, "source": source, "category": category,
                "event": event, "detail": detail, "data": self._make_json_serializable(data) if data is not None else {},
            })

    def _runtime_event_sink(self, event: Dict[str, Any]) -> None:
        pub = getattr(self, "_task_events_pub", None)
        if pub is not None:
            pub.publish(String(data=json.dumps(event, separators=(",", ":"))))

    @classmethod
    def _make_json_serializable(cls, obj: Any) -> Any:
        if isinstance(obj, dict):
            return {k: cls._make_json_serializable(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [cls._make_json_serializable(i) for i in obj]
        if hasattr(obj, "to_dict") and callable(obj.to_dict):
            return cls._make_json_serializable(obj.to_dict())
        if hasattr(obj, "__dict__"):
            return cls._make_json_serializable(obj.__dict__)
        if hasattr(obj, "__slots__"):
            return {slot: cls._make_json_serializable(getattr(obj, slot)) for slot in obj.__slots__}
        if isinstance(obj, (int, float, str, bool)) or obj is None:
            return obj
        return str(obj)
