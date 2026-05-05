"""Unit tests for query_perception_tool."""

from datetime import datetime, timezone
from pathlib import Path

import pytest
import yaml

from gp4_perception.query_perception_tool import query_perception


class TestQueryPerception:
    def test_blocks_during_motion(self):
        result = query_perception(
            args={},
            context_state={"robot_state": {"mode": "MOVING"}},
        )
        assert not result["ok"]
        assert "perception_blocked_during_motion" in result["error"]

    def test_rejects_stale_calibration(self, tmp_path: Path):
        extrinsics = {
            "hand_eye_extrinsics": {
                "calibration_date": "2020-01-01T00:00:00Z",
                "reprojection_error_mm": 1.0,
            }
        }
        path = tmp_path / "extrinsics.yaml"
        path.write_text(yaml.dump(extrinsics))
        result = query_perception(
            args={},
            context_state={"robot_state": {"mode": "IDLE"}},
            max_age_days=30,
            extrinsics_path=path,
        )
        assert not result["ok"]
        assert "calibration_invalid" in result["error"]

    def test_accepts_idle_with_fresh_calibration(self, tmp_path: Path):
        now = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        extrinsics = {
            "hand_eye_extrinsics": {
                "calibration_date": now,
                "reprojection_error_mm": 1.0,
            }
        }
        path = tmp_path / "extrinsics.yaml"
        path.write_text(yaml.dump(extrinsics))
        result = query_perception(
            args={},
            context_state={"robot_state": {"mode": "IDLE"}},
            max_age_days=30,
            extrinsics_path=path,
        )
        assert result["ok"]
