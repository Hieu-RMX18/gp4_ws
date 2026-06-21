"""Unit tests for gp4_perception.safety_guards."""

from datetime import datetime, timedelta, timezone


from gp4_perception.safety_guards import (
    check_calibration_freshness,
    check_depth_noise,
    check_reprojection_error,
)


class TestCalibrationFreshness:
    def test_missing_date_rejects(self):
        ok, reason = check_calibration_freshness({})
        assert not ok
        assert "missing" in reason

    def test_not_calibrated_rejects(self):
        ok, reason = check_calibration_freshness(
            {"hand_eye_extrinsics": {"calibration_date": "<NOT_CALIBRATED>"}}
        )
        assert not ok
        assert "missing" in reason

    def test_fresh_accept(self):
        now = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        ok, reason = check_calibration_freshness(
            {"hand_eye_extrinsics": {"calibration_date": now}}
        )
        assert ok
        assert reason == ""

    def test_stale_is_accepted_when_age_not_enforced(self):
        old = (
            (datetime.now(timezone.utc) - timedelta(days=31))
            .isoformat()
            .replace("+00:00", "Z")
        )
        ok, reason = check_calibration_freshness(
            {"hand_eye_extrinsics": {"calibration_date": old}}
        )
        assert ok
        assert reason == ""

    def test_very_old_date_still_accepted(self):
        ok, reason = check_calibration_freshness(
            {"hand_eye_extrinsics": {"calibration_date": "2020-01-01T00:00:00Z"}}
        )
        assert ok
        assert reason == ""


class TestReprojectionError:
    def test_missing_rejects(self):
        ok, reason = check_reprojection_error({}, max_mm=3.0)
        assert not ok
        assert "missing" in reason

    def test_within_limit_accept(self):
        ok, reason = check_reprojection_error(
            {"hand_eye_extrinsics": {"reprojection_error_mm": 2.0}},
            max_mm=3.0,
        )
        assert ok
        assert reason == ""

    def test_over_limit_rejects(self):
        ok, reason = check_reprojection_error(
            {"hand_eye_extrinsics": {"reprojection_error_mm": 3.5}},
            max_mm=3.0,
        )
        assert not ok
        assert "3.5" in reason


class TestDepthNoise:
    def test_extrapolation_reject(self):
        breakpoints = [
            {"distance_m": 0.30, "noise_mm_max": 2.0},
            {"distance_m": 0.50, "noise_mm_max": 3.5},
        ]
        ok, reason = check_depth_noise(0.10, 1.0, breakpoints)
        assert not ok
        assert "outside" in reason

    def test_interpolated_threshold_reject(self):
        breakpoints = [
            {"distance_m": 0.30, "noise_mm_max": 2.0},
            {"distance_m": 0.50, "noise_mm_max": 3.5},
            {"distance_m": 0.80, "noise_mm_max": 6.0},
        ]
        # At 0.5 m, threshold = 3.5 mm. 4.0 mm > 3.5 mm -> reject
        ok, reason = check_depth_noise(0.50, 4.0, breakpoints)
        assert not ok
        assert "depth_noise" in reason

    def test_interpolated_threshold_accept(self):
        breakpoints = [
            {"distance_m": 0.30, "noise_mm_max": 2.0},
            {"distance_m": 0.50, "noise_mm_max": 3.5},
            {"distance_m": 0.80, "noise_mm_max": 6.0},
        ]
        # At 0.7 m, interpolated threshold = 3.5 + (0.7-0.5)/(0.8-0.5)*(6.0-3.5) = 3.5 + 0.667*2.5 = ~5.17 mm
        # 4.0 mm < 5.17 mm -> accept
        ok, reason = check_depth_noise(0.70, 4.0, breakpoints)
        assert ok
        assert reason == ""
