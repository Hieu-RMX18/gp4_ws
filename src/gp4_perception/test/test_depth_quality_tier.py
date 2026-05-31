"""Two-tier depth quality: OK (executable) / DEGRADED_DEPTH (viz only) / REJECT."""

from gp4_perception.safety_guards import classify_depth_quality

_BREAKPOINTS = [
    {"distance_m": 0.30, "noise_mm_max": 5.0},
    {"distance_m": 0.50, "noise_mm_max": 8.0},
    {"distance_m": 0.80, "noise_mm_max": 15.0},
]


class TestClassifyDepthQuality:
    def test_below_default_is_ok(self):
        quality, threshold, reason = classify_depth_quality(
            0.50, 6.0, _BREAKPOINTS, degraded_max_mm=15.0
        )
        assert quality == "OK"
        assert abs(threshold - 8.0) < 1e-6
        assert reason == ""

    def test_between_default_and_degraded_is_degraded(self):
        quality, threshold, reason = classify_depth_quality(
            0.50, 12.0, _BREAKPOINTS, degraded_max_mm=15.0
        )
        assert quality == "DEGRADED_DEPTH"
        assert abs(threshold - 8.0) < 1e-6
        assert "default" in reason

    def test_above_degraded_is_reject(self):
        quality, threshold, reason = classify_depth_quality(
            0.50, 20.0, _BREAKPOINTS, degraded_max_mm=15.0
        )
        assert quality == "REJECT"
        assert "degraded_max" in reason

    def test_distance_outside_breakpoints_rejects(self):
        quality, threshold, reason = classify_depth_quality(
            2.0, 4.0, _BREAKPOINTS, degraded_max_mm=15.0, extrapolation="reject"
        )
        assert quality == "REJECT"
        assert threshold is None
        assert "outside" in reason

    def test_clamp_uses_nearest_breakpoint(self):
        # Beyond 0.80 m with clamp → threshold clamps to 15 mm.
        quality, threshold, _ = classify_depth_quality(
            1.2, 14.0, _BREAKPOINTS, degraded_max_mm=15.0, extrapolation="clamp"
        )
        assert abs(threshold - 15.0) < 1e-6
        assert quality == "OK"
