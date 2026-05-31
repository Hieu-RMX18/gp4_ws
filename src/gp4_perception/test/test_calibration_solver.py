"""Hand-eye solver reduced to PARK (primary) + DANIILIDIS (cross-check)."""

import cv2
import numpy as np

from gp4_perception import calibration
from gp4_perception.calibration import (
    _HAND_EYE_METHODS,
    _solve_park_with_crosscheck,
)


def test_only_park_and_daniilidis_configured():
    names = {name for name, _ in _HAND_EYE_METHODS}
    assert names == {"PARK", "DANIILIDIS"}


def _identity_args():
    R = np.tile(np.eye(3), (4, 1, 1))
    t = np.zeros((4, 3, 1))
    samples = [(np.eye(3), np.zeros(3), np.eye(3), np.zeros(3)) for _ in range(4)]
    return R, t, R, t, samples


def test_returns_park_result(monkeypatch):
    R_park = np.eye(3)
    t_park = np.array([0.1, 0.2, 0.3])
    R_dan = np.eye(3) * 2
    t_dan = np.array([0.9, 0.9, 0.9])

    def fake_solve(Rg, tg, Rt, tt, method):
        if method == cv2.CALIB_HAND_EYE_PARK:
            return R_park, t_park.reshape(3, 1)
        return R_dan, t_dan.reshape(3, 1)

    residuals = {id(R_park): 1.0, id(R_dan): 1.5}
    monkeypatch.setattr(cv2, "calibrateHandEye", fake_solve)
    monkeypatch.setattr(
        calibration,
        "_pairwise_translation_residual_mm",
        lambda samples, R, t: residuals[id(R)],
    )

    Rg, tg, Rt, tt, samples = _identity_args()
    result = _solve_park_with_crosscheck(Rg, tg, Rt, tt, samples)
    assert result is not None
    name, R_out, t_out, residual = result
    assert name == "PARK"
    np.testing.assert_array_equal(R_out, R_park)
    np.testing.assert_array_equal(t_out, t_park)
    assert residual == 1.0


def test_warns_when_solvers_disagree(monkeypatch):
    def fake_solve(Rg, tg, Rt, tt, method):
        return np.eye(3), np.zeros((3, 1))

    # PARK residual 1.0, DANIILIDIS 5.0 → disagreement > 2 mm.
    seq = {"calls": 0}

    def fake_residual(samples, R, t):
        seq["calls"] += 1
        return 1.0 if seq["calls"] == 1 else 5.0

    monkeypatch.setattr(cv2, "calibrateHandEye", fake_solve)
    monkeypatch.setattr(calibration, "_pairwise_translation_residual_mm", fake_residual)

    # Attach a dedicated handler — robust to ROS logging redirection that can
    # break caplog's root-handler propagation.
    import logging

    records: list[logging.LogRecord] = []

    class _Capture(logging.Handler):
        def emit(self, record):
            records.append(record)

    handler = _Capture(level=logging.WARNING)
    calibration._LOGGER.addHandler(handler)
    try:
        result = _solve_park_with_crosscheck(*_identity_args())
    finally:
        calibration._LOGGER.removeHandler(handler)

    assert result is not None
    assert result[0] == "PARK"
    assert any("disagree" in r.getMessage().lower() for r in records)


def test_returns_none_when_park_fails(monkeypatch):
    def fake_solve(Rg, tg, Rt, tt, method):
        raise cv2.error("solver failed")

    monkeypatch.setattr(cv2, "calibrateHandEye", fake_solve)
    Rg, tg, Rt, tt, samples = _identity_args()
    assert _solve_park_with_crosscheck(Rg, tg, Rt, tt, samples) is None
