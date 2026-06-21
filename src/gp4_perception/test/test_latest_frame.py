import numpy as np

from gp4_perception.latest_frame import LatestValueSlot, MonotonicRateGate
from gp4_perception.unified_visualizer import _downscale_to_max_width


def test_latest_value_slot_overwrites_stale_value():
    slot = LatestValueSlot()
    slot.put("old")
    slot.put("new")

    assert slot.take() == "new"
    assert slot.take() is None


def test_rate_gate_allows_first_call_and_rejects_early_call():
    gate = MonotonicRateGate(rate_hz=10.0)

    assert gate.allow(now=1.0)
    assert not gate.allow(now=1.05)
    assert gate.allow(now=1.10)


def test_zero_rate_disables_gate_blocking():
    gate = MonotonicRateGate(rate_hz=0.0)

    assert gate.allow(now=1.0)
    assert gate.allow(now=1.01)
    assert gate.allow(now=1.02)

def test_downscale_to_max_width_preserves_aspect_ratio():
    image = np.zeros((100, 200, 3), dtype="uint8")

    resized = _downscale_to_max_width(image, max_width=50)

    assert resized.shape == (25, 50, 3)

def test_downscale_to_max_width_leaves_small_images_unchanged():
    image = np.zeros((25, 50, 3), dtype="uint8")

    resized = _downscale_to_max_width(image, max_width=100)

    assert resized is image
