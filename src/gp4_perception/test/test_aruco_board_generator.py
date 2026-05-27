from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
GENERATOR_PATH = ROOT / "tools" / "generate_aruco_board.py"
FIDUCIALS_PATH = ROOT / "src" / "gp4_perception" / "config" / "fiducials.yaml"


def _load_generator_module(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.chdir(tmp_path)
    spec = importlib.util.spec_from_file_location(
        "generate_aruco_board", GENERATOR_PATH
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_import_does_not_write_board_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    _load_generator_module(tmp_path, monkeypatch)

    assert not (tmp_path / "charuco_board_10x11.png").exists()


def test_loads_board_spec_from_fiducials_yaml(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    generator = _load_generator_module(tmp_path, monkeypatch)

    board_spec = generator.load_board_spec(FIDUCIALS_PATH, dpi=300)

    assert board_spec.target_type == "charuco"
    assert board_spec.rows == 10
    assert board_spec.cols == 11
    assert board_spec.square_length_mm == pytest.approx(20.0)
    assert board_spec.marker_length_mm == pytest.approx(15.0)
    assert board_spec.dictionary_name == "DICT_5X5_100"


def test_generated_image_dimensions_follow_config(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    generator = _load_generator_module(tmp_path, monkeypatch)
    board_spec = generator.load_board_spec(FIDUCIALS_PATH, dpi=254)

    image = generator.generate_board_image(board_spec)

    assert image.shape == (
        board_spec.height_px,
        board_spec.width_px,
    )
    assert board_spec.square_px == 200
    assert board_spec.marker_px == 150
