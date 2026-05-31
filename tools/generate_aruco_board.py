#!/usr/bin/env python3
"""Generate a printable ArUco/Charuco board from gp4_perception fiducials.yaml."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
import yaml


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "gp4_perception"
    / "config"
    / "fiducials.yaml"
)
DEFAULT_DPI = 300
DEFAULT_MARGIN_MM = 20.0

ARUCO_DICTIONARIES = {
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
}


@dataclass(frozen=True)
class BoardSpec:
    target_type: str
    dictionary_name: str
    rows: int
    cols: int
    square_length_mm: float
    marker_length_mm: float
    marker_separation_mm: float
    margin_mm: float
    dpi: int

    @property
    def px_per_mm(self) -> float:
        return self.dpi / 25.4

    @property
    def marker_px(self) -> int:
        return _mm_to_px(self.marker_length_mm, self.px_per_mm)

    @property
    def square_px(self) -> int:
        return _mm_to_px(self.square_length_mm, self.px_per_mm)

    @property
    def marker_sep_px(self) -> int:
        return _mm_to_px(self.marker_separation_mm, self.px_per_mm)

    @property
    def margin_px(self) -> int:
        return _mm_to_px(self.margin_mm, self.px_per_mm)

    @property
    def width_px(self) -> int:
        if self.target_type == "charuco":
            return self.cols * self.square_px + 2 * self.margin_px
        return (
            self.cols * self.marker_px
            + (self.cols - 1) * self.marker_sep_px
            + 2 * self.margin_px
        )

    @property
    def height_px(self) -> int:
        if self.target_type == "charuco":
            return self.rows * self.square_px + 2 * self.margin_px
        return (
            self.rows * self.marker_px
            + (self.rows - 1) * self.marker_sep_px
            + 2 * self.margin_px
        )

    @property
    def width_mm(self) -> float:
        if self.target_type == "charuco":
            return self.cols * self.square_length_mm + 2 * self.margin_mm
        return (
            self.cols * self.marker_length_mm
            + (self.cols - 1) * self.marker_separation_mm
            + 2 * self.margin_mm
        )

    @property
    def height_mm(self) -> float:
        if self.target_type == "charuco":
            return self.rows * self.square_length_mm + 2 * self.margin_mm
        return (
            self.rows * self.marker_length_mm
            + (self.rows - 1) * self.marker_separation_mm
            + 2 * self.margin_mm
        )


def _mm_to_px(mm_value: float, px_per_mm: float) -> int:
    return int(round(mm_value * px_per_mm))


def _positive_int(raw: object, name: str) -> int:
    value = int(raw)
    if value <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return value


def _positive_mm_from_m(raw: object, name: str) -> float:
    value_m = float(raw)
    if value_m <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return value_m * 1000.0


def load_board_spec(
    config_path: Path = DEFAULT_CONFIG_PATH,
    *,
    dpi: int = DEFAULT_DPI,
    margin_mm: float = DEFAULT_MARGIN_MM,
) -> BoardSpec:
    """Load board geometry from the same YAML used by calibration."""
    config = yaml.safe_load(config_path.read_text()) or {}
    fiducials = config.get("fiducials") or {}

    dictionary_name = str(fiducials.get("marker_dictionary", ""))
    if dictionary_name not in ARUCO_DICTIONARIES:
        supported = ", ".join(sorted(ARUCO_DICTIONARIES))
        raise ValueError(
            f"Unsupported marker_dictionary {dictionary_name!r}; supported: {supported}"
        )

    target_type = str(fiducials.get("target_type", "aruco")).lower()
    if target_type not in {"aruco", "charuco"}:
        raise ValueError("fiducials.target_type must be 'aruco' or 'charuco'")

    return BoardSpec(
        target_type=target_type,
        dictionary_name=dictionary_name,
        rows=_positive_int(fiducials.get("board_rows"), "fiducials.board_rows"),
        cols=_positive_int(fiducials.get("board_columns"), "fiducials.board_columns"),
        square_length_mm=_positive_mm_from_m(
            fiducials.get("square_length_m", fiducials.get("marker_length_m")),
            "fiducials.square_length_m",
        ),
        marker_length_mm=_positive_mm_from_m(
            fiducials.get("marker_length_m"), "fiducials.marker_length_m"
        ),
        marker_separation_mm=_positive_mm_from_m(
            fiducials.get("marker_separation_m"), "fiducials.marker_separation_m"
        ),
        margin_mm=float(margin_mm),
        dpi=_positive_int(dpi, "dpi"),
    )


def _get_aruco_dictionary(dictionary_name: str):
    dictionary_id = ARUCO_DICTIONARIES[dictionary_name]
    try:
        return cv2.aruco.Dictionary_get(dictionary_id)
    except AttributeError:
        return cv2.aruco.getPredefinedDictionary(dictionary_id)


def _draw_marker(aruco_dict, marker_id: int, marker_px: int):
    try:
        return cv2.aruco.drawMarker(aruco_dict, marker_id, marker_px)
    except AttributeError:
        return cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_px)


def _create_charuco_board(board_spec: BoardSpec, aruco_dict):
    size = (board_spec.cols, board_spec.rows)
    try:
        return cv2.aruco.CharucoBoard_create(
            board_spec.cols,
            board_spec.rows,
            board_spec.square_length_mm,
            board_spec.marker_length_mm,
            aruco_dict,
        )
    except AttributeError:
        return cv2.aruco.CharucoBoard(
            size,
            board_spec.square_length_mm,
            board_spec.marker_length_mm,
            aruco_dict,
        )


def _draw_charuco_board(board_spec: BoardSpec, aruco_dict) -> np.ndarray:
    board = _create_charuco_board(board_spec, aruco_dict)
    inner_size = (
        board_spec.cols * board_spec.square_px,
        board_spec.rows * board_spec.square_px,
    )
    try:
        inner = board.draw(inner_size)
    except AttributeError:
        inner = board.generateImage(inner_size)

    image = np.full((board_spec.height_px, board_spec.width_px), 255, dtype=np.uint8)
    y0 = board_spec.margin_px
    x0 = board_spec.margin_px
    image[y0 : y0 + inner.shape[0], x0 : x0 + inner.shape[1]] = inner
    return image


def generate_board_image(board_spec: BoardSpec) -> np.ndarray:
    """Return a white printable board image with marker IDs filled row-major."""
    image = np.full((board_spec.height_px, board_spec.width_px), 255, dtype=np.uint8)
    aruco_dict = _get_aruco_dictionary(board_spec.dictionary_name)

    if board_spec.target_type == "charuco":
        return _draw_charuco_board(board_spec, aruco_dict)

    marker_id = 0
    for row in range(board_spec.rows):
        for col in range(board_spec.cols):
            x = board_spec.margin_px + col * (
                board_spec.marker_px + board_spec.marker_sep_px
            )
            y = board_spec.margin_px + row * (
                board_spec.marker_px + board_spec.marker_sep_px
            )
            marker_image = _draw_marker(aruco_dict, marker_id, board_spec.marker_px)
            image[y : y + board_spec.marker_px, x : x + board_spec.marker_px] = (
                marker_image
            )
            marker_id += 1

    return image


def write_board_image(output_path: Path, board_spec: BoardSpec) -> None:
    image = generate_board_image(board_spec)
    if not cv2.imwrite(str(output_path), image):
        raise OSError(f"Failed to write board image: {output_path}")


def _default_output_path(board_spec: BoardSpec) -> Path:
    return Path(
        f"{board_spec.target_type}_board_{board_spec.rows}x{board_spec.cols}.png"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate the printable ArUco/Charuco board used for "
            "GP4 hand-eye calibration."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to gp4_perception fiducials.yaml.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. Defaults to <target>_board_<rows>x<cols>.png.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=DEFAULT_DPI,
        help="Target print resolution. Print at this DPI for correct scale.",
    )
    parser.add_argument(
        "--margin-mm",
        type=float,
        default=DEFAULT_MARGIN_MM,
        help="White margin around the marker grid.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    board_spec = load_board_spec(args.config, dpi=args.dpi, margin_mm=args.margin_mm)
    output_path = args.output or _default_output_path(board_spec)
    write_board_image(output_path, board_spec)

    print(f"Board saved: {output_path}")
    print(
        "  Size: "
        f"{board_spec.width_px}x{board_spec.height_px} px "
        f"({board_spec.width_mm:.0f}x{board_spec.height_mm:.0f} mm)"
    )
    print(
        f"  Board: {board_spec.target_type} {board_spec.rows}x{board_spec.cols}, "
        f"square {board_spec.square_length_mm:.0f} mm, "
        f"marker {board_spec.marker_length_mm:.0f} mm"
    )
    print(f"  Dictionary: {board_spec.dictionary_name}")
    print(f"  Print at {board_spec.dpi} DPI for correct scale.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
