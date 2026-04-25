from pathlib import Path

import pytest

from llm_gateway.text_cli import (
    build_command_argument_parser,
    build_command_from_args,
    format_command_payload,
    validate_command_payload,
)


def test_move_joints_command_is_readable_and_structured():
    parser = build_command_argument_parser()
    parsed = parser.parse_args(
        ["move-joints", "0", "0", "0", "0", "0", "0", "--speed", "0.05"]
    )

    command = build_command_from_args(parsed)

    assert command == {
        "primitive_type": "MOVE_JOINTS",
        "joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "velocity_scale": 0.05,
    }


def test_lin_command_supports_xyz_and_rpy():
    parser = build_command_argument_parser()
    parsed = parser.parse_args(
        [
            "lin",
            "--xyz",
            "0.30",
            "0.10",
            "0.42",
            "--rpy",
            "180",
            "0",
            "0",
            "--speed",
            "0.05",
        ]
    )

    command = build_command_from_args(parsed)

    assert command["primitive_type"] == "LIN"
    assert command["target_pose"]["position"] == {"x": 0.30, "y": 0.10, "z": 0.42}
    assert command["target_pose"]["orientation"] == {
        "roll": 180.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    assert command["velocity_scale"] == 0.05


def test_from_file_accepts_yaml(tmp_path: Path):
    payload_path = tmp_path / "move_rel.yaml"
    payload_path.write_text(
        "\n".join(
            [
                "primitive_type: MOVE_REL",
                "delta_x: 0.0",
                "delta_y: 0.0",
                "delta_z: -0.03",
                "velocity_scale: 0.05",
            ]
        ),
        encoding="utf-8",
    )

    parser = build_command_argument_parser()
    parsed = parser.parse_args(["from-file", str(payload_path)])
    command = build_command_from_args(parsed)

    assert command["primitive_type"] == "MOVE_REL"
    assert command["delta_z"] == -0.03


def test_validate_command_payload_rejects_out_of_range_velocity():
    with pytest.raises(ValueError, match="Schema validation failed: velocity_scale"):
        validate_command_payload(
            {
                "primitive_type": "MOVE_JOINTS",
                "joint_target": [0, 0, 0, 0, 0, 0],
                "velocity_scale": 0.10,
            }
        )


def test_format_command_payload_is_pretty_json():
    text = format_command_payload({"primitive_type": "STOP"})
    assert '"primitive_type": "STOP"' in text
    assert text.startswith("{\n")
