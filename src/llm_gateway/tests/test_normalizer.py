import math


def test_normalizer_mm_to_m_and_pose_orientation(normalizer):
    normalized = normalizer.normalize(
        {
            "primitive_type": "LIN",
            "target_pose": {
                "position": {"x": 1000.0, "y": -500.0, "z": 250.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 180.0},
            },
            "velocity_scale": 0.2,
            "acceleration_scale": 0.3,
            "require_approval": False,
        }
    )

    pose = normalized["target_pose_msg"]
    assert math.isclose(pose.position.x, 1.0, rel_tol=1e-9)
    assert math.isclose(pose.position.y, -0.5, rel_tol=1e-9)
    assert math.isclose(pose.position.z, 0.25, rel_tol=1e-9)
    assert math.isclose(pose.orientation.z, 1.0, abs_tol=1e-9)
    assert math.isclose(pose.orientation.w, 0.0, abs_tol=1e-9)


def test_normalizer_joint_target_deg_to_rad(normalizer):
    normalized = normalizer.normalize(
        {
            "primitive_type": "PTP",
            "joint_target": [0.0, 90.0, -90.0, 180.0, -180.0, 45.0],
        }
    )
    joints = normalized["joint_target"]
    assert len(joints) == 6
    assert math.isclose(joints[1], math.pi / 2.0, rel_tol=1e-9)
    assert math.isclose(joints[2], -math.pi / 2.0, rel_tol=1e-9)
    assert math.isclose(joints[3], math.pi, rel_tol=1e-9)


def test_normalizer_defaults(normalizer):
    normalized = normalizer.normalize({"primitive_type": "HOME"})
    assert normalized["velocity_scale"] == 0.06
    assert normalized["acceleration_scale"] == 0.06
    assert normalized["planner_id"] == "PILZ_PTP"
    assert normalized["require_approval"] is False


def test_normalizer_accepts_quaternion_orientation(normalizer):
    normalized = normalizer.normalize(
        {
            "primitive_type": "LIN",
            "target_pose": {
                "position": {"x": 0.35, "y": 0.1, "z": 0.2},
                "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
            },
        }
    )

    pose = normalized["target_pose_msg"]
    assert math.isclose(pose.orientation.x, 0.0, abs_tol=1e-9)
    assert math.isclose(pose.orientation.y, 0.707, abs_tol=1e-9)
    assert math.isclose(pose.orientation.z, 0.0, abs_tol=1e-9)
    assert math.isclose(pose.orientation.w, 0.707, abs_tol=1e-9)


# ── New primitive normalization tests ──


def test_normalizer_wait_normalizes_duration(normalizer):
    """WAIT: wait_duration_sec is cast to float."""
    normalized = normalizer.normalize(
        {"primitive_type": "WAIT", "wait_duration_sec": 3}
    )
    assert normalized["wait_duration_sec"] == 3.0
    assert isinstance(normalized["wait_duration_sec"], float)
    # Non-motion primitive: no velocity_scale or planner_id injected
    assert "velocity_scale" not in normalized
    assert "planner_id" not in normalized


def test_normalizer_io_set_normalizes_types(normalizer):
    """IO_SET: io_address and io_value are cast to int."""
    normalized = normalizer.normalize(
        {"primitive_type": "IO_SET", "io_address": "10010", "io_value": "1"}
    )
    assert normalized["io_address"] == 10010
    assert normalized["io_value"] == 1
    assert isinstance(normalized["io_address"], int)
    assert isinstance(normalized["io_value"], int)


def test_normalizer_move_joint_normalizes_types(normalizer):
    """MOVE_JOINT: joint_index is int, joint_angle is float."""
    normalized = normalizer.normalize(
        {"primitive_type": "MOVE_JOINT", "joint_index": "2", "joint_angle": "0.5"}
    )
    assert normalized["joint_index"] == 2
    assert isinstance(normalized["joint_index"], int)
    assert math.isclose(normalized["joint_angle"], 0.5)
    assert isinstance(normalized["joint_angle"], float)
    # MOVE_JOINT is a motion primitive → planner_id defaults to PILZ_PTP
    assert normalized["planner_id"] == "PILZ_PTP"


def test_normalizer_move_joint_wraps_angle_to_pi(normalizer):
    """MOVE_JOINT: normalize revolute angle into (-pi, pi]."""
    normalized = normalizer.normalize(
        {"primitive_type": "MOVE_JOINT", "joint_index": 5, "joint_angle": 450.0}
    )
    assert math.isclose(normalized["joint_angle"], math.pi / 2.0, rel_tol=1e-9)


def test_normalizer_set_speed_bypasses_planner(normalizer):
    """SET_SPEED: velocity_scale provided by command, no planner default needed.

    SET_SPEED is handled as a velocity-bearing command (not in the bypass set)
    so it gets velocity_scale defaulted — but the LLM always provides it.
    """
    normalized = normalizer.normalize(
        {"primitive_type": "SET_SPEED", "velocity_scale": 0.15}
    )
    assert normalized["velocity_scale"] == 0.15


def test_normalizer_alarm_reset_bypasses_velocity(normalizer):
    """ALARM_RESET: no velocity_scale or planner_id injected."""
    normalized = normalizer.normalize({"primitive_type": "ALARM_RESET"})
    assert "velocity_scale" not in normalized
    assert "planner_id" not in normalized


def test_normalizer_stop_bypasses_velocity(normalizer):
    """STOP: no velocity_scale or planner_id injected."""
    normalized = normalizer.normalize({"primitive_type": "STOP"})
    assert "velocity_scale" not in normalized
    assert "planner_id" not in normalized


def test_normalizer_move_joints_planner_default(normalizer):
    """MOVE_JOINTS gets PILZ_PTP planner default and joint_target normalized."""
    normalized = normalizer.normalize(
        {
            "primitive_type": "MOVE_JOINTS",
            "joint_target": [0.0, 90.0, -90.0, 180.0, -180.0, 45.0],
        }
    )
    assert normalized["planner_id"] == "PILZ_PTP"
    assert len(normalized["joint_target"]) == 6
    # Degrees detected and converted to radians
    assert math.isclose(normalized["joint_target"][1], math.pi / 2.0, rel_tol=1e-9)


def test_normalizer_move_joints_wraps_angles_to_pi(normalizer):
    normalized = normalizer.normalize(
        {
            "primitive_type": "MOVE_JOINTS",
            "joint_target": [450.0, -450.0, 720.0, -720.0, 0.0, 810.0],
        }
    )
    joints = normalized["joint_target"]
    assert math.isclose(joints[0], math.pi / 2.0, rel_tol=1e-9)
    assert math.isclose(joints[1], -math.pi / 2.0, rel_tol=1e-9)
    assert math.isclose(joints[2], 0.0, abs_tol=1e-9)
    assert math.isclose(joints[3], 0.0, abs_tol=1e-9)
    assert math.isclose(joints[5], math.pi / 2.0, rel_tol=1e-9)


def test_normalizer_cartesian_path_normalizes_waypoints(normalizer):
    normalized = normalizer.normalize(
        {
            "primitive_type": "CARTESIAN_PATH",
            "reference_frame": "base_link",
            "waypoints": [
                {"position": {"x": 0.30, "y": 0.00, "z": 0.30}},
                {"position": {"x": 0.32, "y": 0.02, "z": 0.30}},
            ],
        }
    )

    assert normalized["planner_id"] == "PILZ_LIN"
    assert len(normalized["waypoints_msg"]) == 2


def test_normalizer_plan_only_forces_require_approval(normalizer):
    normalized = normalizer.normalize(
        {
            "primitive_type": "LIN",
            "target_pose": {
                "position": {"x": 0.30, "y": 0.00, "z": 0.30},
                "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
            },
            "plan_only": True,
            "require_approval": False,
        }
    )
    assert normalized["plan_only"] is True
    assert normalized["require_approval"] is True
