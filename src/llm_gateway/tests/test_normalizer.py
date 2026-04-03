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
    assert normalized["velocity_scale"] == 0.1
    assert normalized["acceleration_scale"] == 0.1
    assert normalized["planner_id"] == "PILZ_PTP"
    assert normalized["require_approval"] is True
