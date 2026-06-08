from llm_gateway.composite_tools import CandidatePoseRequest, generate_candidate_poses


def test_candidate_pose_rejects_verify_config_geometry():
    request = CandidatePoseRequest(
        purpose="drop",
        region={"geometry": {"center": {"x": "VERIFY_CONFIG", "y": 0.0, "z": 0.3}}},
        safety_rules={
            "workspace_bounds": {
                "x_min": -0.45,
                "x_max": 0.45,
                "y_min": -0.16,
                "y_max": 0.52,
                "z_min": 0.15,
                "z_max": 0.65,
            }
        },
    )

    result = generate_candidate_poses(request)

    assert result.ok is False
    assert result.error == "verify_config_required"


def test_candidate_pose_applies_tool_offset_once_and_keeps_workspace_bounds():
    request = CandidatePoseRequest(
        purpose="drop",
        region={"geometry": {"center": {"x": 0.30, "y": 0.10, "z": 0.30}}},
        safety_rules={
            "workspace_bounds": {
                "x_min": -0.45,
                "x_max": 0.45,
                "y_min": -0.16,
                "y_max": 0.52,
                "z_min": 0.15,
                "z_max": 0.65,
            }
        },
        tcp_offset_m=0.12,
        approach_axis="+z_base",
    )

    result = generate_candidate_poses(request)

    assert result.ok is True
    assert result.poses[0]["position"]["z"] == 0.42
