"""Consolidated tests; original source sections are marked below."""



# ---- test_compute_arc_points.py ----
"""Tests for compute_arc_points ReAct tool."""

import math

import pytest

from llm_gateway.react_planner import ComputeArcPointsTool


@pytest.fixture
def tool():
    return ComputeArcPointsTool()


def test_90_deg_arc_xy_plane(tool):
    args = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.2},
        "radius_m": 0.05,
        "start_angle_rad": 0.0,
        "sweep_angle_rad": math.radians(90),
        "plane_normal": {"x": 0.0, "y": 0.0, "z": 1.0},
    }
    result = tool.invoke(args, None)
    assert result.ok is True
    payload = result.payload
    start = payload["start_pose"]["pose"]["position"]
    aux = payload["auxiliary_pose"]["pose"]["position"]
    end = payload["target_pose"]["pose"]["position"]
    # Start at 0 deg -> (0.05, 0, 0.2)
    assert start["x"] == pytest.approx(0.05, abs=1e-6)
    assert start["y"] == pytest.approx(0.0, abs=1e-6)
    assert start["z"] == pytest.approx(0.2, abs=1e-6)
    # Auxiliary at 45 deg -> (0.05*cos45, 0.05*sin45)
    assert aux["x"] == pytest.approx(0.05 * math.cos(math.radians(45)), abs=1e-6)
    assert aux["y"] == pytest.approx(0.05 * math.sin(math.radians(45)), abs=1e-6)
    # End at 90 deg -> (0, 0.05, 0.2)
    assert end["x"] == pytest.approx(0.0, abs=1e-6)
    assert end["y"] == pytest.approx(0.05, abs=1e-6)
    assert end["z"] == pytest.approx(0.2, abs=1e-6)


def test_zero_sweep_rejected(tool):
    args = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.2},
        "radius_m": 0.05,
        "start_angle_rad": 0.0,
        "sweep_angle_rad": 0.0,
        "plane_normal": {"x": 0.0, "y": 0.0, "z": 1.0},
    }
    result = tool.invoke(args, None)
    assert result.ok is False
    assert "sweep_angle_rad must be non-zero" in result.error


def test_negative_radius_rejected(tool):
    args = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.2},
        "radius_m": -0.01,
        "start_angle_rad": 0.0,
        "sweep_angle_rad": math.radians(90),
        "plane_normal": {"x": 0.0, "y": 0.0, "z": 1.0},
    }
    result = tool.invoke(args, None)
    assert result.ok is False
    assert "radius_m must be > 0" in result.error


def test_yz_plane_arc(tool):
    args = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.0},
        "radius_m": 0.05,
        "start_angle_rad": 0.0,
        "sweep_angle_rad": math.radians(90),
        "plane_normal": {"x": 0.0, "y": 1.0, "z": 0.0},
    }
    result = tool.invoke(args, None)
    assert result.ok is True
    start = result.payload["start_pose"]["pose"]["position"]
    end = result.payload["target_pose"]["pose"]["position"]
    # Start at 0 deg in XZ plane -> (0.05, 0, 0)
    assert start["x"] == pytest.approx(0.05, abs=1e-6)
    assert start["y"] == pytest.approx(0.0, abs=1e-6)
    assert start["z"] == pytest.approx(0.0, abs=1e-6)
    # End at 90 deg -> (0, 0, -0.05) since normal is +Y (right-handed)
    assert end["x"] == pytest.approx(0.0, abs=1e-6)
    assert end["z"] == pytest.approx(-0.05, abs=1e-6)


# ---- test_motion_tools.py ----
"""Safety regression tests for ReAct motion tools."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from llm_gateway.intent_engine import GoalMapper
from llm_gateway.react_planner import AgentContext
from llm_gateway.react_planner import StateInjector
from llm_gateway.react_planner import GetCurrentPoseTool
from llm_gateway.react_planner import PlanMotionTool
from llm_gateway.react_planner import QueryPerceptionTool
from llm_gateway.react_planner import SubmitMotionTool
from llm_gateway.intent_engine import Normalizer
from llm_gateway.intent_engine import SchemaValidator
from llm_gateway.intent_engine import SemanticValidator


class _ReadyPoseClient:
    def service_is_ready(self):
        return True


class _PoseNode:
    _get_pose_client = _ReadyPoseClient()


class _PoseSnapshotNode:
    def __init__(self):
        self.requested_frame = None

    def _request_current_pose_snapshot(self, reference_frame):
        self.requested_frame = reference_frame
        return {
            "position": {"x": 0.30, "y": 0.0, "z": 0.40},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }


class _ValidationResponse:
    valid = True
    sanitized_json = ""


class _DoneFuture:
    def __init__(self, response=None):
        self._response = response or _ValidationResponse()

    def done(self):
        return True

    def result(self):
        return self._response


class _ValidateRequest:
    def __init__(self):
        self.command_json = ""
        self.primitive_type = ""
        from geometry_msgs.msg import Pose

        self.target_pose = Pose()
        self.velocity_scale = 0.0


class _ValidateClient:
    RequestType = _ValidateRequest

    def __init__(self):
        self.last_request = None

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.last_request = request
        return _DoneFuture(getattr(self, "response", _ValidationResponse()))


class _SourceOnlyGoalMapper:
    def __init__(self):
        self._payload_mapper = GoalMapper(
            default_velocity_scale=0.06,
            default_acceleration_scale=0.06,
        )

    def to_command_payload(self, command):
        return self._payload_mapper.to_command_payload(command)

    def to_execute_motion_goal(self, command):
        return SimpleNamespace(
            primitive_type=str(command["primitive_type"]),
            joint_target=list(command.get("joint_target", [])),
            target_pose=command.get("target_pose_msg"),
            velocity_scale=float(command.get("velocity_scale", 0.0)),
            acceleration_scale=float(command.get("acceleration_scale", 0.0)),
            planner_id=str(command.get("planner_id", "")),
        )


class _PlanNode:
    def __init__(self):
        self._validate_client = _ValidateClient()
        self._react_plan_cache = {}


class _PerceptionQueryNode:
    def __init__(self, result):
        self.result = result
        self.last_args = None

    def _query_perception_detections(self, args):
        self.last_args = dict(args)
        return self.result


class _GatewayLikePlanNode:
    def __init__(self):
        self._validate_client = _ValidateClient()
        self._react_plan_cache = {}
        self._safety_service_timeout_sec = 2.0
        self._schema_validator = SchemaValidator()
        self._normalizer = Normalizer(
            default_velocity_scale=0.06,
            default_acceleration_scale=0.06,
        )
        self._semantic_validator = SemanticValidator()
        self._goal_mapper = _SourceOnlyGoalMapper()

    def _normalize_and_validate(self, command):
        normalized = self._normalizer.normalize(command)
        self._semantic_validator.validate(normalized)
        return normalized

    def _build_validate_request(self, normalized_command, command_payload):
        request = _ValidateRequest()
        request.command_json = json.dumps(
            command_payload, ensure_ascii=True, separators=(",", ":")
        )
        request.primitive_type = normalized_command["primitive_type"]
        request.velocity_scale = normalized_command.get("velocity_scale", 0.0)
        if "target_pose_msg" in normalized_command:
            request.target_pose = normalized_command["target_pose_msg"]
        return request

    def _command_from_sanitized_json(self, sanitized_json, fallback_payload):
        if sanitized_json:
            return json.loads(sanitized_json)
        return fallback_payload

    def _prepare_execution_command(self, normalized_command):
        return normalized_command

    def _wait_for_future_without_spinning(self, future, timeout_sec):
        return True, future.result()


class _UnavailableActionClient:
    def server_is_ready(self):
        return False


class _SubmitNodeWithUnavailableAction:
    def __init__(self):
        self._execute_client = _UnavailableActionClient()
        self._react_plan_cache = {"plan-001": {"primitive_type": "HOME"}}


class _ActionClient:
    def __init__(self):
        self.sent_goal = None

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.sent_goal = goal
        return "goal-future"


class _SubmitNodeWithReadyAction:
    def __init__(self, plan):
        self._execute_client = _ActionClient()
        self._react_plan_cache = {"plan-001": plan}


def test_react_callback_path_does_not_use_nested_spin_until_future_complete():
    package_root = Path(__file__).resolve().parents[1]
    checked_paths = [
        package_root / "llm_gateway" / "llm_gateway_node.py",
        package_root / "llm_gateway" / "react_planner.py",
    ]
    offenders = [
        str(path.relative_to(package_root))
        for path in checked_paths
        if "spin_until_future_complete" in path.read_text(encoding="utf-8")
    ]
    assert offenders == []


def test_get_current_pose_uses_get_current_pose_srv_contract():
    result = GetCurrentPoseTool().invoke(
        {},
        AgentContext(state_injector=StateInjector(), ros_node=_PoseNode()),
    )
    assert result.ok is False
    assert result.error == "get_current_pose async client is not exposed to ReAct tools"


def test_get_current_pose_uses_node_pose_snapshot_contract():
    node = _PoseSnapshotNode()
    result = GetCurrentPoseTool().invoke(
        {"reference_frame": "base_link"},
        AgentContext(state_injector=StateInjector(), ros_node=node),
    )
    assert result.ok is True
    assert result.payload["pose"]["header"]["frame_id"] == "base_link"
    assert result.payload["pose"]["pose"]["position"]["x"] == 0.30
    assert node.requested_frame == "base_link"


def test_plan_motion_uses_validate_command_srv_contract():
    node = _PlanNode()
    result = PlanMotionTool().invoke(
        {
            "target": {"joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            "velocity_scale": 0.05,
            "acceleration_scale": 0.05,
        },
        AgentContext(state_injector=StateInjector(), ros_node=node),
    )
    assert result.ok is True
    request = node._validate_client.last_request
    assert request.command_json
    assert request.primitive_type == "PTP"
    assert request.velocity_scale == 0.05
    assert not hasattr(request, "command")
    assert result.payload["plan_id"] in node._react_plan_cache


def test_plan_motion_populates_pose_request_through_gateway_contract():
    node = _GatewayLikePlanNode()
    result = PlanMotionTool().invoke(
        {
            "target": {
                "header": {"frame_id": "base_link"},
                "pose": {
                    "position": {"x": 0.30, "y": 0.0, "z": 0.40},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "velocity_scale": 0.05,
            "acceleration_scale": 0.05,
        },
        AgentContext(state_injector=StateInjector(), ros_node=node),
    )
    assert result.ok is True
    request = node._validate_client.last_request
    assert request.primitive_type == "LIN"
    assert request.target_pose.position.x == 0.30
    plan = node._react_plan_cache[result.payload["plan_id"]]
    assert plan["goal"].target_pose.position.z == 0.40


def test_plan_motion_caches_sanitized_execution_command():
    node = _GatewayLikePlanNode()
    sanitized = {
        "primitive_type": "PTP",
        "joint_target": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
        "velocity_scale": 0.03,
        "acceleration_scale": 0.04,
        "planner_id": "PILZ_PTP",
    }
    node._validate_client.response = SimpleNamespace(
        valid=True,
        reason="OK",
        sanitized_json=json.dumps(sanitized),
    )
    result = PlanMotionTool().invoke(
        {
            "target": {"joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            "velocity_scale": 0.05,
            "acceleration_scale": 0.05,
        },
        AgentContext(state_injector=StateInjector(), ros_node=node),
    )
    assert result.ok is True
    plan = node._react_plan_cache[result.payload["plan_id"]]
    assert plan["command"]["velocity_scale"] == 0.03
    assert plan["command_payload"]["acceleration_scale"] == 0.04
    assert list(plan["goal"].joint_target) == pytest.approx(sanitized["joint_target"])


def test_plan_motion_generates_unique_plan_ids_for_repeated_command():
    node = _GatewayLikePlanNode()
    args = {
        "target": {"joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        "velocity_scale": 0.05,
        "acceleration_scale": 0.05,
    }

    first = PlanMotionTool().invoke(
        args,
        AgentContext(state_injector=StateInjector(), ros_node=node),
    )
    second = PlanMotionTool().invoke(
        args,
        AgentContext(state_injector=StateInjector(), ros_node=node),
    )

    assert first.ok is True
    assert second.ok is True
    assert first.payload["plan_id"] != second.payload["plan_id"]
    assert len(node._react_plan_cache) == 2


def test_plan_motion_fails_closed_without_ros_node():
    result = PlanMotionTool().invoke(
        {"target": {"joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}},
        AgentContext(state_injector=StateInjector()),
    )
    assert result.ok is False
    assert result.error == "ros_node not available in AgentContext"


def test_submit_motion_fails_closed_when_execute_motion_unavailable():
    result = SubmitMotionTool().invoke(
        {"plan_id": "plan-001"},
        AgentContext(
            state_injector=StateInjector(),
            ros_node=_SubmitNodeWithUnavailableAction(),
        ),
    )
    assert result.ok is False
    assert result.error == "execute_motion action server unavailable"


def test_submit_motion_returns_confirmation_handoff_without_sending_goal():
    goal = SimpleNamespace(primitive_type="HOME")
    node = _SubmitNodeWithReadyAction(
        {"goal": goal, "command": {"primitive_type": "HOME"}}
    )
    result = SubmitMotionTool().invoke(
        {"plan_id": "plan-001"},
        AgentContext(
            state_injector=StateInjector(),
            ros_node=node,
        ),
    )
    assert result.ok is True
    assert result.payload["status"] == "READY_FOR_CONFIRM"
    assert result.payload["plan_id"] == "plan-001"
    assert result.payload["command"]["primitive_type"] == "HOME"
    assert node._execute_client.sent_goal is None


def test_submit_motion_description_matches_confirmation_handoff_contract():
    description = SubmitMotionTool.description.lower()

    assert "confirmation" in description
    assert "without executing" in description


def test_submit_motion_fails_closed_without_ros_node():
    result = SubmitMotionTool().invoke(
        {"plan_id": "test-plan-001"},
        AgentContext(state_injector=StateInjector()),
    )
    assert result.ok is False
    assert result.error == "ros_node not available in AgentContext"


def test_query_perception_uses_live_ros_query_when_available():
    node = _PerceptionQueryNode(
        {
            "ok": True,
            "payload": {
                "detections": [
                    {
                        "class_id": "red_circle",
                        "position": {"x": 0.32, "y": 0.05, "z": 0.30},
                    }
                ]
            },
        }
    )

    result = QueryPerceptionTool().invoke(
        {"class_filter": "red_circle"},
        AgentContext(state_injector=StateInjector(), ros_node=node),
    )

    assert result.ok is True
    assert node.last_args == {"class_filter": "red_circle"}
    assert result.payload["detections"][0]["class_id"] == "red_circle"


def test_query_perception_fails_closed_when_live_ros_query_rejects():
    node = _PerceptionQueryNode(
        {
            "ok": False,
            "error": "calibration_invalid: calibration_date missing",
            "payload": None,
        }
    )

    result = QueryPerceptionTool().invoke(
        {},
        AgentContext(state_injector=StateInjector(), ros_node=node),
    )

    assert result.ok is False
    assert "calibration_invalid" in result.error
