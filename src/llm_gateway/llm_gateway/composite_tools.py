"""Backward-compatibility re-export shim.

All tool logic has been consolidated into ``llm_gateway.factory_task`` and
gripper hardware adapters into ``llm_gateway.gripper_adapter`` per the
factory pipeline spec §3. This module re-exports every public symbol so
that existing consumers continue to work until they are updated (G5/G6).

Do NOT add new code here — edit ``factory_task.py`` instead.
"""
# fmt: off
from llm_gateway.factory_task import (  # noqa: F401
    CandidatePoseRequest,
    CandidatePoseResult,
    ToolResult,
    PostconditionVerifier,
    VerificationResult,
    EmitSequenceTool,
    RefreshSceneTool,
    PickObjectTool,
    ApproachObjectTool,
    PlaceObjectTool,
    VerifyPostconditionTool,
    VerifyGraspTool,
    mtc_select,
    generate_candidate_poses,
)
from llm_gateway.gripper_adapter import (  # noqa: F401
    GripperConfig,
    GripperIoAdapter,
    GripperResult,
)
# fmt: on
