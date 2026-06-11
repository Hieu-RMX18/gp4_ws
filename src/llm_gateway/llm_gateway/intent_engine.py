"""Backward-compatibility re-export shim.

All content has been consolidated into ``llm_gateway.factory_task`` per the
factory pipeline spec §3. This module re-exports every public symbol so that
existing consumers (HMI backend, tests) continue to work without import
changes until they are updated (G5/G6).

Do NOT add new code here — edit ``factory_task.py`` instead.
"""
# fmt: off
from llm_gateway.factory_task import (  # noqa: F401
    GP4_JOINT_NAMES,
    GoalMapper,
    IntentRouter,
    LLMParser,
    Normalizer,
    RouteResult,
    SchemaValidator,
    SemanticValidator,
    SequenceValidationError,
    SequenceValidationResult,
    SequenceValidator,
    canonicalize_named_pose,
    command_from_sanitized_json,
    compile_strokes_to_commands,
    hydrate_draw_workplane,
    lift_points_to_poses,
    load_macro_policy,
    load_srdf_named_poses,
    normalize_joints,
    normalize_pose,
    parse_llm_output,
    prepare_execution_command,
    prepare_semantic_ir_for_routing,
    resolve_gp4_joint_index,
)
# fmt: on
