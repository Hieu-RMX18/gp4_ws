# LLM Gateway — Test Environment Guide

## Two-Tier Test Model

The test suite is designed to run in two environment tiers. Every test is always
**collected** by pytest; nothing is silently excluded.

### Tier 1 — Source-only / lightweight mode

**What you need:** `rclpy` installed (system **ros-humble-rclpy** package).  
**What you do NOT need:** `colcon build`, sourced workspace, `interfaces` package.

This tier covers:

| Test file                         | Focus                                         |
| --------------------------------- | --------------------------------------------- |
| `test_core_pipeline.py`           | Parser, execution command helpers, workplane hydration |
| `test_contracts.py`               | Schema, semantic IR, and cross-layer contract checks |
| `test_normalizer.py`              | Normalization defaults                        |
| `test_semantic_validator.py`      | Semantic safety rules                         |
| `test_intent_router.py`           | Semantic IR → primitive routing               |
| `test_drawing.py`                 | Drawing geometry, draw_shape, draw_text, stroke font |
| `test_sequence_validator.py`      | Sequence prevalidation logic                  |
| `test_factory_task_prompt.py`     | FactoryTask system prompt contract tests      |
| `test_react_agent.py`             | ReAct agent loop, state injector, iteration budget |
| `test_station_scene_graph.py`     | Station map loader, alias resolver, VERIFY_CONFIG guards |
| `test_composite_tools.py`         | Composite ReAct tools, candidate poses, postconditions, MTC selector, verify_grasp |
| `test_scene_cache.py`             | Scene snapshot cache: TTL, invalidation, world-change |
| `test_gripper_adapter.py`         | Gripper adapter fail-closed behavior |
| `test_factory_task_contract.py`   | FactoryTask schema, compiler, runtime-control sentinel, policy decisions |
| `test_pick_white_workpiece_sim.py`| Sim integration: pick/place goal compiler and mocked scene |
| `test_react_tools.py`             | Source-level ReAct tool safety contracts      |
| `test_react_gateway_pipeline.py`  | ReAct-to-gateway contract and FactoryTask repair paths |
| `test_llm_backend.py`             | LLM backend config and retry behavior         |
| `test_get_pose.py` (Tier 1 tests) | GET_POSE schema, normalizer, semantic, parser |

**How to run:**

```bash
cd src/llm_gateway
python3 -m pytest
```

Or, to explicitly exclude ros_integration tests:

```bash
python3 -m pytest -m "not ros_integration"
```

### Tier 2 — Colcon / built-workspace mode

**What you need:** Built workspace with `interfaces` package on `PYTHONPATH`.

This tier adds:

| Test file                               | Focus                                          |
| --------------------------------------- | ---------------------------------------------- |
| `test_integration.py`                   | Full LLMGatewayNode pipeline (mocked services) |
| `test_get_pose.py` (`@ros_integration`) | GET_POSE gateway routing, fail-closed paths    |

**How to run:**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/llm_gateway
python3 -m pytest
```

Or, to run ONLY the integration tests:

```bash
python3 -m pytest -m ros_integration
```

## Skip Policy

- **No test is silently excluded.** All tests are collected by pytest.
- Tests requiring `interfaces` are skipped with an explicit reason:
  `SKIPPED: requires colcon-sourced workspace with built interfaces`
- Skip reasons are always visible in output (`-rs` is configured in `setup.cfg`).
- **No real failure is converted to a skip.** Skips are conditional on import
  availability only, not on test outcomes.

## Markers

| Marker            | Meaning                                                           |
| ----------------- | ----------------------------------------------------------------- |
| `ros_integration` | Requires colcon-sourced workspace with built `interfaces` package |

## Expected Results

### Source-only mode

All source-only tests should collect. Tests that require generated ROS
interfaces are skipped with reason:
`requires colcon-sourced workspace with built interfaces`.

### Full colcon mode

All tests pass with no interface-availability skips, assuming ROS services are
mocked correctly.
