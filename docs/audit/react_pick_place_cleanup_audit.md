# ReAct Pick/Place Cleanup Audit

Date: 2026-06-08

`src/llm_gateway/llm_gateway/drawing_geometry.py` and `src/llm_gateway/config/macro_policy.yaml` remain in the repository during the pick/place behavior work. Deletion requires a separate cleanup change with call-graph evidence and tests for drawing regressions.

## 2026-06-09 cleanup check

Keep both files. Current code still imports `llm_gateway.drawing_geometry` from `intent_engine.py`, loads `macro_policy.yaml` through `load_macro_policy`, and exercises both through drawing, contract, intent-router, ReAct gateway, and HMI supervisor tests. Deleting either file would remove active `draw_shape` / `draw_text` behavior rather than stale code.
