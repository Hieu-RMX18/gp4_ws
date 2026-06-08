# ReAct Pick/Place Cleanup Audit

Date: 2026-06-08

`src/llm_gateway/llm_gateway/drawing_geometry.py` and `src/llm_gateway/config/macro_policy.yaml` remain in the repository during the pick/place behavior work. Deletion requires a separate cleanup change with call-graph evidence and tests for drawing regressions.
