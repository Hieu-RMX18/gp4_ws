from typing import Any, Dict

from llm_gateway.task_runtime import RuntimeStepResult


class RuntimeSkillExecutor:
    def __init__(self, node_deps: Any):
        self._node_deps = node_deps

    def __call__(self, name: str, args: Dict[str, Any]) -> RuntimeStepResult:
        try:
            if name == "verify_grasp":
                if not hasattr(self._node_deps, "verify_grasp"):
                    return RuntimeStepResult(success=False, reason="verify_grasp not available on node dependencies")
                return self._node_deps.verify_grasp()
                
            semantic_ir = self._node_deps.semantic_ir_for_skill(name, args)
            return self._node_deps.validate_and_dispatch(semantic_ir)
        except Exception as exc:
            return RuntimeStepResult(success=False, reason=str(exc))
