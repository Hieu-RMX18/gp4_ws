from llm_gateway.runtime_skill_executor import RuntimeSkillExecutor
from llm_gateway.task_runtime import RuntimeStepResult


class _FakeNodeDeps:
    def semantic_ir_for_skill(self, name, args): return {"intent": name}
    def validate_and_dispatch(self, semantic_ir):
        return RuntimeStepResult(success=True)


def test_executor_grounds_validates_dispatches():
    ex = RuntimeSkillExecutor(_FakeNodeDeps())
    result = ex("pick_object", {"object_id": "w"})
    assert result.success is True
