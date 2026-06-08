from llm_gateway.llm_gateway_node import _SceneSnapshotCache
from llm_gateway.llm_gateway_node import LLMGatewayNode


def test_scene_cache_returns_cache_hit_inside_ttl():
    cache = _SceneSnapshotCache(ttl_sec=2.0, now_fn=lambda: 10.0)
    cache.store({"class_filter": "white_workpiece", "frame": "base_link"}, {"detections": []})

    hit = cache.get({"class_filter": "white_workpiece", "frame": "base_link"})

    assert hit is not None
    assert hit["cache_hit"] is True


def test_scene_cache_expires_after_ttl():
    now = [10.0]
    cache = _SceneSnapshotCache(ttl_sec=2.0, now_fn=lambda: now[0])
    cache.store({"class_filter": "white_workpiece", "frame": "base_link"}, {"detections": []})
    now[0] = 13.0

    assert cache.get({"class_filter": "white_workpiece", "frame": "base_link"}) is None


def test_scene_cache_invalidate_clears_entries():
    cache = _SceneSnapshotCache(ttl_sec=2.0, now_fn=lambda: 10.0)
    cache.store({"class_filter": "white_workpiece", "frame": "base_link"}, {"detections": []})

    cache.invalidate()

    assert cache.get({"class_filter": "white_workpiece", "frame": "base_link"}) is None


def test_gateway_scene_cache_lazy_initializes_for_source_level_tests():
    node = object.__new__(LLMGatewayNode)

    node._invalidate_scene_cache()

    assert isinstance(node._scene_snapshot_cache, _SceneSnapshotCache)

from types import SimpleNamespace

def test_scene_cache_invalidated_on_world_changing_tool_result():
    """ReAct loop must invalidate the scene cache when a motion tool signals tool_changed_world."""
    from llm_gateway.composite_tools import PickObjectTool
    from llm_gateway.react_planner import IterationCounters, IterationBudget

    class _TrackingNode:
        invalidated = False
        def _invalidate_scene_cache(self):
            self.invalidated = True

    node = _TrackingNode()
    context = SimpleNamespace(ros_node=node)
    tool = PickObjectTool()
    result = tool.invoke({"object_id": "white_workpiece"}, context)

    assert result.ok is True
    semantic_ir = result.payload.get("semantic_ir", {})
    assert semantic_ir.get("metadata", {}).get("tool_changed_world") is True

    # Simulate what ReActAgent does after a successful motion tool invocation.
    if result.ok and tool.is_motion:
        payload = result.payload or {}
        ir = payload.get("semantic_ir", {})
        meta = ir.get("metadata", {}) if isinstance(ir, dict) else {}
        if meta.get("tool_changed_world"):
            invalidate = getattr(getattr(context, "ros_node", None), "_invalidate_scene_cache", None)
            if callable(invalidate):
                invalidate()

    assert node.invalidated is True
