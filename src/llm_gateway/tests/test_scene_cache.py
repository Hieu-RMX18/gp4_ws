from llm_gateway.scene_cache import _SceneSnapshotCache
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

    assert isinstance(node._get_scene_snapshot_cache(), _SceneSnapshotCache)
