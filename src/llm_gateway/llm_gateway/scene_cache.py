from typing import Any, Dict, List, Optional
import time

class _SceneSnapshotCache:
    def __init__(self, ttl_sec: float, now_fn=time.monotonic):
        self._ttl_sec = float(ttl_sec)
        self._now_fn = now_fn
        self._entries: Dict[tuple[str, str], tuple[float, Dict[str, Any]]] = {}

    def get(self, args: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        key = self._key(args)
        entry = self._entries.get(key)
        if entry is None:
            return None
        stamp, payload = entry
        if self._now_fn() - stamp > self._ttl_sec:
            self._entries.pop(key, None)
            return None
        cached = dict(payload)
        cached["cache_hit"] = True
        return cached

    def store(self, args: Dict[str, Any], payload: Dict[str, Any]) -> None:
        stored = dict(payload)
        stored["cache_hit"] = False
        self._entries[self._key(args)] = (self._now_fn(), stored)

    def snapshots(self) -> List[Dict[str, Any]]:
        snapshots: List[Dict[str, Any]] = []
        for class_filter, frame in list(self._entries.keys()):
            payload = self.get({"class_filter": class_filter, "frame": frame})
            if payload is not None:
                snapshots.append(payload)
        return snapshots

    def invalidate(self) -> None:
        self._entries.clear()

    @staticmethod
    def _key(args: Dict[str, Any]) -> tuple[str, str]:
        return (
            str(args.get("class_filter") or ""),
            str(args.get("frame") or "base_link"),
        )

class SceneCacheMixin:
    def _get_scene_snapshot_cache(self) -> _SceneSnapshotCache:
        if not hasattr(self, "_scene_snapshot_cache_instance"):
            self._scene_snapshot_cache_instance = _SceneSnapshotCache(ttl_sec=2.0)
        return self._scene_snapshot_cache_instance

    def _cache_current_pose_snapshot(
        self, reference_frame: str, pose: Any
    ) -> Dict[str, Any]:
        frame = str(reference_frame or "base_link")
        pose_data = {
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }
        if not hasattr(self, "_latest_pose_by_frame"):
            self._latest_pose_by_frame = {}
        self._latest_pose_by_frame[frame] = {
            "pose": pose_data,
            "timestamp": time.monotonic(),
        }
        return pose_data

    def _get_cached_current_pose_snapshot(
        self, reference_frame: str
    ) -> Optional[Dict[str, Any]]:
        frame = str(reference_frame or "base_link")
        cache = getattr(self, "_latest_pose_by_frame", {})
        entry = cache.get(frame) if isinstance(cache, dict) else None
        if not isinstance(entry, dict):
            return None
        timestamp = entry.get("timestamp")
        pose = entry.get("pose")
        ttl = float(getattr(self, "_current_pose_cache_ttl_sec", 5.0))
        if not isinstance(timestamp, (int, float)) or time.monotonic() - timestamp > ttl:
            return None
        if not isinstance(pose, dict):
            return None
        position = pose.get("position")
        orientation = pose.get("orientation")
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            return None
        return {
            "position": dict(position),
            "orientation": dict(orientation),
        }
