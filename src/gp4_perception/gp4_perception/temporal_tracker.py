"""Temporal voting tracker for 3D detections.

Suppresses flicker by requiring a detection to appear in a stable position
across several consecutive frames before it is published. Operates on
``vision_msgs/Detection3D``-shaped objects via the access paths
``results[0].hypothesis.class_id`` and ``results[0].pose.pose.position``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np


def _centroid(detection: Any) -> np.ndarray:
    pos = detection.results[0].pose.pose.position
    return np.array([pos.x, pos.y, pos.z], dtype=np.float64)


def _class_id(detection: Any) -> str:
    return str(detection.results[0].hypothesis.class_id)


@dataclass
class _Track:
    class_id: str
    centroid: np.ndarray
    hits: deque = field(default_factory=deque)
    last_detection: Any | None = None
    missed_frames: int = 0
    confirmed: bool = False

    def score(self, window: int) -> float:
        return sum(self.hits) / window


class TemporalTracker:
    """Sliding-window detection stabiliser.

    A detection is returned only once its track has accumulated ``min_hits``
    hits within the last ``window_frames`` frames, and only while consecutive
    matches stay within ``jitter_max_mm`` of the track centroid.

    Confirmed tracks survive up to ``miss_tolerance_frames`` consecutive missed
    frames before being evicted (hysteresis).
    """

    def __init__(
        self,
        window_frames: int = 5,
        min_hits: int = 3,
        jitter_max_mm: float = 15.0,
        miss_tolerance_frames: int = 2,
    ) -> None:
        self._window = int(window_frames)
        self._min_hits = int(min_hits)
        self._jitter_max_m = float(jitter_max_mm) / 1000.0
        self._miss_tolerance = int(miss_tolerance_frames)
        self._tracks: list[_Track] = []

    def update(self, detections: list[Any]) -> list[tuple[Any, float]]:
        """Advance one frame; return ``(detection, temporal_score)`` for
        confirmed tracks, including those coasting through missed frames."""
        matched_tracks: set[int] = set()

        for det in detections:
            cls = _class_id(det)
            cen = _centroid(det)
            track = self._match(cls, cen, matched_tracks)
            if track is None:
                track = _Track(class_id=cls, centroid=cen)
                self._tracks.append(track)
            track.centroid = cen
            track.last_detection = det
            track.missed_frames = 0
            track.hits.append(1)
            matched_tracks.add(id(track))

        # Tracks not matched this frame record a miss.
        for track in self._tracks:
            if id(track) not in matched_tracks:
                track.hits.append(0)
                track.missed_frames += 1

        # Trim each track's window.
        for track in self._tracks:
            while len(track.hits) > self._window:
                track.hits.popleft()

        # Update confirmed status.
        for track in self._tracks:
            if sum(track.hits) >= self._min_hits:
                track.confirmed = True

        # Evict dead tracks:
        # - Unconfirmed: keep while sum(hits) > 0 (still accumulating)
        # - Confirmed: keep while missed_frames <= miss_tolerance
        self._tracks = [
            t for t in self._tracks
            if (t.confirmed and t.missed_frames <= self._miss_tolerance)
            or (not t.confirmed and sum(t.hits) > 0)
        ]

        # Emit all confirmed tracks that have a detection to report.
        results: list[tuple[Any, float]] = []
        for track in self._tracks:
            if track.confirmed and track.last_detection is not None:
                results.append((track.last_detection, track.score(self._window)))
        return results

    def _match(
        self, cls: str, cen: np.ndarray, already_matched: set[int]
    ) -> _Track | None:
        best: _Track | None = None
        best_dist = self._jitter_max_m
        for track in self._tracks:
            if track.class_id != cls or id(track) in already_matched:
                continue
            dist = float(np.linalg.norm(track.centroid - cen))
            if dist <= best_dist:
                best_dist = dist
                best = track
        return best
