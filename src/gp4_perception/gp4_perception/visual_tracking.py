"""Visual detection hysteresis tracker — pure Python, no ROS2 imports."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


def _iou(a: tuple, b: tuple) -> float:
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    ax2, ay2 = ax1 + aw, ay1 + ah
    bx2, by2 = bx1 + bw, by1 + bh
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def _dist(a: tuple, b: tuple) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class _VTrack:
    class_id: str
    bbox: tuple
    center_uv: tuple
    confidence: float
    distance_m: float
    hits: int = 0
    missed_frames: int = 0
    confirmed: bool = False
    last_obj: Optional[dict] = field(default=None, repr=False)


class VisualDetectionTracker:
    def __init__(
        self,
        min_hits: int = 2,
        miss_tolerance_frames: int = 2,
        max_centroid_distance_px: float = 45.0,
        min_iou: float = 0.15,
        ema_alpha: float = 0.35,
    ) -> None:
        self._min_hits = min_hits
        self._miss_tolerance = miss_tolerance_frames
        self._max_dist = max_centroid_distance_px
        self._min_iou = min_iou
        self._ema_alpha = ema_alpha
        self._tracks: list[_VTrack] = []

    def update(self, detections: list[dict]) -> list[dict]:
        alpha = self._ema_alpha
        unmatched_dets = list(range(len(detections)))
        matched_track_ids: set[int] = set()

        for ti, track in enumerate(self._tracks):
            best_det_i = None
            best_score = -1.0
            for di in unmatched_dets:
                det = detections[di]
                if det["class_id"] != track.class_id:
                    continue
                iou_val = _iou(track.bbox, det["bbox"])
                dist_val = _dist(track.center_uv, det["center_uv"])
                if iou_val >= self._min_iou or dist_val <= self._max_dist:
                    # prefer by iou then distance
                    score = iou_val - dist_val * 0.001
                    if score > best_score:
                        best_score = score
                        best_det_i = di

            if best_det_i is not None:
                det = detections[best_det_i]
                x, y, w, h = det["bbox"]
                ox, oy, ow, oh = track.bbox
                track.bbox = (
                    int(alpha * x + (1 - alpha) * ox),
                    int(alpha * y + (1 - alpha) * oy),
                    int(alpha * w + (1 - alpha) * ow),
                    int(alpha * h + (1 - alpha) * oh),
                )
                cx, cy = det["center_uv"]
                ocx, ocy = track.center_uv
                track.center_uv = (
                    int(alpha * cx + (1 - alpha) * ocx),
                    int(alpha * cy + (1 - alpha) * ocy),
                )
                track.confidence = alpha * det["confidence"] + (1 - alpha) * track.confidence
                track.distance_m = alpha * det["distance_m"] + (1 - alpha) * track.distance_m
                track.missed_frames = 0
                track.hits += 1
                if track.hits >= self._min_hits:
                    track.confirmed = True
                track.last_obj = dict(det)
                matched_track_ids.add(ti)
                unmatched_dets.remove(best_det_i)
            else:
                track.missed_frames += 1

        # new tracks for unmatched detections
        for di in unmatched_dets:
            det = detections[di]
            t = _VTrack(
                class_id=det["class_id"],
                bbox=det["bbox"],
                center_uv=det["center_uv"],
                confidence=det["confidence"],
                distance_m=det["distance_m"],
                hits=1,
                missed_frames=0,
                confirmed=False,
                last_obj=dict(det),
            )
            self._tracks.append(t)

        # eviction
        def _keep(t: _VTrack) -> bool:
            if not t.confirmed:
                return t.missed_frames == 0
            return t.missed_frames <= self._miss_tolerance

        self._tracks = [t for t in self._tracks if _keep(t)]

        # emit confirmed tracks
        results = []
        for t in self._tracks:
            if not t.confirmed:
                continue
            out = dict(t.last_obj)
            out["held_for_display"] = t.missed_frames > 0
            results.append(out)
        return results
