"""Temporal voting: sliding-window stability, jitter rejection,
class consistency."""

from gp4_perception.temporal_tracker import TemporalTracker


def _det(class_id: str, x: float, y: float, z: float):
    """Build a minimal stand-in matching Detection3D access paths used by
    the tracker: results[0].hypothesis.class_id and results[0].pose.pose.position."""
    position = type("Point", (), {"x": x, "y": y, "z": z})()
    pose = type("Pose", (), {"position": position})()
    pose_cov = type("PoseWithCovariance", (), {"pose": pose})()
    hyp = type("Hypothesis", (), {"class_id": class_id, "score": 0.8})()
    result = type("Result", (), {"hypothesis": hyp, "pose": pose_cov})()
    return type("Detection3D", (), {"results": [result]})()


class TestTemporalTracker:
    def test_stable_object_published_after_min_hits(self):
        tr = TemporalTracker(window_frames=5, min_hits=3, jitter_max_mm=15.0)
        out1 = tr.update([_det("red_box", 0.3, 0.0, 0.1)])
        out2 = tr.update([_det("red_box", 0.3, 0.0, 0.1)])
        assert out1 == [] and out2 == []  # below min_hits
        out3 = tr.update([_det("red_box", 0.3, 0.0, 0.1)])
        assert len(out3) == 1
        _, score = out3[0]
        assert score >= 3 / 5

    def test_flicker_below_min_hits_not_published(self):
        tr = TemporalTracker(window_frames=5, min_hits=3, jitter_max_mm=15.0)
        out1 = tr.update([_det("red_box", 0.3, 0.0, 0.1)])
        out2 = tr.update([_det("red_box", 0.3, 0.0, 0.1)])
        assert out1 == []
        assert out2 == []

    def test_jittery_centroid_never_stable(self):
        tr = TemporalTracker(window_frames=5, min_hits=3, jitter_max_mm=15.0)
        # Jumps 50 mm each frame → never matches the same track.
        published = []
        for k in range(6):
            published += tr.update([_det("red_box", 0.3 + 0.05 * k, 0.0, 0.1)])
        assert published == []

    def test_class_change_starts_new_track(self):
        tr = TemporalTracker(window_frames=5, min_hits=2, jitter_max_mm=15.0)
        tr.update([_det("red_box", 0.3, 0.0, 0.1)])
        # Same location, different class → must not inherit hits.
        out = tr.update([_det("apple", 0.3, 0.0, 0.1)])
        assert out == []

    def test_score_increases_with_hits(self):
        tr = TemporalTracker(window_frames=5, min_hits=1, jitter_max_mm=15.0)
        s1 = tr.update([_det("red_box", 0.3, 0.0, 0.1)])[0][1]
        s2 = tr.update([_det("red_box", 0.3, 0.0, 0.1)])[0][1]
        assert s2 > s1

    def test_small_jitter_within_threshold_stays_matched(self):
        tr = TemporalTracker(window_frames=5, min_hits=3, jitter_max_mm=15.0)
        # 10 mm steps < 15 mm threshold → same track.
        tr.update([_det("red_box", 0.300, 0.0, 0.1)])
        tr.update([_det("red_box", 0.310, 0.0, 0.1)])
        out = tr.update([_det("red_box", 0.300, 0.0, 0.1)])
        assert len(out) == 1
