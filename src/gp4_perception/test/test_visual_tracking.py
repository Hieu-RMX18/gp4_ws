from gp4_perception.visual_tracking import VisualDetectionTracker


def obj(class_id="red_box", bbox=(100, 100, 80, 60), confidence=0.8, z_m=0.4):
    x, y, w, h = bbox
    return {
        "class_id": class_id,
        "bbox": bbox,
        "center_uv": (x + w // 2, y + h // 2),
        "confidence": confidence,
        "distance_m": z_m,
    }


def test_visual_track_is_confirmed_after_two_hits():
    tracker = VisualDetectionTracker(min_hits=2, miss_tolerance_frames=2,
                                     max_centroid_distance_px=45.0, min_iou=0.15,
                                     ema_alpha=0.35)
    assert tracker.update([obj()]) == []
    assert len(tracker.update([obj(bbox=(102, 101, 80, 60))])) == 1


def test_confirmed_visual_track_survives_two_misses_for_display_only():
    tracker = VisualDetectionTracker(min_hits=2, miss_tolerance_frames=2,
                                     max_centroid_distance_px=45.0, min_iou=0.15,
                                     ema_alpha=0.35)
    tracker.update([obj()])
    tracker.update([obj()])
    first = tracker.update([])
    second = tracker.update([])
    expired = tracker.update([])
    assert first[0]["held_for_display"] is True
    assert second[0]["held_for_display"] is True
    assert expired == []
