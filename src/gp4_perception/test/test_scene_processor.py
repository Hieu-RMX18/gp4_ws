"""Algorithmic unit tests for scene_processor pipeline functions."""

import time
from datetime import datetime, timezone

import numpy as np

from gp4_perception.scene_processor import (
    SceneProcessor,
    _contour_filter,
    _detection_class_id,
    _euclidean_clusters,
    _filter_detections,
    _pca_bbox,
    _ransac_plane,
    _roi_crop,
    _semantic_class_id,
    _transform_points,
    _voxel_downsample,
)
from gp4_perception.scene_geometry import _display_name
from interfaces.srv import GetObjectPositions
from moveit_msgs.msg import CollisionObject


class TestRoiCrop:
    def test_crops_outside(self):
        pts = np.array(
            [
                [0.30, 0.0, 0.1],  # inside
                [0.6, 0.0, 0.1],  # outside x max
                [0.3, -0.4, 0.1],  # outside y min
            ],
            dtype=np.float32,
        )
        bbox = {"x": [0.2, 0.55], "y": [-0.30, 0.30], "z": [0.00, 0.40]}
        cropped = _roi_crop(pts, bbox)
        assert len(cropped) == 1
        np.testing.assert_array_equal(cropped[0], pts[0])


class TestVoxelDownsample:
    def test_reduces_points(self):
        pts = np.random.rand(1000, 3).astype(np.float32) * 0.1
        down = _voxel_downsample(pts, voxel_size=0.02)
        assert len(down) < len(pts)

    def test_empty_input(self):
        pts = np.empty((0, 3), dtype=np.float32)
        down = _voxel_downsample(pts, voxel_size=0.01)
        assert len(down) == 0


class TestRansacPlane:
    def test_removes_largest_plane(self):
        # Large plane at z=0
        plane = np.random.rand(500, 3).astype(np.float32)
        plane[:, 2] = 0.0
        # Small cluster above
        cluster = np.random.rand(50, 3).astype(np.float32)
        cluster[:, 2] = 0.1
        pts = np.vstack([plane, cluster])
        inliers, outliers = _ransac_plane(pts, threshold=0.005)
        assert len(inliers) > len(outliers)
        assert len(outliers) > 0

    def test_empty_input(self):
        pts = np.empty((0, 3), dtype=np.float32)
        inliers, outliers = _ransac_plane(pts, threshold=0.005)
        assert len(inliers) == 0
        assert len(outliers) == 0


class TestEuclideanClusters:
    def test_finds_two_clusters(self):
        c1 = np.random.randn(100, 3).astype(np.float32) * 0.01 + np.array(
            [0.3, 0.0, 0.1]
        )
        c2 = np.random.randn(100, 3).astype(np.float32) * 0.01 + np.array(
            [0.4, 0.0, 0.1]
        )
        pts = np.vstack([c1, c2])
        clusters = _euclidean_clusters(pts, tolerance=0.03, min_size=20, max_size=500)
        assert len(clusters) == 2

    def test_noise_only(self):
        pts = np.random.rand(20, 3).astype(np.float32) * 0.1
        clusters = _euclidean_clusters(pts, tolerance=0.01, min_size=10, max_size=500)
        assert len(clusters) == 0


class TestPcaBbox:
    def test_box_on_axis_aligned_cube(self):
        pts = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.1, 0.0, 0.0],
                [0.0, 0.05, 0.0],
                [0.0, 0.0, 0.02],
            ],
            dtype=np.float32,
        )
        pose, dims, shape_class = _pca_bbox(pts)
        assert dims.x > 0
        assert dims.y > 0
        assert dims.z > 0
        assert abs(pose.position.x - 0.025) < 0.01
        assert shape_class in ("sphere", "cylinder", "box", "flat", "unknown")


class TestFrameTransform:
    def test_transforms_camera_points_to_base_link(self):
        transform = type("TransformStamped", (), {})()
        transform.transform = type("Transform", (), {})()
        transform.transform.translation = type(
            "Translation",
            (),
            {"x": 0.5, "y": -0.1, "z": 0.2},
        )()
        transform.transform.rotation = type(
            "Rotation",
            (),
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        )()
        pts = np.array([[0.1, 0.2, 0.3]], dtype=np.float32)

        transformed = _transform_points(pts, transform)

        np.testing.assert_allclose(transformed, [[0.6, 0.1, 0.5]], atol=1e-6)

    def test_non_base_cloud_without_transform_is_rejected_before_roi(self, monkeypatch):
        from gp4_perception import scene_processor

        processor = object.__new__(SceneProcessor)
        processor._last_detections = []
        processor._published_collision_ids = set()
        processor._collision_pub = type(
            "Publisher", (), {"publish": lambda *_args: None}
        )()
        processor.get_logger = lambda: __import__("logging").getLogger("test")
        processor._calibration_status = lambda: (True, "", "2026-05-09T00:00:00Z", 1.0)
        processor._tf_buffer = type(
            "TfBuffer",
            (),
            {
                "lookup_transform": lambda *_args, **_kwargs: (_ for _ in ()).throw(
                    RuntimeError("missing transform")
                )
            },
        )()

        def fail_if_roi_receives_camera_points(_pts, _bbox):
            raise AssertionError("camera-frame points must not reach ROI crop")

        monkeypatch.setattr(
            scene_processor,
            "_read_xyz_rgb",
            lambda _cloud: (np.array([[0.1, 0.2, 0.3]], dtype=np.float32), None),
        )
        monkeypatch.setattr(
            scene_processor, "_roi_crop", fail_if_roi_receives_camera_points
        )

        fake_cloud = type(
            "Cloud",
            (),
            {
                "header": type(
                    "Header",
                    (),
                    {"frame_id": "camera_color_optical_frame", "stamp": object()},
                )(),
                "fields": [],
                "point_step": 0,
                "is_bigendian": False,
            },
        )()
        processor._on_synced(fake_cloud, object())

        assert processor._last_detections == []


class TestDetectionFiltering:
    def test_filters_by_detection_class_id(self):
        detection = type("Detection", (), {})()
        hypothesis = type("Hypothesis", (), {"class_id": "red_circle", "score": 1.0})()
        result = type("Result", (), {"hypothesis": hypothesis})()
        detection.results = [result]

        assert _detection_class_id(detection) == "red_circle"
        assert _filter_detections([detection], "red_circle") == [detection]
        assert _filter_detections([detection], "blue_square") == []

    def test_empty_class_filter_returns_all_detections(self):
        detections = [object(), object()]

        assert _filter_detections(detections, "") == detections


class TestPerceptionContracts:
    def test_get_object_positions_response_has_calibration_and_depth_metadata(self):
        response = GetObjectPositions.Response()

        assert hasattr(response, "calibration_valid")
        assert hasattr(response, "calibration_date_iso")
        assert hasattr(response, "calibration_age_days")
        assert hasattr(response, "stamp")
        assert hasattr(response, "depth_noise_mm_p95")
        assert hasattr(response, "depth_in_range")

    def test_depth_quality_is_not_ready_without_samples(self):
        processor = object.__new__(SceneProcessor)
        processor._depth_noise_samples_mm = []
        processor._depth_in_range_samples = []

        assert processor._depth_in_range() is False

    def test_depth_tier_records_degraded_as_not_in_range(self):
        processor = object.__new__(SceneProcessor)
        processor._depth_noise_samples_mm = []
        processor._depth_in_range_samples = []

        processor._record_depth_tier(noise_mm=8.0, quality="DEGRADED_DEPTH")

        assert processor._depth_noise_p95() == 8.0
        assert processor._depth_in_range() is False

    def test_uncalibrated_processor_does_not_read_or_publish_scene(self, monkeypatch):
        from gp4_perception import scene_processor

        processor = object.__new__(SceneProcessor)
        processor._last_detections = []
        processor._calibration_status = lambda: (False, "calibration_invalid", "", 0.0)

        def fail_if_point_cloud_is_read(_cloud):
            raise AssertionError("uncalibrated scene processing must fail closed")

        monkeypatch.setattr(
            scene_processor, "_read_xyz_rgb", fail_if_point_cloud_is_read
        )

        fake_cloud = type("Cloud", (), {"header": object()})()
        processor._on_synced(fake_cloud, object())

        assert processor._last_detections == []

    def test_uncalibrated_publish_clears_cached_scene(self):
        class FailingPublisher:
            def publish(self, _msg):
                raise AssertionError("uncalibrated data must not be published")

        processor = object.__new__(SceneProcessor)
        processor._last_detections = [(time.time(), object())]
        processor._published_collision_ids = set()
        processor._ttl = 2.0
        processor._collision_pub = FailingPublisher()
        processor._calibration_status = lambda: (False, "calibration_invalid", "", 0.0)

        processor._publish_detections()

        assert processor._last_detections == []

    def test_uncalibrated_publish_removes_advertised_collision_objects(self):
        published = []

        class CollisionPublisher:
            def publish(self, msg):
                published.append(msg)

        processor = object.__new__(SceneProcessor)
        processor._last_detections = [(time.time(), object())]
        processor._published_collision_ids = {"perception_obj_0", "perception_obj_2"}
        processor._ttl = 2.0
        processor._collision_pub = CollisionPublisher()
        processor._calibration_status = lambda: (False, "calibration_invalid", "", 0.0)

        processor._publish_detections()

        assert {msg.id for msg in published} == {"perception_obj_0", "perception_obj_2"}
        assert all(msg.operation == CollisionObject.REMOVE for msg in published)
        assert processor._published_collision_ids == set()

    def test_object_query_rejects_when_depth_quality_is_not_ready(self):
        from builtin_interfaces.msg import Time

        processor = object.__new__(SceneProcessor)
        processor._calibration_status = lambda: (True, "", "2026-05-09T00:00:00Z", 1.0)
        processor._depth_noise_p95 = lambda: 0.0
        processor._depth_in_range = lambda: False
        processor._last_detections = []
        processor._last_stamp = type("Header", (), {"stamp": Time()})()

        request = type("Request", (), {"class_filter": ""})()
        response = GetObjectPositions.Response()

        result = processor._handle_get_object_positions(request, response)

        assert result.ok is False
        assert result.failure_reason == "depth_quality_invalid"
        assert result.detections == []

    def test_calibration_status_reuses_cache_until_extrinsics_mtime_changes(
        self,
        tmp_path,
        monkeypatch,
    ):
        from gp4_perception import scene_processor

        extrinsics_path = tmp_path / "extrinsics.yaml"
        extrinsics_path.write_text("hand_eye_extrinsics: {}\n")
        load_calls = []
        fresh_calibration = datetime.now(timezone.utc).isoformat()

        def fake_load_extrinsics(path):
            load_calls.append(path)
            return {
                "hand_eye_extrinsics": {
                    "calibration_date": fresh_calibration,
                    "reprojection_error_mm": 1.0,
                }
            }

        monkeypatch.setattr(scene_processor, "load_extrinsics", fake_load_extrinsics)

        processor = object.__new__(SceneProcessor)
        processor._extrinsics_path = extrinsics_path

        assert processor._calibration_status()[0] is True
        assert processor._calibration_status()[0] is True
        assert load_calls == [extrinsics_path]


class TestContourFilter:
    def test_solid_sphere_cluster_passes_contour_filter(self):
        """A dense, spherical cluster should pass the solidity check."""
        rng = np.random.default_rng(seed=42)
        # Generate a solid ball of points.
        pts = rng.normal(size=(300, 3)).astype(np.float32) * 0.02
        pts += np.array([0.3, 0.0, 0.1])
        keep, props = _contour_filter(pts)
        assert keep is True
        if props is not None:
            assert props["solidity"] >= 0.30

    def test_sparse_noise_cluster_may_be_rejected(self):
        """Random sparse noise often produces low-solidity contours."""
        rng = np.random.default_rng(seed=99)
        pts = rng.random(size=(200, 3)).astype(np.float32) * 0.5
        keep, props = _contour_filter(pts)
        # We don't assert rejection since random points *can* be solid,
        # but we verify the function runs without error and returns props.
        assert isinstance(keep, bool)

    def test_tiny_cluster_fails_open(self):
        """Clusters with <5 points cannot form a contour — kept (fail-open)."""
        pts = np.array(
            [[0.1, 0.0, 0.0], [0.11, 0.0, 0.0], [0.1, 0.01, 0.0]],
            dtype=np.float32,
        )
        keep, props = _contour_filter(pts)
        assert keep is True
        assert props is None

    def test_pca_bbox_accepts_contour_props(self):
        """_pca_bbox should accept contour_props without error."""
        rng = np.random.default_rng(seed=42)
        pts = rng.normal(size=(100, 3)).astype(np.float32) * 0.02
        pts += np.array([0.3, 0.0, 0.1])
        props = {"circularity": 0.9, "solidity": 0.95, "aspect_ratio": 1.1, "area_px": 500.0}
        pose, dims, shape_class = _pca_bbox(pts, props)
        assert shape_class == "sphere"


class _FakeDims:
    """Minimal stand-in for geometry_msgs.msg.Vector3 in tests."""

    def __init__(self, x: float = 0.04, y: float = 0.04, z: float = 0.04):
        self.x = x
        self.y = y
        self.z = z


class TestSemanticClassId:
    """Verify _semantic_class_id maps (color, shape, dims) → semantic name."""

    # --- Core 5 objects ---
    def test_red_sphere_is_apple(self):
        assert _semantic_class_id("red", "sphere", _FakeDims()) == "apple"

    def test_green_sphere_is_apple(self):
        assert _semantic_class_id("green", "sphere", _FakeDims()) == "apple"

    def test_yellow_sphere_is_yellow_ball(self):
        assert _semantic_class_id("yellow", "sphere", _FakeDims()) == "yellow_ball"

    def test_white_sphere_is_white_ball(self):
        assert _semantic_class_id("white", "sphere", _FakeDims()) == "white_ball"

    def test_red_box_is_red_box(self):
        assert _semantic_class_id("red", "box", _FakeDims()) == "red_box"

    def test_blue_box_is_blue_rectangle(self):
        assert _semantic_class_id("blue", "box", _FakeDims()) == "blue_rectangle"

    # --- Defensive: flat / unknown shape fallback ---
    def test_red_flat_is_red_box(self):
        assert _semantic_class_id("red", "flat", _FakeDims()) == "red_box"

    def test_red_unknown_is_red_box(self):
        assert _semantic_class_id("red", "unknown", _FakeDims()) == "red_box"

    def test_blue_flat_is_blue_rectangle(self):
        assert _semantic_class_id("blue", "flat", _FakeDims()) == "blue_rectangle"

    def test_blue_unknown_is_blue_rectangle(self):
        assert _semantic_class_id("blue", "unknown", _FakeDims()) == "blue_rectangle"

    # --- Fallback: unrecognised combos get colour_shape ---
    def test_yellow_box_fallback(self):
        assert _semantic_class_id("yellow", "box", _FakeDims()) == "yellow_box"

    def test_purple_cylinder_fallback(self):
        assert _semantic_class_id("purple", "cylinder", _FakeDims()) == "purple_cylinder"

    def test_unknown_color_returns_shape_only(self):
        assert _semantic_class_id("unknown", "sphere", _FakeDims()) == "sphere"

    # --- Edge cases ---
    def test_none_color_treated_as_unknown(self):
        assert _semantic_class_id(None, "box", _FakeDims()) == "box"

    def test_case_insensitive(self):
        assert _semantic_class_id("RED", "SPHERE", _FakeDims()) == "apple"


class TestDisplayName:
    """Verify _display_name returns human-readable strings."""

    def test_known_names(self):
        assert _display_name("apple") == "apple"
        assert _display_name("yellow_ball") == "yellow ball"
        assert _display_name("white_ball") == "white ball"
        assert _display_name("red_box") == "red box"
        assert _display_name("blue_rectangle") == "blue rectangle"

    def test_fallback_replaces_underscores(self):
        assert _display_name("purple_cylinder") == "purple cylinder"
        assert _display_name("orange_flat") == "orange flat"

    def test_single_word(self):
        assert _display_name("sphere") == "sphere"

