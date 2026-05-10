"""Algorithmic unit tests for scene_processor pipeline functions."""

import time
from datetime import datetime, timezone

import numpy as np

from gp4_perception.scene_processor import (
    SceneProcessor,
    _detection_class_id,
    _euclidean_clusters,
    _filter_detections,
    _pca_bbox,
    _ransac_plane,
    _roi_crop,
    _transform_points,
    _voxel_downsample,
)
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
                )()
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

    def test_depth_quality_records_out_of_range_sample(self):
        processor = object.__new__(SceneProcessor)
        processor._depth_noise_samples_mm = []
        processor._depth_in_range_samples = []
        processor._breakpoints = [
            {"distance_m": 0.3, "noise_mm_max": 2.0},
            {"distance_m": 0.8, "noise_mm_max": 3.0},
        ]

        processor._record_depth_quality(distance_m=0.5, noise_mm=8.0)

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
                raise AssertionError("uncalibrated detections must not be published")

        processor = object.__new__(SceneProcessor)
        processor._last_detections = [(time.time(), object())]
        processor._published_collision_ids = set()
        processor._ttl = 2.0
        processor._det_pub = FailingPublisher()
        processor._collision_pub = FailingPublisher()
        processor._calibration_status = lambda: (False, "calibration_invalid", "", 0.0)

        processor._publish_detections()

        assert processor._last_detections == []

    def test_uncalibrated_publish_removes_advertised_collision_objects(self):
        published = []

        class FailingDetectionPublisher:
            def publish(self, _msg):
                raise AssertionError("uncalibrated detections must not be published")

        class CollisionPublisher:
            def publish(self, msg):
                published.append(msg)

        processor = object.__new__(SceneProcessor)
        processor._last_detections = [(time.time(), object())]
        processor._published_collision_ids = {"perception_obj_0", "perception_obj_2"}
        processor._ttl = 2.0
        processor._det_pub = FailingDetectionPublisher()
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
