"""Scene processor — ROI crop, voxel downsample, RANSAC plane removal,
Euclidean clustering, PCA bounding box, publish detections + MoveIt collision objects.

Entry point: ros2 run gp4_perception scene_processor
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovariance
from interfaces.msg import PerceptionStatus
from interfaces.srv import GetObjectPositions
from moveit_msgs.msg import CollisionObject
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from message_filters import ApproximateTimeSynchronizer, Subscriber
from .query_perception_tool import load_extrinsics
from .safety_guards import (
    check_depth_noise,
    check_reprojection_error,
)
from .temporal_tracker import TemporalTracker
from .scene_geometry import (
    _confidence_score,
    _contour_filter,
    _depth_noise_at_centroid,
    _detection_class_id,
    _dominant_color_voting,
    _euclidean_cluster_indices,
    _euclidean_clusters,
    _filter_detections,
    _pca_bbox,
    _ransac_plane,
    _ransac_plane_fit,
    _read_xyz_rgb,
    _roi_crop,
    _semantic_class_id,
    _transform_points,
    _voxel_downsample,
    _voxel_downsample_indices,
)

_LOGGER = logging.getLogger(__name__)
MAX_DEPTH_NOISE_SAMPLES = 50
REPROJECTION_ERROR_MAX_MM = 5.0

__all__ = [
    "SceneProcessor",
    "_contour_filter",
    "_detection_class_id",
    "_euclidean_clusters",
    "_filter_detections",
    "_pca_bbox",
    "_ransac_plane",
    "_roi_crop",
    "_semantic_class_id",
    "_transform_points",
    "_voxel_downsample",
    "main",
]


class SceneProcessor(Node):
    """Synchronized point-cloud + camera-info processor."""

    def __init__(self) -> None:
        super().__init__("scene_processor")
        try:
            from ament_index_python.packages import get_package_share_directory

            share = Path(get_package_share_directory("gp4_perception")) / "config"
        except Exception:
            share = Path(__file__).resolve().parents[1] / "config"
        with open(share / "perception.yaml") as f:
            cfg = yaml.safe_load(f) or {}
        self._cfg = cfg.get("perception", {})
        self._bbox = self._cfg.get("workspace_bbox_m", {})
        self._voxel = float(self._cfg.get("voxel_size_m", 0.005))
        self._ransac_thresh = float(self._cfg.get("ransac_distance_threshold_m", 0.005))
        self._cluster_tol = float(self._cfg.get("cluster_tolerance_m", 0.02))
        self._cluster_min = int(self._cfg.get("cluster_min_size", 50))
        self._cluster_max = int(self._cfg.get("cluster_max_size", 5000))
        self._ransac_normal_z_min = float(
            self._cfg.get("ransac_plane_normal_z_min", 0.0)
        )
        self._min_publish_confidence = float(
            self._cfg.get("min_publish_confidence", 0.0)
        )
        self._tracker = TemporalTracker(
            window_frames=int(self._cfg.get("temporal_window_frames", 5)),
            min_hits=int(self._cfg.get("temporal_min_hits", 3)),
            jitter_max_mm=float(self._cfg.get("centroid_jitter_max_mm", 15.0)),
        )
        self._ttl = float(self._cfg.get("detection_ttl_s", 2.0))
        self._breakpoints = self._cfg.get("depth_noise", {}).get("breakpoints", [])
        self._extrapolation = self._cfg.get("depth_noise", {}).get("extrapolation", "reject")

        # Workspace bbox filter — can be toggled at runtime via:
        #   ros2 param set /scene_processor enable_bbox_filter false
        self.declare_parameter("enable_bbox_filter", True)
        self._enable_bbox_filter: bool = self.get_parameter(
            "enable_bbox_filter"
        ).get_parameter_value().bool_value
        self.add_on_set_parameters_callback(self._on_param_change)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Low-latency QoS profile with depth=1 to discard old messages and prevent queue buildup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._cloud_sub = Subscriber(
            self,
            PointCloud2,
            "/camera/depth/color/points",
            qos_profile=qos_profile,
        )
        self._info_sub = Subscriber(
            self,
            CameraInfo,
            "/camera/color/camera_info",
            qos_profile=qos_profile,
        )
        self._sync = ApproximateTimeSynchronizer(
            [self._cloud_sub, self._info_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._on_synced)

        self._last_transform = None

        self._det_pub = self.create_publisher(
            Detection3DArray, "/perception/detections", 10
        )
        self._collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self._timer = self.create_timer(0.2, self._publish_detections)  # 5 Hz
        self._last_detections: list[
            tuple[float, Detection3D]
        ] = []  # (timestamp, detection)
        self._published_collision_ids: set[str] = set()
        self._depth_noise_samples_mm: list[float] = []
        self._depth_in_range_samples: list[bool] = []
        self._last_stamp = Header()
        self._last_cloud_time = time.time()
        self._health_timer = self.create_timer(2.0, self._check_camera_health)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self._status_pub = self.create_publisher(
            PerceptionStatus, "/perception/status", 10
        )
        self._object_query_srv = self.create_service(
            GetObjectPositions,
            "/perception/get_object_positions",
            self._handle_get_object_positions,
        )
        self._extrinsics_path = share / "extrinsics.yaml"
        self._calibration_cache_mtime_ns: int | None = None
        self._calibration_cache_status: tuple[bool, str, str, float] | None = None

    def _on_param_change(self, params) -> list:
        """Handle runtime parameter changes (e.g. enable_bbox_filter toggle)."""
        from rcl_interfaces.msg import SetParametersResult

        for p in params:
            if p.name == "enable_bbox_filter":
                self._enable_bbox_filter = bool(p.value)
                self.get_logger().info(
                    "Workspace bbox filter %s",
                    "ENABLED" if self._enable_bbox_filter else "DISABLED",
                )
        return [SetParametersResult(successful=True)]

    def _check_camera_health(self) -> None:
        elapsed = time.time() - self._last_cloud_time
        if elapsed > 5.0:
            _LOGGER.warning(
                "No point cloud received for %.1f s — check camera connection or QoS match",
                elapsed,
            )

    def _on_synced(self, cloud: PointCloud2, _: CameraInfo) -> None:
        self._last_cloud_time = time.time()
        if not self._calibration_allows_scene_output():
            return

        # Read XYZ + optional RGB for color classification.
        xyz_all, rgb_all = _read_xyz_rgb(cloud)
        if xyz_all is None or len(xyz_all) == 0:
            _LOGGER.info("No points read from PointCloud2")
            return

        _LOGGER.info(f"_on_synced: received {len(xyz_all)} points from camera")

        pts = self._points_in_base_link(xyz_all, cloud)
        if pts is None or len(pts) == 0:
            _LOGGER.info("No points after transforming to base_link")
            self._remove_published_collision_objects()
            self._last_detections = []
            return

        # RGB stays index-aligned with pts (transform preserves point order),
        # so we slice rgb with the same indices at every pipeline step.
        rgb = rgb_all
        _LOGGER.info(f"Points in base_link: {len(pts)}")

        # 1. ROI crop (skipped when enable_bbox_filter is False)
        if self._bbox and self._enable_bbox_filter:
            mask = (
                (pts[:, 0] >= self._bbox["x"][0])
                & (pts[:, 0] <= self._bbox["x"][1])
                & (pts[:, 1] >= self._bbox["y"][0])
                & (pts[:, 1] <= self._bbox["y"][1])
                & (pts[:, 2] >= self._bbox["z"][0])
                & (pts[:, 2] <= self._bbox["z"][1])
            )
            pts = pts[mask]
            if rgb is not None:
                rgb = rgb[mask]

        _LOGGER.info(f"Points after ROI crop: {len(pts)} (bbox: {self._bbox})")
        if len(pts) == 0:
            return

        # 2. Voxel downsample (carry RGB by index)
        keep = _voxel_downsample_indices(pts, self._voxel)
        pts = pts[keep]
        if rgb is not None:
            rgb = rgb[keep]
        _LOGGER.info(f"Points after voxel downsample: {len(pts)}")

        # 3. RANSAC plane removal — only remove near-vertical-normal planes
        #    (table/floor); keep vertical object faces.
        _, out_idx, _ = _ransac_plane_fit(
            pts, self._ransac_thresh, normal_z_min=self._ransac_normal_z_min
        )
        pts = pts[out_idx]
        if rgb is not None:
            rgb = rgb[out_idx]
        _LOGGER.info(f"Points after RANSAC plane removal: {len(pts)}")
        if len(pts) == 0:
            return

        # 4. Euclidean clustering (index arrays so RGB stays aligned)
        cluster_indices = _euclidean_cluster_indices(
            pts, self._cluster_tol, self._cluster_min, self._cluster_max
        )
        _LOGGER.info(f"Number of clusters found: {len(cluster_indices)}")
        if not cluster_indices:
            return

        now = time.time()
        detections: list[Detection3D] = []
        det_factors: dict[int, tuple[float, float, float]] = {}
        for i, cluster_idx in enumerate(cluster_indices):
            cluster = pts[cluster_idx]
            cluster_rgb = rgb[cluster_idx] if rgb is not None else None
            # 4a. Contour filter — reject fragmented/noisy clusters.
            contour_ok, contour_props = _contour_filter(cluster)
            if not contour_ok:
                _LOGGER.info(
                    f"Cluster {i} rejected by contour filter (solidity={(contour_props['solidity'] if contour_props else 0.0):.2f})"
                )
                continue

            noise_mm = _depth_noise_at_centroid(cluster)
            centroid = cluster.mean(axis=0)
            dist = float(np.linalg.norm(centroid))

            depth_ok, depth_reason = self._record_depth_quality(
                distance_m=dist,
                noise_mm=noise_mm,
            )
            if not depth_ok:
                _LOGGER.info(
                    f"Cluster {i} rejected: {depth_reason} (measured noise: {noise_mm:.2f} mm, distance: {dist:.2f} m)"
                )
                continue

            pose, dims, shape_class = _pca_bbox(cluster, contour_props)

            # Color classification: exact per-cluster RGB → HSV pixel voting.
            color_name, color_conf = _dominant_color_voting(cluster_rgb)

            # Semantic class_id: "apple", "yellow_ball", "red_box", etc.
            class_id = _semantic_class_id(color_name, shape_class, dims)

            # Multi-factor confidence: colour vote + contour solidity + depth
            # quality. Temporal term is added in Wave 2.
            geometry_conf = (
                float(contour_props["solidity"]) if contour_props else 0.5
            )
            depth_conf = float(np.clip(1.0 - noise_mm / 15.0, 0.0, 1.0))
            score = _confidence_score(color_conf, geometry_conf, depth_conf)
            _LOGGER.debug(
                "Cluster %d: raw=%s_%s → semantic=%s score=%.2f "
                "(color=%.2f geom=%.2f depth=%.2f, dims=%.3f×%.3f×%.3f m)",
                i, color_name, shape_class, class_id, score,
                color_conf, geometry_conf, depth_conf,
                dims.x, dims.y, dims.z,
            )

            if score < self._min_publish_confidence:
                _LOGGER.info(
                    f"Cluster {i} rejected: confidence {score:.2f} < "
                    f"{self._min_publish_confidence:.2f} (color={color_name})"
                )
                continue

            hyp = ObjectHypothesis()
            hyp.class_id = class_id
            hyp.score = float(score)
            pose_with_covariance = PoseWithCovariance()
            pose_with_covariance.pose = pose
            det = Detection3D()
            det.header = Header(stamp=cloud.header.stamp, frame_id="base_link")
            det.results.append(
                ObjectHypothesisWithPose(
                    hypothesis=hyp,
                    pose=pose_with_covariance,
                )
            )
            det.bbox.size = dims
            detections.append(det)
            # Stash factors to recompute score with the temporal term once the
            # tracker confirms stability.
            det_factors[id(det)] = (color_conf, geometry_conf, depth_conf)

        # 5. Temporal voting — keep only detections stable across frames and
        #    fold the temporal score into the published confidence.
        stable = self._tracker.update(detections)
        published: list[Detection3D] = []
        for obj_i, (det, temporal_score) in enumerate(stable):
            color_conf, geometry_conf, depth_conf = det_factors[id(det)]
            det.results[0].hypothesis.score = _confidence_score(
                color_conf, geometry_conf, depth_conf, temporal_score
            )
            published.append(det)

            # Publish collision object only for confirmed detections.
            co = CollisionObject()
            co.header = det.header
            co.id = f"perception_obj_{obj_i}"
            co.operation = CollisionObject.ADD
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [
                det.bbox.size.x,
                det.bbox.size.y,
                det.bbox.size.z,
            ]
            co.primitives.append(box)
            co.primitive_poses.append(det.results[0].pose.pose)
            self._collision_pub.publish(co)
            self._published_collision_ids.add(co.id)

        self._last_detections = [(now, d) for d in published]
        self._last_stamp = cloud.header

    def _points_in_base_link(
        self,
        pts: np.ndarray,
        cloud: PointCloud2,
    ) -> np.ndarray | None:
        source_frame = str(getattr(cloud.header, "frame_id", "") or "").strip()
        if not source_frame or source_frame == "base_link":
            return pts

        # Non-blocking TF lookup (timeout=0.0) with fallback to last successful transform
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            self._last_transform = transform
        except Exception as exc:
            last_tf = getattr(self, "_last_transform", None)
            if last_tf is not None:
                transform = last_tf
            else:
                _LOGGER.warning(
                    "Rejecting perception cloud in frame '%s': missing transform to base_link (%s)",
                    source_frame,
                    exc,
                )
                return None

        return _transform_points(pts, transform)

    def _publish_detections(self) -> None:
        if not self._calibration_allows_scene_output():
            return

        now = time.time()
        # TTL eviction
        self._last_detections = [
            (t, d) for t, d in self._last_detections if (now - t) < self._ttl
        ]
        if not self._last_detections:
            return
        arr = Detection3DArray()
        arr.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        arr.detections = [d for _, d in self._last_detections]
        self._det_pub.publish(arr)
        # Remove stale collision objects
        for i in range(20):
            if not any(
                d for _, d in self._last_detections if f"cluster_{i}" in str(d.results)
            ):
                co = CollisionObject()
                co.header = arr.header
                co.id = f"perception_obj_{i}"
                co.operation = CollisionObject.REMOVE
                self._collision_pub.publish(co)
                self._published_collision_ids.discard(co.id)

    def _record_depth_noise(self, noise_mm: float) -> None:
        self._depth_noise_samples_mm.append(float(noise_mm))
        if len(self._depth_noise_samples_mm) > MAX_DEPTH_NOISE_SAMPLES:
            self._depth_noise_samples_mm = self._depth_noise_samples_mm[
                -MAX_DEPTH_NOISE_SAMPLES:
            ]

    def _record_depth_quality(
        self, *, distance_m: float, noise_mm: float
    ) -> tuple[bool, str]:
        self._record_depth_noise(noise_mm)
        ok, reason = check_depth_noise(distance_m, noise_mm, self._breakpoints, self._extrapolation)
        self._depth_in_range_samples.append(bool(ok))
        if len(self._depth_in_range_samples) > MAX_DEPTH_NOISE_SAMPLES:
            self._depth_in_range_samples = self._depth_in_range_samples[
                -MAX_DEPTH_NOISE_SAMPLES:
            ]
        return ok, reason

    def _calibration_status(self) -> tuple[bool, str, str, float]:
        cache_mtime_ns = getattr(self, "_calibration_cache_mtime_ns", None)
        cache_status = getattr(self, "_calibration_cache_status", None)
        try:
            current_mtime_ns = self._extrinsics_path.stat().st_mtime_ns
        except OSError:
            current_mtime_ns = None
        if (
            current_mtime_ns is not None
            and cache_status is not None
            and cache_mtime_ns == current_mtime_ns
        ):
            return cache_status

        try:
            extrinsics = load_extrinsics(self._extrinsics_path)
        except Exception as exc:
            return self._cache_calibration_status(
                current_mtime_ns,
                (False, f"calibration_invalid: {exc}", "", 0.0),
            )

        # Verify calibration_date exists (no age expiry enforced).
        data = extrinsics.get("hand_eye_extrinsics", {})
        calibration_date = str(data.get("calibration_date", ""))
        if not calibration_date or calibration_date == "<NOT_CALIBRATED>":
            return self._cache_calibration_status(
                current_mtime_ns,
                (False, "calibration_invalid: calibration_date missing", "", 0.0),
            )

        ok, reason = check_reprojection_error(
            extrinsics, max_mm=REPROJECTION_ERROR_MAX_MM
        )
        if not ok:
            return self._cache_calibration_status(
                current_mtime_ns,
                (False, f"calibration_invalid: {reason}", "", 0.0),
            )

        return self._cache_calibration_status(
            current_mtime_ns,
            (True, "", calibration_date, self._calibration_age_days(calibration_date)),
        )

    def _cache_calibration_status(
        self,
        mtime_ns: int | None,
        status: tuple[bool, str, str, float],
    ) -> tuple[bool, str, str, float]:
        if mtime_ns is not None:
            self._calibration_cache_mtime_ns = mtime_ns
            self._calibration_cache_status = status
        return status

    @staticmethod
    def _calibration_age_days(calibration_date: str) -> float:
        from datetime import datetime, timezone

        parsed = datetime.fromisoformat(calibration_date.replace("Z", "+00:00"))
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return (datetime.now(timezone.utc) - parsed).total_seconds() / 86400.0

    def _depth_noise_p95(self) -> float:
        if not self._depth_noise_samples_mm:
            return 0.0
        return float(np.percentile(np.asarray(self._depth_noise_samples_mm), 95))

    def _depth_in_range(self) -> bool:
        return bool(self._depth_in_range_samples) and all(self._depth_in_range_samples)

    def _calibration_allows_scene_output(self) -> bool:
        calibration_valid, failure_reason, _, _ = self._calibration_status()
        if calibration_valid:
            return True

        self._remove_published_collision_objects()
        if self._last_detections:
            self._last_detections = []
        _LOGGER.debug(
            "Perception scene output disabled until calibration is valid: %s",
            failure_reason,
        )
        return False

    def _remove_published_collision_objects(self) -> None:
        collision_ids = sorted(getattr(self, "_published_collision_ids", set()))
        for collision_id in collision_ids:
            co = CollisionObject()
            co.header = Header(frame_id="base_link")
            co.id = collision_id
            co.operation = CollisionObject.REMOVE
            self._collision_pub.publish(co)
        self._published_collision_ids = set()

    def _handle_get_object_positions(
        self,
        request: GetObjectPositions.Request,
        response: GetObjectPositions.Response,
    ) -> GetObjectPositions.Response:
        calibration_valid, failure_reason, calibration_date, age_days = (
            self._calibration_status()
        )
        response.calibration_valid = calibration_valid
        response.calibration_date_iso = calibration_date
        response.calibration_age_days = float(age_days)
        response.stamp = self._last_stamp.stamp
        response.depth_noise_mm_p95 = self._depth_noise_p95()
        response.depth_in_range = self._depth_in_range()

        if not calibration_valid:
            response.ok = False
            response.failure_reason = failure_reason
            response.detections = []
            return response
        if not response.depth_in_range:
            response.ok = False
            response.failure_reason = "depth_quality_invalid"
            response.detections = []
            return response

        now = time.time()
        self._last_detections = [
            (t, d) for t, d in self._last_detections if (now - t) < self._ttl
        ]
        response.detections = _filter_detections(
            [d for _, d in self._last_detections],
            request.class_filter,
        )
        response.ok = True
        response.failure_reason = ""
        return response

    def _publish_status(self) -> None:
        status = PerceptionStatus()
        calibration_valid, failure_reason, calibration_date, age_days = (
            self._calibration_status()
        )
        status.calibration_valid = calibration_valid
        status.calibration_date_iso = calibration_date
        status.calibration_age_days = float(age_days)
        status.depth_noise_mm_p95 = self._depth_noise_p95()
        status.depth_in_range = self._depth_in_range()
        if not calibration_valid:
            status.capability = "DISABLED"
            status.detail = failure_reason
        elif not status.depth_in_range:
            status.capability = "DEGRADED"
            status.detail = "depth quality unavailable or outside calibrated range"
        else:
            status.capability = "READY"
            status.detail = ""
        self._status_pub.publish(status)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = SceneProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
