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
from rclpy.qos import qos_profile_sensor_data
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
    check_calibration_freshness,
    check_depth_noise,
    check_reprojection_error,
)
from .scene_geometry import (
    _depth_noise_at_centroid,
    _detection_class_id,
    _dominant_color_name,
    _euclidean_clusters,
    _filter_detections,
    _pca_bbox,
    _ransac_plane,
    _read_xyz_rgb,
    _roi_crop,
    _transform_points,
    _voxel_downsample,
)

_LOGGER = logging.getLogger(__name__)
MAX_DEPTH_NOISE_SAMPLES = 50
CALIBRATION_MAX_AGE_DAYS = 30
REPROJECTION_ERROR_MAX_MM = 3.0

__all__ = [
    "SceneProcessor",
    "_detection_class_id",
    "_euclidean_clusters",
    "_filter_detections",
    "_pca_bbox",
    "_ransac_plane",
    "_roi_crop",
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
        self._ttl = float(self._cfg.get("detection_ttl_s", 2.0))
        self._breakpoints = self._cfg.get("depth_noise", {}).get("breakpoints", [])

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cloud_sub = Subscriber(
            self,
            PointCloud2,
            "/camera/depth/color/points",
            qos_profile=qos_profile_sensor_data,
        )
        self._info_sub = Subscriber(
            self,
            CameraInfo,
            "/camera/color/camera_info",
            qos_profile=qos_profile_sensor_data,
        )
        self._sync = ApproximateTimeSynchronizer(
            [self._cloud_sub, self._info_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._on_synced)

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
            return

        # Build index map for RGB lookup after transform/crop/downsample.
        # We track original indices through the pipeline so we can map
        # cluster points back to their RGB values.
        n_original = len(xyz_all)
        indices = np.arange(n_original, dtype=np.int32)

        pts = self._points_in_base_link(xyz_all, cloud)
        if pts is None or len(pts) == 0:
            self._remove_published_collision_objects()
            self._last_detections = []
            return

        # 1. ROI crop
        if self._bbox:
            mask = (
                (pts[:, 0] >= self._bbox["x"][0])
                & (pts[:, 0] <= self._bbox["x"][1])
                & (pts[:, 1] >= self._bbox["y"][0])
                & (pts[:, 1] <= self._bbox["y"][1])
                & (pts[:, 2] >= self._bbox["z"][0])
                & (pts[:, 2] <= self._bbox["z"][1])
            )
            pts = pts[mask]
            indices = indices[mask] if len(indices) == len(mask) else indices
        if len(pts) == 0:
            return

        # 2. Voxel downsample
        pts = _voxel_downsample(pts, self._voxel)

        # 3. RANSAC plane removal
        _, pts = _ransac_plane(pts, self._ransac_thresh)
        if len(pts) == 0:
            return

        # 4. Euclidean clustering
        clusters = _euclidean_clusters(
            pts, self._cluster_tol, self._cluster_min, self._cluster_max
        )
        if not clusters:
            return

        now = time.time()
        detections: list[Detection3D] = []
        for i, cluster in enumerate(clusters):
            noise_mm = _depth_noise_at_centroid(cluster)
            centroid = cluster.mean(axis=0)
            dist = float(np.linalg.norm(centroid))

            depth_ok, depth_reason = self._record_depth_quality(
                distance_m=dist,
                noise_mm=noise_mm,
            )
            if not depth_ok:
                _LOGGER.debug(
                    "Cluster %d rejected: %s",
                    i,
                    depth_reason,
                )
                continue

            pose, dims, shape_class = _pca_bbox(cluster)

            # Color classification: find nearest original indices for cluster
            # points and extract their RGB values.
            color_name = "unknown"
            if rgb_all is not None and n_original > 0:
                try:
                    # Map cluster points back to original cloud via nearest
                    # neighbor in the original XYZ (approximate after transform).
                    cluster_indices_approx = None
                    if len(indices) > 0:
                        from scipy.spatial import cKDTree as _cKDTree

                        tree = _cKDTree(xyz_all)
                        _, cluster_indices_approx = tree.query(
                            cluster[: min(200, len(cluster))]
                        )
                    color_name = _dominant_color_name(rgb_all, cluster_indices_approx)
                except Exception:
                    color_name = "unknown"

            # Build descriptive class_id: "red_sphere", "blue_box", etc.
            class_id = (
                f"{color_name}_{shape_class}"
                if color_name != "unknown"
                else shape_class
            )

            hyp = ObjectHypothesis()
            hyp.class_id = class_id
            hyp.score = 1.0
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

            # Publish collision object
            co = CollisionObject()
            co.header = det.header
            co.id = f"perception_obj_{i}"
            co.operation = CollisionObject.ADD
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [dims.x, dims.y, dims.z]
            co.primitives.append(box)
            co.primitive_poses.append(pose)
            self._collision_pub.publish(co)
            self._published_collision_ids.add(co.id)

        self._last_detections = [(now, d) for d in detections]
        self._last_stamp = cloud.header

    def _points_in_base_link(
        self,
        pts: np.ndarray,
        cloud: PointCloud2,
    ) -> np.ndarray | None:
        source_frame = str(getattr(cloud.header, "frame_id", "") or "").strip()
        if not source_frame or source_frame == "base_link":
            return pts

        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                source_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
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
        ok, reason = check_depth_noise(distance_m, noise_mm, self._breakpoints)
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

        ok, reason = check_calibration_freshness(
            extrinsics, max_age_days=CALIBRATION_MAX_AGE_DAYS
        )
        if not ok:
            return self._cache_calibration_status(
                current_mtime_ns,
                (False, f"calibration_invalid: {reason}", "", 0.0),
            )

        ok, reason = check_reprojection_error(
            extrinsics, max_mm=REPROJECTION_ERROR_MAX_MM
        )
        if not ok:
            return self._cache_calibration_status(
                current_mtime_ns,
                (False, f"calibration_invalid: {reason}", "", 0.0),
            )

        data = extrinsics.get("hand_eye_extrinsics", {})
        calibration_date = str(data.get("calibration_date", ""))
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
