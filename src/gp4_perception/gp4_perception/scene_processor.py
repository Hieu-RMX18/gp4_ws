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
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Vector3
from interfaces.msg import PerceptionStatus
from interfaces.srv import GetObjectPositions
from moveit_msgs.msg import CollisionObject
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
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

_LOGGER = logging.getLogger(__name__)
MAX_DEPTH_NOISE_SAMPLES = 50
CALIBRATION_MAX_AGE_DAYS = 30

# ── Shape classification thresholds (PCA eigenvalue ratios) ─────────────────
# Eigenvalues sorted descending: λ0 >= λ1 >= λ2.
# Ratios: r01 = λ1/λ0, r02 = λ2/λ0.
# sphere:   all eigenvalues similar   → r01 > 0.6 AND r02 > 0.4
# cylinder: two similar, one small    → r01 > 0.6 AND r02 < 0.4
# flat:     one dominant, two small    → r01 < 0.3
# box:      fallback
_SHAPE_SPHERE_R01_MIN = 0.6
_SHAPE_SPHERE_R02_MIN = 0.4
_SHAPE_CYLINDER_R01_MIN = 0.6
_SHAPE_CYLINDER_R02_MAX = 0.4
_SHAPE_FLAT_R01_MAX = 0.3

# ── Color classification (HSV ranges) ──────────────────────────────────────
_COLOR_RANGES = [
    # (name, h_min, h_max, s_min, v_min)
    ("red",     0,   10, 80, 60),
    ("red",   170,  180, 80, 60),
    ("orange",  10,   25, 80, 60),
    ("yellow",  25,   35, 80, 60),
    ("green",   35,   85, 50, 40),
    ("blue",    85,  130, 50, 40),
    ("purple", 130,  170, 50, 40),
]
REPROJECTION_ERROR_MAX_MM = 3.0


def _read_xyz(cloud: PointCloud2) -> np.ndarray | None:
    """Convert PointCloud2 to Nx3 float32 numpy array (x, y, z)."""
    try:
        from sensor_msgs_py.point_cloud2 import read_points

        pts = list(read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            return None
        arr = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        return arr
    except Exception as exc:
        _LOGGER.warning("point_cloud2 read failed: %s", exc)
        return None


def _read_xyz_rgb(cloud: PointCloud2) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Read XYZ + RGB from PointCloud2. Returns (Nx3 xyz, Nx3 rgb_uint8) or (None, None)."""
    try:
        from sensor_msgs_py.point_cloud2 import read_points

        field_names_available = {f.name for f in cloud.fields}
        has_rgb = "rgb" in field_names_available or "r" in field_names_available

        if has_rgb and "r" in field_names_available:
            pts = list(read_points(
                cloud, field_names=("x", "y", "z", "r", "g", "b"), skip_nans=True
            ))
            if not pts:
                return None, None
            xyz = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
            rgb = np.array([(p[3], p[4], p[5]) for p in pts], dtype=np.uint8)
            return xyz, rgb

        # Fallback: no RGB channels available
        pts = list(read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            return None, None
        xyz = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        return xyz, None
    except Exception as exc:
        _LOGGER.debug("point_cloud2 rgb read failed (non-fatal): %s", exc)
        xyz = _read_xyz(cloud)
        return xyz, None


def _classify_shape(eigvals: np.ndarray) -> str:
    """Classify object shape from PCA eigenvalues (sorted descending).

    Uses eigenvalue ratio analysis:
      sphere:   all eigenvalues similar
      cylinder: two similar, one small
      flat:     one dominant, two small
      box:      fallback
    """
    if eigvals[0] < 1e-10:
        return "unknown"
    r01 = eigvals[1] / eigvals[0]
    r02 = eigvals[2] / eigvals[0]
    if r01 >= _SHAPE_SPHERE_R01_MIN and r02 >= _SHAPE_SPHERE_R02_MIN:
        return "sphere"
    if r01 >= _SHAPE_CYLINDER_R01_MIN and r02 < _SHAPE_CYLINDER_R02_MAX:
        return "cylinder"
    if r01 < _SHAPE_FLAT_R01_MAX:
        return "flat"
    return "box"


def _dominant_color_name(
    rgb_all: np.ndarray | None,
    cluster_indices: np.ndarray | None,
) -> str:
    """Determine the dominant color name for a cluster from RGB values.

    Returns a color name (red, green, blue, etc.) or 'unknown' when
    RGB data is unavailable or the color doesn't match any known range.
    """
    if rgb_all is None or cluster_indices is None or len(cluster_indices) == 0:
        return "unknown"

    cluster_rgb = rgb_all[cluster_indices]
    if cluster_rgb.size == 0:
        return "unknown"

    # Convert mean RGB to HSV for robust color classification.
    mean_rgb = cluster_rgb.mean(axis=0).astype(np.uint8).reshape(1, 1, 3)
    try:
        import cv2
        mean_hsv = cv2.cvtColor(mean_rgb, cv2.COLOR_RGB2HSV)[0, 0]
    except Exception:
        return "unknown"

    h, s, v = int(mean_hsv[0]), int(mean_hsv[1]), int(mean_hsv[2])

    # Low saturation → gray/white/black
    if s < 40:
        return "white" if v > 180 else ("gray" if v > 60 else "black")

    for name, h_min, h_max, s_min, v_min in _COLOR_RANGES:
        if h_min <= h <= h_max and s >= s_min and v >= v_min:
            return name

    return "unknown"


def _roi_crop(pts: np.ndarray, bbox: dict) -> np.ndarray:
    """Keep points inside workspace bbox. bbox: {x:[min,max], y:[min,max], z:[min,max]}."""
    mask = (
        (pts[:, 0] >= bbox["x"][0])
        & (pts[:, 0] <= bbox["x"][1])
        & (pts[:, 1] >= bbox["y"][0])
        & (pts[:, 1] <= bbox["y"][1])
        & (pts[:, 2] >= bbox["z"][0])
        & (pts[:, 2] <= bbox["z"][1])
    )
    return pts[mask]


def _voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Simple grid-bin downsample."""
    if pts.size == 0:
        return pts
    coords = np.floor(pts / voxel_size).astype(np.int32)
    unique, idx = np.unique(coords, axis=0, return_index=True)
    return pts[idx]


def _ransac_plane(
    pts: np.ndarray, threshold: float, max_iter: int = 100
) -> tuple[np.ndarray, np.ndarray]:
    """Return (inliers, outliers) for largest plane."""
    if len(pts) < 10:
        return pts, np.empty((0, 3), dtype=np.float32)
    best_inliers = np.array([], dtype=int)
    for _ in range(max_iter):
        sample = pts[np.random.choice(len(pts), 3, replace=False)]
        p1, p2, p3 = sample
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-8:
            continue
        normal = normal / norm_len
        d = -np.dot(normal, p1)
        dists = np.abs(pts @ normal + d)
        inliers = np.where(dists < threshold)[0]
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
    if len(best_inliers) == 0:
        return pts, np.empty((0, 3), dtype=np.float32)
    mask = np.ones(len(pts), dtype=bool)
    mask[best_inliers] = False
    return pts[best_inliers], pts[mask]


def _euclidean_clusters(
    pts: np.ndarray, tolerance: float, min_size: int, max_size: int
) -> list[np.ndarray]:
    """DBSCAN-like clustering with cKDTree."""
    if len(pts) == 0:
        return []
    tree = cKDTree(pts)
    visited = np.zeros(len(pts), dtype=bool)
    clusters: list[list[int]] = []
    for i in range(len(pts)):
        if visited[i]:
            continue
        neighbors = tree.query_ball_point(pts[i], tolerance)
        if len(neighbors) < min_size:
            visited[neighbors] = True
            continue
        cluster = set(neighbors)
        queue = list(neighbors)
        while queue:
            j = queue.pop()
            if visited[j]:
                continue
            visited[j] = True
            nn = tree.query_ball_point(pts[j], tolerance)
            for k in nn:
                if k not in cluster:
                    cluster.add(k)
                    queue.append(k)
        clusters.append(list(cluster))
    result = []
    for c in clusters:
        if min_size <= len(c) <= max_size:
            result.append(pts[c])
    return result


def _pca_bbox(cluster: np.ndarray) -> tuple[Pose, Vector3, str]:
    """Return (centroid_pose, dimensions, shape_class) from PCA of cluster."""
    centroid = cluster.mean(axis=0)
    centered = cluster - centroid
    cov = np.cov(centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    # Sort descending
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]
    # Rotation matrix from eigvecs to quaternion
    rot = Rotation.from_matrix(eigvecs)
    quat = rot.as_quat()  # [x, y, z, w]
    pose = Pose()
    pose.position = Point(
        x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2])
    )
    pose.orientation = Quaternion(
        x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])
    )
    dims = Vector3(
        x=float(2 * np.sqrt(eigvals[0])),
        y=float(2 * np.sqrt(eigvals[1])),
        z=float(2 * np.sqrt(eigvals[2])),
    )
    shape_class = _classify_shape(eigvals)
    return pose, dims, shape_class


def _transform_points(pts: np.ndarray, transform) -> np.ndarray:
    """Apply a geometry_msgs TransformStamped to Nx3 points."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    rot = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    )
    offset = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
    return (rot.apply(pts.astype(np.float64)) + offset).astype(np.float32)


def _detection_class_id(detection: Detection3D) -> str:
    if not detection.results:
        return ""
    return str(detection.results[0].hypothesis.class_id)


def _filter_detections(
    detections: list[Detection3D], class_filter: str
) -> list[Detection3D]:
    requested_class = class_filter.strip()
    if not requested_class:
        return detections
    return [
        detection
        for detection in detections
        if _detection_class_id(detection) == requested_class
    ]


def _depth_noise_at_centroid(cluster: np.ndarray) -> float:
    """Estimate depth noise as std-dev of z near centroid (5x5 nearest-neighbor patch)."""
    if len(cluster) < 5:
        return 0.0
    centroid = cluster.mean(axis=0)
    dists = np.linalg.norm(cluster - centroid, axis=1)
    idx = np.argsort(dists)[: min(25, len(cluster))]
    zs = cluster[idx, 2]
    return float(np.std(zs) * 1000.0)


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
                            cluster[:min(200, len(cluster))]
                        )
                    color_name = _dominant_color_name(rgb_all, cluster_indices_approx)
                except Exception:
                    color_name = "unknown"

            # Build descriptive class_id: "red_sphere", "blue_box", etc.
            class_id = f"{color_name}_{shape_class}" if color_name != "unknown" else shape_class

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
        return bool(self._depth_in_range_samples) and all(
            self._depth_in_range_samples
        )

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
