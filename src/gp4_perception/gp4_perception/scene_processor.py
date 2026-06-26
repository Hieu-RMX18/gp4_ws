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
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from .latest_frame import LatestValueSlot
from .query_perception_tool import load_extrinsics
from .safety_guards import (
    check_calibration_freshness,
    check_reprojection_error,
    classify_depth_quality,
)
from .temporal_tracker import TemporalTracker
from .scene_geometry import (
    _color_diagnostics,
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

# Debug cluster overlay colours (RGBA) by outcome.
_DEBUG_MARKER_COLORS = {
    "accepted": (0.0, 1.0, 0.0, 0.8),
    "rejected_color_unknown": (1.0, 0.0, 1.0, 0.6),
    "rejected_confidence": (1.0, 0.5, 0.0, 0.6),
    "rejected_depth": (1.0, 0.0, 0.0, 0.6),
    "rejected_contour": (0.5, 0.5, 0.5, 0.5),
}

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
            miss_tolerance_frames=int(self._cfg.get("temporal_miss_tolerance_frames", 2)),
        )
        self._ttl = float(self._cfg.get("detection_ttl_s", 2.0))
        self._breakpoints = self._cfg.get("depth_noise", {}).get("breakpoints", [])
        self._extrapolation = self._cfg.get("depth_noise", {}).get("extrapolation", "reject")
        # Two-tier depth: noise above the calibrated (safe) threshold but within
        # this ceiling is published for visualization only (DEGRADED_DEPTH),
        # never executable. Above the ceiling → rejected.
        self._depth_degraded_max_mm = float(
            self._cfg.get("depth_noise_degraded_mm_max", 0.0)
        )
        self._allow_degraded_depth = bool(
            self._cfg.get("allow_degraded_depth_detections", False)
        )
        self._debug_color = bool(self._cfg.get("debug_color_diagnostics", True))

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

        self._cloud_sub = self.create_subscription(
            PointCloud2,
            "/camera/depth/color/points",
            self._on_cloud,
            qos_profile,
        )

        pc_proc_cfg = self._cfg.get("pointcloud_processor", {})
        self._cloud_processing_rate_hz = float(pc_proc_cfg.get("processing_rate_hz", 8.0))
        self._latest_cloud: LatestValueSlot = LatestValueSlot()
        self._last_cloud_callback_time: float = time.monotonic()
        self._last_cloud_processed_time: float = time.monotonic()
        self._cloud_worker_timer = self.create_timer(
            1.0 / max(self._cloud_processing_rate_hz, 0.1),
            self._process_latest_cloud,
        )

        self._last_transform = None

        # Detection3DArray is now published by unified_visualizer node.
        self._collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self._debug_marker_pub = self.create_publisher(
            MarkerArray, "/perception/debug_clusters", 10
        )
        self._timer = self.create_timer(0.2, self._publish_detections)  # 5 Hz
        self._last_detections: list[
            tuple[float, Detection3D]
        ] = []  # (timestamp, detection) — cloud clusters; feed collision + viz
        # Grasp-grounding detections come from the RGB detector (unified_visualizer),
        # which classifies colour off the full-resolution colour image. The cloud's
        # per-point colour skews dark on depth-valid points and mis-votes (yellow→
        # gray), so it is no longer the grounding source; it still owns geometry,
        # depth-quality sampling, and collision objects.
        self._last_rgb_detections: list[tuple[float, Detection3D]] = []
        self._published_collision_ids: set[str] = set()
        self._depth_noise_samples_mm: list[float] = []
        self._depth_in_range_samples: list[bool] = []
        self._last_stamp = Header()
        self._last_cloud_time = time.time()
        self._logged_cloud_fields = False
        self._health_timer = self.create_timer(2.0, self._check_camera_health)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self._status_pub = self.create_publisher(
            PerceptionStatus, "/perception/status", 10
        )
        self._rgb_detections_sub = self.create_subscription(
            Detection3DArray,
            "/perception/detections",
            self._on_rgb_detections,
            10,
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
        now = time.monotonic()
        input_age = now - self._last_cloud_callback_time
        processed_age = now - self._last_cloud_processed_time
        if input_age > 5.0:
            _LOGGER.warning(
                "CLOUD_INPUT_STALE: no PointCloud2 callback for %.1f s", input_age
            )
        elif processed_age > 5.0:
            _LOGGER.warning(
                "CLOUD_PROCESSING_OVERRUN: latest cloud not processed for %.1f s",
                processed_age,
            )

    def _on_cloud(self, cloud: PointCloud2) -> None:
        self._last_cloud_callback_time = time.monotonic()
        self._last_cloud_time = time.time()  # keep existing health check working
        self._latest_cloud.put(cloud)

    def _process_latest_cloud(self) -> None:
        cloud = self._latest_cloud.take()
        if cloud is None:
            return
        self._last_cloud_processed_time = time.monotonic()
        self._process_cloud(cloud)

    def _process_cloud(self, cloud: PointCloud2) -> None:
        if not self._calibration_allows_scene_output():
            return

        # Read XYZ + optional RGB for color classification.
        xyz_all, rgb_all = _read_xyz_rgb(cloud)
        if not getattr(self, "_logged_cloud_fields", False):
            self._logged_cloud_fields = True
            field_desc = [
                f"{f.name}:off={f.offset}:type={f.datatype}:count={f.count}"
                for f in cloud.fields
            ]
            self.get_logger().info(
                f"[perception] cloud_fields={field_desc} point_step={cloud.point_step} "
                f"bigendian={cloud.is_bigendian} rgb_decoded={rgb_all is not None}"
            )
        if xyz_all is None or len(xyz_all) == 0:
            _LOGGER.info("No points read from PointCloud2")
            return

        pts_raw = len(xyz_all)

        pts = self._points_in_base_link(xyz_all, cloud)
        if pts is None or len(pts) == 0:
            self.get_logger().info(
                f"[perception] pts_raw={pts_raw} pts_base_link=0 "
                "(no transform to base_link)"
            )
            self._remove_published_collision_objects()
            self._last_detections = []
            return

        # RGB stays index-aligned with pts (transform preserves point order),
        # so we slice rgb with the same indices at every pipeline step.
        rgb = rgb_all
        # All read points have valid depth (read_points skips NaNs).
        pts_valid_depth = pts_raw

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
        pts_roi = len(pts)
        if pts_roi == 0:
            self.get_logger().info(
                f"[perception] pts_raw={pts_raw} pts_valid_depth={pts_valid_depth} "
                "pts_roi=0 (empty after ROI crop)"
            )
            return

        # 2. Voxel downsample (carry RGB by index)
        keep = _voxel_downsample_indices(pts, self._voxel)
        pts = pts[keep]
        if rgb is not None:
            rgb = rgb[keep]
        pts_after_voxel = len(pts)

        # 3. RANSAC plane removal — only remove near-vertical-normal planes
        #    (table/floor); keep vertical object faces.
        _, out_idx, _ = _ransac_plane_fit(
            pts, self._ransac_thresh, normal_z_min=self._ransac_normal_z_min
        )
        pts = pts[out_idx]
        if rgb is not None:
            rgb = rgb[out_idx]
        pts_after_plane = len(pts)
        if pts_after_plane == 0:
            self.get_logger().debug(
                f"[perception] pts_raw={pts_raw} pts_roi={pts_roi} "
                f"pts_after_voxel={pts_after_voxel} pts_after_plane=0"
            )
            return

        # 4. Euclidean clustering (index arrays so RGB stays aligned)
        cluster_indices = _euclidean_cluster_indices(
            pts, self._cluster_tol, self._cluster_min, self._cluster_max
        )
        self.get_logger().debug(
            f"[perception] pts_raw={pts_raw} pts_valid_depth={pts_valid_depth} "
            f"pts_roi={pts_roi} pts_after_voxel={pts_after_voxel} "
            f"pts_after_plane={pts_after_plane} clusters={len(cluster_indices)}"
        )
        if not cluster_indices:
            return

        now = time.time()
        tracker_items: list[tuple[Detection3D, tuple[float, float, float]]] = []
        debug_records: list[dict] = []
        for i, cluster_idx in enumerate(cluster_indices):
            cluster = pts[cluster_idx]
            cluster_rgb = rgb[cluster_idx] if rgb is not None else None
            aabb_min = cluster.min(axis=0)
            aabb_max = cluster.max(axis=0)
            aabb_center = (aabb_min + aabb_max) / 2.0
            aabb_size = np.maximum(aabb_max - aabb_min, 1e-3)
            # 4a. Contour filter — reject fragmented/noisy clusters.
            contour_ok, contour_props = _contour_filter(cluster)
            if not contour_ok:
                self.get_logger().debug(
                    f"[perception] cluster[{i}] rejected reject_reason=contour "
                    f"solidity={(contour_props['solidity'] if contour_props else 0.0):.2f}"
                )
                debug_records.append({
                    "id": i, "center": aabb_center, "size": aabb_size,
                    "status": "rejected_contour", "color": "-",
                    "color_score": 0.0, "total": 0.0, "reason": "contour",
                })
                continue

            noise_mm = _depth_noise_at_centroid(cluster)
            centroid = cluster.mean(axis=0)
            dist = float(np.linalg.norm(centroid))

            # Two-tier depth: OK (executable) / DEGRADED_DEPTH (viz only) / REJECT.
            depth_quality, threshold_used, depth_reason = classify_depth_quality(
                dist,
                noise_mm,
                self._breakpoints,
                degraded_max_mm=self._depth_degraded_max_mm,
                extrapolation=self._extrapolation,
            )
            if depth_quality == "DEGRADED_DEPTH" and not self._allow_degraded_depth:
                depth_quality = "REJECT"
            self._record_depth_tier(noise_mm, depth_quality)
            thr_str = f"{threshold_used:.2f}" if threshold_used is not None else "n/a"
            if depth_quality == "REJECT":
                self.get_logger().debug(
                    f"[perception] cluster[{i}] rejected reject_reason=depth "
                    f"depth_noise_mm={noise_mm:.2f} threshold_used={thr_str} "
                    f"dist={dist:.2f} ({depth_reason})"
                )
                debug_records.append({
                    "id": i, "center": aabb_center, "size": aabb_size,
                    "status": "rejected_depth", "color": "-",
                    "color_score": 0.0, "total": 0.0,
                    "reason": f"depth>{thr_str}",
                })
                continue

            pose, dims, shape_class = _pca_bbox(cluster, contour_props)

            # Color classification: exact per-cluster RGB → HSV pixel voting.
            color_name, color_conf = _dominant_color_voting(cluster_rgb)
            if self._debug_color:
                diag = _color_diagnostics(cluster_rgb)
                self.get_logger().debug(
                    f"[perception] cluster[{i}] diag n={diag.get('n')} "
                    f"rgb_min={diag.get('rgb_min')} rgb_max={diag.get('rgb_max')} "
                    f"rgb_mean={diag.get('rgb_mean')} "
                    f"vote_rgb={diag.get('vote_rgb')} vote_bgr={diag.get('vote_bgr')} "
                    f"sample={diag.get('sample')}"
                )

            # Semantic class_id: "apple", "yellow_ball", "red_box", etc.
            class_id = _semantic_class_id(color_name, shape_class, dims, rgb_pixels=cluster_rgb, cluster=cluster)

            # Multi-factor confidence: colour vote + contour solidity + depth
            # quality. Temporal term is added by the tracker below.
            geometry_conf = (
                float(contour_props["solidity"]) if contour_props else 0.5
            )
            depth_conf = float(np.clip(1.0 - noise_mm / 15.0, 0.0, 1.0))
            score = _confidence_score(color_conf, geometry_conf, depth_conf)
            self.get_logger().debug(
                f"[perception] cluster[{i}] points={len(cluster)} "
                f"color={color_name} color_score={color_conf:.2f} "
                f"geometry_score={geometry_conf:.2f} depth_score={depth_conf:.2f} "
                f"depth_noise_mm={noise_mm:.2f} threshold_used={thr_str} "
                f"quality={depth_quality} total={score:.2f} class_id={class_id}"
            )

            if score < self._min_publish_confidence:
                reason = "color_unknown" if color_name == "unknown" else "confidence"
                status = (
                    "rejected_color_unknown"
                    if color_name == "unknown"
                    else "rejected_confidence"
                )
                self.get_logger().debug(
                    f"[perception] cluster[{i}] rejected reject_reason={reason} "
                    f"total={score:.2f} < min={self._min_publish_confidence:.2f} "
                    f"color={color_name}"
                )
                debug_records.append({
                    "id": i, "center": aabb_center, "size": aabb_size,
                    "status": status, "color": color_name,
                    "color_score": color_conf, "total": score, "reason": reason,
                })
                continue

            debug_records.append({
                "id": i, "center": aabb_center, "size": aabb_size,
                "status": "accepted", "color": color_name,
                "color_score": color_conf, "total": score,
                "reason": depth_quality,
            })

            hyp = ObjectHypothesis()
            hyp.class_id = class_id
            hyp.score = float(score)
            pose_with_covariance = PoseWithCovariance()
            pose_with_covariance.pose = pose
            det = Detection3D()
            det.header = Header(stamp=cloud.header.stamp, frame_id="base_link")
            # Tag degraded-depth detections so the execution path (service) can
            # exclude them; visualization still shows them on the topic.
            det.id = "" if depth_quality == "OK" else depth_quality
            det.results.append(
                ObjectHypothesisWithPose(
                    hypothesis=hyp,
                    pose=pose_with_covariance,
                )
            )
            det.bbox.size = dims
            tracker_items.append((det, (color_conf, geometry_conf, depth_conf)))

        # 5. Temporal voting — keep only detections stable across frames and
        #    fold the temporal score into the published confidence.
        stable = self._tracker.update(tracker_items)
        published: list[Detection3D] = []
        for obj_i, (det, temporal_score, factors) in enumerate(stable):
            color_conf, geometry_conf, depth_conf = factors
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

        # Debug overlay: publish every cluster (accepted + rejected) colour-coded.
        self._publish_debug_markers(debug_records, cloud.header.stamp)

    def _publish_debug_markers(self, records: list[dict], stamp) -> None:
        """Publish a MarkerArray on /perception/debug_clusters: a translucent
        CUBE plus a TEXT label per cluster, coloured by accept/reject outcome."""
        arr = MarkerArray()
        clear = Marker()
        clear.header = Header(stamp=stamp, frame_id="base_link")
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for rec in records:
            r, g, b, a = _DEBUG_MARKER_COLORS.get(
                rec["status"], (1.0, 1.0, 1.0, 0.5)
            )
            center = rec["center"]
            size = rec["size"]
            cube = Marker()
            cube.header = Header(stamp=stamp, frame_id="base_link")
            cube.ns = "debug_clusters"
            cube.id = rec["id"] * 2
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(center[0])
            cube.pose.position.y = float(center[1])
            cube.pose.position.z = float(center[2])
            cube.pose.orientation.w = 1.0
            cube.scale.x = float(size[0])
            cube.scale.y = float(size[1])
            cube.scale.z = float(size[2])
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = r, g, b, a
            arr.markers.append(cube)

            text = Marker()
            text.header = Header(stamp=stamp, frame_id="base_link")
            text.ns = "debug_clusters_text"
            text.id = rec["id"] * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(center[0])
            text.pose.position.y = float(center[1])
            text.pose.position.z = float(center[2] + size[2] / 2 + 0.03)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.03
            text.color.r, text.color.g, text.color.b, text.color.a = r, g, b, 1.0
            text.text = (
                f"c{rec['id']} {rec['status']} {rec['color']} "
                f"cs={rec['color_score']:.2f} t={rec['total']:.2f} {rec['reason']}"
            )
            arr.markers.append(text)
        self._debug_marker_pub.publish(arr)

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
        arr_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        # Remove stale collision objects
        for i in range(20):
            if not any(
                d for _, d in self._last_detections if f"cluster_{i}" in str(d.results)
            ):
                co = CollisionObject()
                co.header = arr_header
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

    def _record_depth_tier(self, noise_mm: float, quality: str) -> None:
        """Record a sample for stats: p95 over all noise, but only OK-tier
        counts toward depth_in_range (degraded must not enable execution)."""
        self._record_depth_noise(noise_mm)
        self._depth_in_range_samples.append(quality == "OK")
        if len(self._depth_in_range_samples) > MAX_DEPTH_NOISE_SAMPLES:
            self._depth_in_range_samples = self._depth_in_range_samples[
                -MAX_DEPTH_NOISE_SAMPLES:
            ]

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

        # Verify calibration_date exists and is valid ISO 8601 (no age expiry enforced).
        data = extrinsics.get("hand_eye_extrinsics", {})
        ok, reason = check_calibration_freshness(extrinsics)
        if not ok:
            return self._cache_calibration_status(
                current_mtime_ns,
                (False, f"calibration_invalid: {reason}", "", 0.0),
            )
        calibration_date = str(data.get("calibration_date", ""))

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

    def _on_rgb_detections(self, msg: Detection3DArray) -> None:
        """Latest-snapshot store of RGB-detector detections for grasp grounding.

        Each Detection3DArray is a complete frame, so we replace rather than
        append; the service applies TTL so detections expire if the detector
        stops publishing.
        """
        now = time.time()
        # Only accept base_link-framed detections; the detector falls back to the
        # camera frame when TF is unavailable, which must not reach grasp grounding.
        self._last_rgb_detections = [
            (now, d) for d in msg.detections if d.header.frame_id == "base_link"
        ]

    def _in_workspace(self, det: Detection3D) -> bool:
        if not (self._bbox and self._enable_bbox_filter):
            return True
        if not det.results:
            return False
        p = det.results[0].pose.pose.position
        return (
            self._bbox["x"][0] <= p.x <= self._bbox["x"][1]
            and self._bbox["y"][0] <= p.y <= self._bbox["y"][1]
            and self._bbox["z"][0] <= p.z <= self._bbox["z"][1]
        )

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
        self._last_rgb_detections = [
            (t, d) for t, d in self._last_rgb_detections if (now - t) < self._ttl
        ]
        # Execution path: keep in-workspace detections; exclude DEGRADED_DEPTH
        # (defense-in-depth — the RGB detector already drops invalid-depth ones).
        executable = [
            d
            for _, d in self._last_rgb_detections
            if str(getattr(d, "id", "")) != "DEGRADED_DEPTH" and self._in_workspace(d)
        ]
        response.detections = _filter_detections(executable, request.class_filter)
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
