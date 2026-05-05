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
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesis, ObjectHypothesisWithPose

from message_filters import ApproximateTimeSynchronizer, Subscriber

_LOGGER = logging.getLogger(__name__)


def _read_xyz(cloud: PointCloud2) -> np.ndarray | None:
    """Convert PointCloud2 to Nx3 float32 numpy array (x, y, z)."""
    try:
        from sensor_msgs_py.point_cloud2 import read_points

        pts = list(read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            return None
        arr = np.asarray(pts, dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape(-1, 3)
        return arr[:, :3]
    except Exception as exc:
        _LOGGER.warning("point_cloud2 read failed: %s", exc)
        return None


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


def _ransac_plane(pts: np.ndarray, threshold: float, max_iter: int = 100) -> tuple[np.ndarray, np.ndarray]:
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


def _euclidean_clusters(pts: np.ndarray, tolerance: float, min_size: int, max_size: int) -> list[np.ndarray]:
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


def _pca_bbox(cluster: np.ndarray) -> tuple[Pose, Vector3]:
    """Return (centroid_pose, dimensions) from PCA of cluster."""
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
    pose.position = Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2]))
    pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
    dims = Vector3(
        x=float(2 * np.sqrt(eigvals[0])),
        y=float(2 * np.sqrt(eigvals[1])),
        z=float(2 * np.sqrt(eigvals[2])),
    )
    return pose, dims


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
            self, PointCloud2, "/camera/depth/color/points", qos_profile=qos_profile_sensor_data
        )
        self._info_sub = Subscriber(
            self, CameraInfo, "/camera/color/camera_info", qos_profile=qos_profile_sensor_data
        )
        self._sync = ApproximateTimeSynchronizer(
            [self._cloud_sub, self._info_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._on_synced)

        self._det_pub = self.create_publisher(Detection3DArray, "/perception/detections", 10)
        self._collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self._timer = self.create_timer(0.2, self._publish_detections)  # 5 Hz
        self._last_detections: list[tuple[float, Detection3D]] = []  # (timestamp, detection)
        self._last_stamp = Header()
        self._last_cloud_time = time.time()
        self._health_timer = self.create_timer(2.0, self._check_camera_health)

    def _check_camera_health(self) -> None:
        elapsed = time.time() - self._last_cloud_time
        if elapsed > 5.0:
            _LOGGER.warning(
                "No point cloud received for %.1f s — check camera connection or QoS match", elapsed
            )

    def _on_synced(self, cloud: PointCloud2, _: CameraInfo) -> None:
        self._last_cloud_time = time.time()
        pts = _read_xyz(cloud)
        if pts is None or len(pts) == 0:
            return

        # 1. ROI crop
        if self._bbox:
            pts = _roi_crop(pts, self._bbox)
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
            # Check range-aware depth noise
            centroid = cluster.mean(axis=0)
            dist = float(np.linalg.norm(centroid))
            from .safety_guards import _interpolate_threshold

            threshold = _interpolate_threshold(dist, self._breakpoints)
            if threshold is not None and noise_mm > threshold:
                _LOGGER.debug("Cluster %d rejected: noise %.2f mm > %.2f mm", i, noise_mm, threshold)
                continue

            pose, dims = _pca_bbox(cluster)
            hyp = ObjectHypothesis()
            hyp.class_id = f"cluster_{i}"
            hyp.score = 1.0
            det = Detection3D()
            det.header = Header(stamp=cloud.header.stamp, frame_id="base_link")
            det.results.append(ObjectHypothesisWithPose(hypothesis=hyp, pose=pose))
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

        self._last_detections = [(now, d) for d in detections]
        self._last_stamp = cloud.header

    def _publish_detections(self) -> None:
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
            if not any(d for _, d in self._last_detections if f"cluster_{i}" in str(d.results)):
                co = CollisionObject()
                co.header = arr.header
                co.id = f"perception_obj_{i}"
                co.operation = CollisionObject.REMOVE
                self._collision_pub.publish(co)


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
