"""Pure scene-processing helpers for point clouds and detections."""

from __future__ import annotations

import logging

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3D


_LOGGER = logging.getLogger(__name__)

_SHAPE_SPHERE_R01_MIN = 0.6
_SHAPE_SPHERE_R02_MIN = 0.4
_SHAPE_CYLINDER_R01_MIN = 0.6
_SHAPE_CYLINDER_R02_MAX = 0.4
_SHAPE_FLAT_R01_MAX = 0.3

_COLOR_RANGES = [
    ("red", 0, 10, 80, 60),
    ("red", 170, 180, 80, 60),
    ("orange", 10, 25, 80, 60),
    ("yellow", 25, 35, 80, 60),
    ("green", 35, 85, 50, 40),
    ("blue", 85, 130, 50, 40),
    ("purple", 130, 170, 50, 40),
]


def _read_xyz(cloud: PointCloud2) -> np.ndarray | None:
    """Convert PointCloud2 to Nx3 float32 numpy array."""
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
    """Read XYZ plus RGB from PointCloud2."""
    try:
        from sensor_msgs_py.point_cloud2 import read_points

        field_names_available = {f.name for f in cloud.fields}
        has_rgb = "rgb" in field_names_available or "r" in field_names_available

        if has_rgb and "r" in field_names_available:
            pts = list(
                read_points(
                    cloud, field_names=("x", "y", "z", "r", "g", "b"), skip_nans=True
                )
            )
            if not pts:
                return None, None
            xyz = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
            rgb = np.array([(p[3], p[4], p[5]) for p in pts], dtype=np.uint8)
            return xyz, rgb

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
    """Classify object shape from PCA eigenvalues sorted descending."""
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
    """Determine the dominant color name for a cluster from RGB values."""
    if rgb_all is None or cluster_indices is None or len(cluster_indices) == 0:
        return "unknown"

    cluster_rgb = rgb_all[cluster_indices]
    if cluster_rgb.size == 0:
        return "unknown"

    mean_rgb = cluster_rgb.mean(axis=0).astype(np.uint8).reshape(1, 1, 3)
    try:
        import cv2

        mean_hsv = cv2.cvtColor(mean_rgb, cv2.COLOR_RGB2HSV)[0, 0]
    except Exception:
        return "unknown"

    h, s, v = int(mean_hsv[0]), int(mean_hsv[1]), int(mean_hsv[2])
    if s < 40:
        return "white" if v > 180 else ("gray" if v > 60 else "black")

    for name, h_min, h_max, s_min, v_min in _COLOR_RANGES:
        if h_min <= h <= h_max and s >= s_min and v >= v_min:
            return name

    return "unknown"


def _roi_crop(pts: np.ndarray, bbox: dict) -> np.ndarray:
    """Keep points inside workspace bbox."""
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
    _, idx = np.unique(coords, axis=0, return_index=True)
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
    for cluster in clusters:
        if min_size <= len(cluster) <= max_size:
            result.append(pts[cluster])
    return result


def _pca_bbox(cluster: np.ndarray) -> tuple[Pose, Vector3, str]:
    """Return (centroid_pose, dimensions, shape_class) from PCA of cluster."""
    centroid = cluster.mean(axis=0)
    centered = cluster - centroid
    cov = np.cov(centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]
    rot = Rotation.from_matrix(eigvecs)
    quat = rot.as_quat()
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
    rot = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
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
    """Estimate depth noise as std-dev of z near centroid."""
    if len(cluster) < 5:
        return 0.0
    centroid = cluster.mean(axis=0)
    dists = np.linalg.norm(cluster - centroid, axis=1)
    idx = np.argsort(dists)[: min(25, len(cluster))]
    zs = cluster[idx, 2]
    return float(np.std(zs) * 1000.0)
