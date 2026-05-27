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

def _classify_shape(eigvals: np.ndarray, contour_props: dict | None = None) -> str:
    """Classify object shape from PCA eigenvalues sorted descending.

    When *contour_props* is provided (from ``_contour_properties``), the
    contour circularity and solidity refine the initial PCA-only guess.
    """
    if eigvals[0] < 1e-10:
        return "unknown"
    r01 = eigvals[1] / eigvals[0]
    r02 = eigvals[2] / eigvals[0]

    # PCA first-pass guess.
    if r01 >= _SHAPE_SPHERE_R01_MIN and r02 >= _SHAPE_SPHERE_R02_MIN:
        label = "sphere"
    elif r01 >= _SHAPE_CYLINDER_R01_MIN and r02 < _SHAPE_CYLINDER_R02_MAX:
        label = "cylinder"
    elif r01 < _SHAPE_FLAT_R01_MAX:
        label = "flat"
    else:
        label = "box"

    # Refine with 2D contour metrics when available.
    if contour_props is not None:
        circularity = contour_props.get("circularity", 0.0)
        solidity = contour_props.get("solidity", 0.0)
        aspect_ratio = contour_props.get("aspect_ratio", 1.0)

        # High circularity + high solidity → sphere (overrides PCA "box").
        if circularity > 0.75 and solidity > 0.85:
            label = "sphere"
        # Very low circularity + elongated → cylinder.
        elif circularity < 0.4 and aspect_ratio > 2.0 and solidity > 0.7:
            label = "cylinder"
        # Rectangular contour with high solidity → box.
        elif 0.4 <= circularity <= 0.85 and solidity > 0.85 and 0.5 < aspect_ratio < 2.0:
            label = "box"

    return label


# ---------------------------------------------------------------------------
#  Contour-based filtering
# ---------------------------------------------------------------------------
# Minimum 2D solidity: rejects fragmented/noisy clusters that don't form a
# coherent contour when projected onto their PCA principal plane.
_CONTOUR_SOLIDITY_MIN = 0.30
# Minimum contour area in pixels (projected at 1000 px/m scale).
_CONTOUR_AREA_MIN_PX = 25.0


def _contour_properties(cluster: np.ndarray) -> dict | None:
    """Project a 3D cluster onto its PCA principal plane and compute 2D
    contour metrics: *solidity*, *circularity*, *aspect_ratio*.

    Returns ``None`` when contour extraction fails (too few points, etc.).
    """
    if len(cluster) < 5:
        return None

    try:
        import cv2
    except ImportError:
        return None

    # PCA: project onto plane spanned by the two largest eigenvectors.
    centroid = cluster.mean(axis=0)
    centered = cluster - centroid
    cov = np.cov(centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvecs = eigvecs[:, order]

    # Project onto first two principal axes.
    pts_2d = centered @ eigvecs[:, :2]

    # Scale to pixel space (1000 px/m) for findContours.
    scale = 1000.0
    pts_px = (pts_2d * scale).astype(np.float32)
    # Shift to positive coordinates.
    min_xy = pts_px.min(axis=0)
    pts_px -= min_xy
    pts_px += 10.0  # margin

    img_w = int(pts_px[:, 0].max()) + 20
    img_h = int(pts_px[:, 1].max()) + 20
    if img_w < 3 or img_h < 3 or img_w > 5000 or img_h > 5000:
        return None

    # Rasterise points into a binary image.
    canvas = np.zeros((img_h, img_w), dtype=np.uint8)
    for x, y in pts_px.astype(int):
        if 0 <= x < img_w and 0 <= y < img_h:
            canvas[y, x] = 255

    # Dilate slightly so sparse points form a connected region.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    canvas = cv2.dilate(canvas, kernel, iterations=1)

    contours, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Use the largest contour.
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < _CONTOUR_AREA_MIN_PX:
        return None

    perimeter = cv2.arcLength(contour, closed=True)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)

    solidity = area / hull_area if hull_area > 0 else 0.0
    circularity = (4.0 * np.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0.0

    # Aspect ratio from bounding rect.
    _, _, w, h = cv2.boundingRect(contour)
    aspect_ratio = max(w, h) / max(min(w, h), 1)

    return {
        "solidity": float(solidity),
        "circularity": float(circularity),
        "aspect_ratio": float(aspect_ratio),
        "area_px": float(area),
    }


def _contour_filter(cluster: np.ndarray) -> tuple[bool, dict | None]:
    """Return ``(keep, props)`` — rejects clusters with low solidity.

    Clusters that fail the solidity threshold are considered noise or
    fragmented point-cloud artefacts, not real objects.
    """
    props = _contour_properties(cluster)
    if props is None:
        # Cannot compute contour → keep cluster (fail-open for small objects).
        return True, None
    if props["solidity"] < _CONTOUR_SOLIDITY_MIN:
        return False, props
    return True, props



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


# ---------------------------------------------------------------------------
#  Semantic class mapping — human/LLM-friendly object names
# ---------------------------------------------------------------------------
# Retained for debug metadata / future size-based refinement.
_BALL_MAX_DIM_M = 0.055  # ping pong ball ≈ 40 mm; apple ≈ 70–90 mm


def _max_dim_m(dims) -> float:
    """Return the largest PCA bounding-box extent in metres."""
    return max(float(dims.x), float(dims.y), float(dims.z))


def _semantic_class_id(color: str, shape: str, dims) -> str:
    """Map raw perception attributes to human/LLM-friendly semantic class_id.

    Deadline policy:
    - Use *color* as the primary discriminator for the known fixed object set.
    - Use *shape* to avoid mapping red boxes as apples (sphere vs non-sphere).
    - Keep *dims* available for later refinement but do not depend on size yet.
    """
    color = (color or "unknown").lower()
    shape = (shape or "unknown").lower()

    # -- Spheres: ball vs apple -----------------------------------------
    if shape == "sphere":
        if color in {"red", "green"}:
            return "apple"
        if color == "yellow":
            return "yellow_ball"
        if color == "white":
            return "white_ball"

    # -- Non-sphere red objects → red_box (defensive: flat/unknown too) --
    if color == "red" and shape in {"box", "flat", "unknown"}:
        return "red_box"

    # -- Non-sphere blue objects → blue_rectangle -----------------------
    # TODO: if dominant color is *white* but cluster contains blue border
    #       pixels, map to blue_rectangle via dual-color analysis.
    if color == "blue" and shape in {"box", "flat", "unknown"}:
        return "blue_rectangle"

    # -- Fallback: colour_shape -----------------------------------------
    if color != "unknown":
        return f"{color}_{shape}"
    return shape


# Human-readable display names for known semantic class_ids.
_CLASS_DISPLAY_NAMES: dict[str, str] = {
    "apple": "apple",
    "yellow_ball": "yellow ball",
    "white_ball": "white ball",
    "red_box": "red box",
    "blue_rectangle": "blue rectangle",
}


def _display_name(class_id: str) -> str:
    """Return a human-readable display name for a semantic class_id."""
    if class_id in _CLASS_DISPLAY_NAMES:
        return _CLASS_DISPLAY_NAMES[class_id]
    return class_id.replace("_", " ")


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


def _pca_bbox(
    cluster: np.ndarray, contour_props: dict | None = None
) -> tuple[Pose, Vector3, str]:
    """Return (centroid_pose, dimensions, shape_class) from PCA of cluster.

    When *contour_props* is supplied, shape classification is refined using
    2D contour circularity and solidity metrics.
    """
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
    shape_class = _classify_shape(eigvals, contour_props)
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
