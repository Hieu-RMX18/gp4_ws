"""Pure scene-processing helpers for point clouds and detections."""

from __future__ import annotations

import logging
import sys

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


def _pointcloud2_to_array(cloud: PointCloud2, fields: tuple[str, ...]) -> np.ndarray:
    """Read PointCloud2 into a structured array without filtering fields.

    This mirrors ``sensor_msgs_py.point_cloud2.read_points`` but keeps the
    packed RGB field untouched so its float bit-pattern is not interpreted as a
    numeric value for NaN filtering.
    """
    from sensor_msgs_py.point_cloud2 import dtype_from_fields

    try:
        dtype = dtype_from_fields(cloud.fields, point_step=cloud.point_step)
    except TypeError:
        # Compatibility with older sensor_msgs_py versions that did not expose
        # the point_step keyword. Preserve PointCloud2 padding explicitly so
        # each record advances by cloud.point_step bytes.
        base_dtype = dtype_from_fields(cloud.fields)
        dtype = np.dtype(
            {
                "names": base_dtype.names,
                "formats": [base_dtype.fields[name][0] for name in base_dtype.names],
                "offsets": [base_dtype.fields[name][1] for name in base_dtype.names],
                "itemsize": cloud.point_step,
            }
        )
    points = np.ndarray(
        shape=(cloud.width * cloud.height,),
        dtype=dtype,
        buffer=cloud.data,
    )
    if bool(sys.byteorder != "little") != bool(cloud.is_bigendian):
        points = points.byteswap(inplace=False)
    return points[list(fields)].copy()


def _unpack_packed_rgb(
    arr: np.ndarray, field: str
) -> tuple[np.ndarray, np.ndarray]:
    """Decode a structured point array with a packed ``rgb``/``rgba`` field.

    RealSense (and PCL) pack colour into a single 32-bit value ``0x00RRGGBB``
    (``rgb``) or ``0xAARRGGBB`` (``rgba``), commonly stored as a ``float32``
    whose bit pattern is the integer.

    Important: do **not** ask ``read_points(..., skip_nans=True)`` to filter this
    field. Many valid packed colour bit-patterns are NaN when interpreted as a
    float32, so filtering all fields silently drops valid coloured points. Mask
    NaNs on XYZ only after decoding.

    Returns ``(xyz Nx3 float32, rgb Nx3 uint8)`` in R, G, B order.
    """
    arr = np.asarray(arr)
    xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32)
    packed = arr[field]
    if packed.dtype.kind == "f":
        u32 = np.ascontiguousarray(packed, dtype=np.float32).view(np.uint32)
    else:
        u32 = packed.astype(np.uint32)
    r = ((u32 >> 16) & 0xFF).astype(np.uint8)
    g = ((u32 >> 8) & 0xFF).astype(np.uint8)
    b = (u32 & 0xFF).astype(np.uint8)
    rgb = np.column_stack([r, g, b])
    valid_xyz = np.isfinite(xyz).all(axis=1)
    return xyz[valid_xyz], rgb[valid_xyz]


def _read_xyz_rgb(cloud: PointCloud2) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Read XYZ plus RGB from PointCloud2.

    Handles both layouts: separate ``r``/``g``/``b`` uint8 fields, and the
    RealSense/PCL packed ``rgb``/``rgba`` 32-bit field.
    """
    try:
        from sensor_msgs_py.point_cloud2 import read_points

        names = {f.name for f in cloud.fields}

        if {"r", "g", "b"} <= names:
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

        rgb_field = "rgb" if "rgb" in names else ("rgba" if "rgba" in names else None)
        if rgb_field is not None:
            arr = _pointcloud2_to_array(cloud, ("x", "y", "z", rgb_field))
            if arr.size == 0:
                return None, None
            return _unpack_packed_rgb(arr, rgb_field)

        pts = list(read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            return None, None
        xyz = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        return xyz, None
    except Exception as exc:
        _LOGGER.warning(
            "point_cloud2 XYZ/RGB decode failed; falling back to XYZ-only: %s",
            exc,
            exc_info=True,
        )
        xyz = _read_xyz(cloud)
        return xyz, None


def _color_diagnostics(cluster_rgb: np.ndarray | None) -> dict:
    """Per-cluster colour diagnostics for debugging the 3D colour path.

    Returns rgb stats, a small sample, and the winning vote interpreting the
    data as RGB and (A/B) as BGR — to detect a channel-order mistake.
    """
    if cluster_rgb is None or len(cluster_rgb) == 0:
        return {"n": 0}
    a = np.asarray(cluster_rgb, dtype=np.uint8)
    name_rgb, conf_rgb = _dominant_color_voting(a)
    name_bgr, conf_bgr = _dominant_color_voting(a[:, ::-1])  # data-as-BGR
    return {
        "n": int(len(a)),
        "rgb_min": a.min(axis=0).tolist(),
        "rgb_max": a.max(axis=0).tolist(),
        "rgb_mean": a.mean(axis=0).round(1).tolist(),
        "sample": a[:10].tolist(),
        "vote_rgb": (name_rgb, round(float(conf_rgb), 2)),
        "vote_bgr": (name_bgr, round(float(conf_bgr), 2)),
    }

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



# Minimum chromatic vote ratio to prefer a colour over an achromatic majority.
# Lets a white object with a thin coloured border (e.g. blue_rectangle) classify
# by its border colour instead of the white interior.
_CHROMATIC_MIN_RATIO = 0.12


def _dominant_color_voting(rgb_pixels: np.ndarray | None) -> tuple[str, float]:
    """Classify cluster colour by per-pixel HSV voting instead of the mean.

    Returns ``(color_name, confidence)`` where confidence is the winning vote
    ratio (0..1). Chromatic colours win over an achromatic majority when their
    ratio exceeds ``_CHROMATIC_MIN_RATIO``. Voting is robust to multi-colour
    surfaces (logos/text) that would pollute a mean.
    """
    if rgb_pixels is None or len(rgb_pixels) == 0:
        return "unknown", 0.0
    try:
        import cv2

        hsv = cv2.cvtColor(
            rgb_pixels.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV
        ).reshape(-1, 3)
    except Exception as exc:
        _LOGGER.warning("HSV color voting failed: %s", exc, exc_info=True)
        return "unknown", 0.0

    h = hsv[:, 0].astype(np.int32)
    s = hsv[:, 1].astype(np.int32)
    v = hsv[:, 2].astype(np.int32)
    total = len(hsv)

    chromatic: dict[str, int] = {}
    for name, h_min, h_max, s_min, v_min in _COLOR_RANGES:
        match = (h >= h_min) & (h <= h_max) & (s >= s_min) & (v >= v_min)
        count = int(np.count_nonzero(match))
        if count:
            chromatic[name] = chromatic.get(name, 0) + count

    achromatic: dict[str, int] = {}
    achro_mask = s < 40
    if np.any(achro_mask):
        av = v[achro_mask]
        achromatic["white"] = int(np.count_nonzero(av > 180))
        achromatic["gray"] = int(np.count_nonzero((av > 60) & (av <= 180)))
        achromatic["black"] = int(np.count_nonzero(av <= 60))

    best_chromatic = max(chromatic.items(), key=lambda kv: kv[1], default=None)
    if best_chromatic is not None:
        name, count = best_chromatic
        ratio = count / total
        if ratio >= _CHROMATIC_MIN_RATIO:
            return name, float(ratio)

    best_achromatic = max(achromatic.items(), key=lambda kv: kv[1], default=None)
    if best_achromatic is not None and best_achromatic[1] > 0:
        if best_chromatic is not None and best_chromatic[1] >= best_achromatic[1]:
            return best_chromatic[0], float(best_chromatic[1] / total)
        return best_achromatic[0], float(best_achromatic[1] / total)
    if best_chromatic is not None:
        return best_chromatic[0], float(best_chromatic[1] / total)
    return "unknown", 0.0


# Multi-factor confidence weights. The temporal term is supplied by
# TemporalTracker (see scene_processor.py); when omitted the remaining
# weights are renormalised so a perfect color/geometry/depth detection
# scores 1.0.
_CONF_WEIGHTS = {"color": 0.35, "geometry": 0.25, "depth": 0.20, "temporal": 0.20}


def _confidence_score(
    color_conf: float,
    geometry_conf: float,
    depth_conf: float,
    temporal_conf: float | None = None,
) -> float:
    """Weighted detection confidence in [0, 1].

    When *temporal_conf* is None the temporal term is dropped and the other
    weights are renormalised to sum to 1.
    """
    terms = {
        "color": float(np.clip(color_conf, 0.0, 1.0)),
        "geometry": float(np.clip(geometry_conf, 0.0, 1.0)),
        "depth": float(np.clip(depth_conf, 0.0, 1.0)),
    }
    if temporal_conf is not None:
        terms["temporal"] = float(np.clip(temporal_conf, 0.0, 1.0))
    weight_sum = sum(_CONF_WEIGHTS[k] for k in terms)
    score = sum(_CONF_WEIGHTS[k] * terms[k] for k in terms) / weight_sum
    return float(np.clip(score, 0.0, 1.0))


# ---------------------------------------------------------------------------
#  Semantic class mapping — human/LLM-friendly object names
# ---------------------------------------------------------------------------
# Retained for debug metadata / future size-based refinement.
_BALL_MAX_DIM_M = 0.055  # ping pong ball ≈ 40 mm; apple ≈ 70–90 mm


def _max_dim_m(dims) -> float:
    """Return the largest PCA bounding-box extent in metres."""
    return max(float(dims.x), float(dims.y), float(dims.z))


def _has_chromatic_border(
    rgb_pixels: np.ndarray | None,
    cluster: np.ndarray | None = None,
    border_fraction: float = 0.20,
    min_ratio: float = 0.25,
) -> tuple[str | None, float]:
    """Detect whether a cluster's border pixels carry a chromatic colour.

    When a white/achromatic object has a coloured border (e.g. blue-bordered
    rectangle), the overall dominant-colour vote returns "white" because the
    interior pixels overwhelm the border.  This helper isolates *edge* pixels
    and runs HSV voting on them alone.

    Returns ``(color_name, ratio)`` of the strongest chromatic border colour,
    or ``(None, 0.0)`` if no significant chromatic border is found.
    """
    if rgb_pixels is None or len(rgb_pixels) < 10:
        return None, 0.0

    # If no 3D cluster is provided, use an index-based edge heuristic.
    if cluster is not None and len(cluster) == len(rgb_pixels):
        centroid = cluster.mean(axis=0)
        dists = np.linalg.norm(cluster - centroid, axis=1)
        if dists.max() < 1e-9:
            return None, 0.0
        # Border = outermost `border_fraction` of points by distance.
        threshold = np.quantile(dists, 1.0 - border_fraction)
        border_mask = dists >= threshold
    else:
        # Fallback: use first and last border_fraction of pixel indices.
        n = len(rgb_pixels)
        k = max(1, int(n * border_fraction / 2))
        border_mask = np.zeros(n, dtype=bool)
        border_mask[:k] = True
        border_mask[-k:] = True

    border_rgb = rgb_pixels[border_mask]
    if len(border_rgb) < 5:
        return None, 0.0

    # Run HSV voting on border pixels only.
    border_color, border_conf = _dominant_color_voting(border_rgb)
    if border_color in {"unknown", "white", "gray", "black"}:
        return None, 0.0
    if border_conf >= min_ratio:
        return border_color, border_conf
    return None, 0.0


def _semantic_class_id(
    color: str,
    shape: str,
    dims,
    rgb_pixels: np.ndarray | None = None,
    cluster: np.ndarray | None = None,
) -> str:
    """Map raw perception attributes to human/LLM-friendly semantic class_id.

    Policy:
    - Use *color* as the primary discriminator for the known fixed object set.
    - Use *shape* to avoid mapping red boxes as apples (sphere vs non-sphere).
    - When *color* is achromatic (white/gray) and *shape* is non-sphere,
      check border pixels for a chromatic colour (dual-color analysis).
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
    if color == "blue" and shape in {"box", "flat", "unknown"}:
        return "blue_rectangle"

    # -- White/gray non-sphere: check for chromatic border (dual-color) --
    if color in {"white", "gray"} and shape in {"box", "flat", "unknown"}:
        border_color, _ = _has_chromatic_border(rgb_pixels, cluster)
        if border_color == "blue":
            return "blue_rectangle"
        if border_color is not None:
            return f"{border_color}_{shape}"

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


def _voxel_downsample_indices(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Return indices of the points kept by grid-bin downsampling.

    Carrying indices (instead of points) lets callers slice an aligned RGB
    array through the same step without losing colour correspondence.
    """
    if pts.size == 0:
        return np.empty((0,), dtype=np.int64)
    coords = np.floor(pts / voxel_size).astype(np.int32)
    _, idx = np.unique(coords, axis=0, return_index=True)
    return idx


def _voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Simple grid-bin downsample."""
    if pts.size == 0:
        return pts
    return pts[_voxel_downsample_indices(pts, voxel_size)]


def _ransac_plane_fit(
    pts: np.ndarray,
    threshold: float,
    max_iter: int = 100,
    normal_z_min: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit the largest plane and return ``(inlier_idx, outlier_idx, normal)``.

    When ``normal_z_min > 0`` the plane is only treated as removable if its
    normal is close to vertical (``abs(normal_z) >= normal_z_min``), i.e. a
    table/floor. A near-vertical object face is kept: ``inlier_idx`` is empty
    and every point is returned as an outlier so it survives plane removal.
    """
    all_idx = np.arange(len(pts), dtype=np.int64)
    if len(pts) < 10:
        return np.empty((0,), dtype=np.int64), all_idx, np.array([0.0, 0.0, 1.0])
    best_inliers = np.array([], dtype=np.int64)
    best_normal = np.array([0.0, 0.0, 1.0])
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
            best_normal = normal
    # Reject the plane when its normal is not vertical enough (object face).
    if len(best_inliers) == 0 or abs(float(best_normal[2])) < normal_z_min:
        return np.empty((0,), dtype=np.int64), all_idx, best_normal
    mask = np.ones(len(pts), dtype=bool)
    mask[best_inliers] = False
    return best_inliers.astype(np.int64), all_idx[mask], best_normal


def _ransac_plane(
    pts: np.ndarray, threshold: float, max_iter: int = 100
) -> tuple[np.ndarray, np.ndarray]:
    """Return (inliers, outliers) for largest plane (back-compat wrapper)."""
    in_idx, out_idx, _ = _ransac_plane_fit(pts, threshold, max_iter, normal_z_min=0.0)
    return pts[in_idx], pts[out_idx]


def _euclidean_cluster_indices(
    pts: np.ndarray, tolerance: float, min_size: int, max_size: int
) -> list[np.ndarray]:
    """Euclidean clustering — returns per-cluster index arrays (PCL style).

    Returning indices lets the caller slice both the XYZ and the aligned RGB
    arrays with the same indices, preserving colour correspondence.
    """
    if len(pts) == 0:
        return []
    tree = cKDTree(pts)
    visited = np.zeros(len(pts), dtype=bool)
    clusters: list[np.ndarray] = []
    for i in range(len(pts)):
        if visited[i]:
            continue
        # Start a new cluster
        cluster = []
        queue = [i]
        visited[i] = True
        q_idx = 0
        while q_idx < len(queue):
            idx = queue[q_idx]
            q_idx += 1
            cluster.append(idx)
            neighbors = tree.query_ball_point(pts[idx], tolerance)
            for n in neighbors:
                if not visited[n]:
                    visited[n] = True
                    queue.append(n)
        if min_size <= len(cluster) <= max_size:
            clusters.append(np.asarray(cluster, dtype=np.int64))
    return clusters


def _euclidean_clusters(
    pts: np.ndarray, tolerance: float, min_size: int, max_size: int
) -> list[np.ndarray]:
    """Euclidean clustering returning point arrays (back-compat wrapper)."""
    return [pts[idx] for idx in _euclidean_cluster_indices(pts, tolerance, min_size, max_size)]


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
