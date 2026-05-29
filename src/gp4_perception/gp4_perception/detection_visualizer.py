"""RGB + aligned-depth object detection and visualization node.

Subscribes to:
    /camera/color/image_raw (Image)
    /camera/aligned_depth_to_color/image_raw (Image, 16UC1)
    /camera/color/camera_info (CameraInfo)

Publishes:
    /perception/detections (Detection3DArray)
    /perception/annotated_image (Image with bounding boxes + labels)

Entry point: ros2 run gp4_perception detection_visualizer
"""

from __future__ import annotations

import logging
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

_LOGGER = logging.getLogger(__name__)


# Color palette for different class labels (BGR for OpenCV).
_COLORS = {
    "red": (0, 0, 255),
    "orange": (0, 140, 255),
    "yellow": (0, 255, 255),
    "green": (0, 200, 0),
    "blue": (255, 100, 0),
    "purple": (200, 0, 200),
    "white": (255, 255, 255),
    "gray": (160, 160, 160),
    "black": (80, 80, 80),
}
_DEFAULT_COLOR = (0, 255, 128)  # bright green


def _color_for_class(class_id: str) -> tuple[int, int, int]:
    """Pick a display color based on the class_id prefix (color name)."""
    parts = class_id.split("_") if class_id else []
    if parts:
        color_name = parts[0].lower()
        if color_name in _COLORS:
            return _COLORS[color_name]
    return _DEFAULT_COLOR


def _deproject_pixel(
    u: float, v: float, z_m: float, fx: float, fy: float, cx: float, cy: float
) -> tuple[float, float, float]:
    """Pinhole deprojection of one pixel to camera_color_optical_frame metres."""
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return (x, y, z_m)


def _median_depth_m(
    depth_raw: np.ndarray | None,
    mask: np.ndarray | None,
    depth_scale: float = 0.001,
) -> float | None:
    """Median depth in metres over masked, nonzero pixels.

    Returns None when no valid (masked & nonzero) pixel exists.
    """
    if depth_raw is None or mask is None:
        return None
    valid = (mask > 0) & (depth_raw > 0)
    sel = depth_raw[valid]
    if sel.size == 0:
        return None
    return float(np.median(sel)) * depth_scale


def _bbox_size_m(
    w_px: float, h_px: float, z_m: float, fx: float, fy: float
) -> tuple[float, float]:
    """Estimate metric bbox width/height from pixel extent at depth z."""
    return (w_px * z_m / fx, h_px * z_m / fy)


# ---------------------------------------------------------------------------
# HSV multi-range contour detection (Task 3)
# ---------------------------------------------------------------------------

def detect_color_objects(
    rgb_image: np.ndarray,
    color_classes: list[dict],
) -> list[dict]:
    """Detect objects via HSV masking per configured color class.

    Args:
        rgb_image: H×W×3 uint8 RGB image.
        color_classes: List of class configs from perception.yaml color_classes.

    Returns:
        List of dicts with keys: class_id, bbox (x,y,w,h), mask, contour,
        center_uv, confidence.
    """
    results: list[dict] = []
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    for cls in color_classes:
        if not cls.get("enabled", True):
            continue
        class_id = cls["class_id"]
        ranges = cls.get("hsv_ranges", [])
        min_area = cls.get("min_area_px", 300)
        kern_sz = cls.get("morph_kernel", 3)

        # Union of all HSV ranges for this class.
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for rng in ranges:
            lo = np.array(rng[:3], dtype=np.uint8)
            hi = np.array(rng[3:], dtype=np.uint8)
            combined_mask |= cv2.inRange(hsv, lo, hi)

        # Morphology cleanup.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kern_sz, kern_sz))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            # Build per-object mask.
            obj_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(obj_mask, [cnt], -1, 255, -1)
            # Border-hue gate.
            if cls.get("require_border"):
                if not validate_border_hue(
                    rgb_image, obj_mask, (x, y, w, h), cls["require_border"]
                ):
                    continue
            # Confidence from mask fill ratio (solidity).
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            confidence = area / hull_area if hull_area > 0 else 0.0
            cx = x + w // 2
            cy = y + h // 2
            results.append({
                "class_id": class_id,
                "bbox": (x, y, w, h),
                "mask": obj_mask,
                "contour": cnt,
                "center_uv": (cx, cy),
                "confidence": float(confidence),
            })
    return results


# ---------------------------------------------------------------------------
# 2D border-hue validation (Task 4)
# ---------------------------------------------------------------------------

# HSV ranges for border colour validation (OpenCV HSV: H 0-179).
_BORDER_HSV_RANGES: dict[str, list[tuple[int, ...]]] = {
    "blue": [(100, 80, 40, 130, 255, 255)],
    "red": [(0, 80, 50, 10, 255, 255), (170, 80, 50, 179, 255, 255)],
    "green": [(35, 60, 40, 85, 255, 255)],
}


def validate_border_hue(
    rgb_image: np.ndarray,
    mask: np.ndarray,
    bbox: tuple[int, int, int, int],
    required_border: str | None,
) -> bool:
    """Check if the border pixels of a detected object match the required hue.

    Uses morphological erosion to isolate a border ring, then HSV-votes
    the border pixels for the required colour.

    Args:
        rgb_image: Full H×W×3 uint8 RGB image.
        mask: H×W uint8 object mask (255 = object).
        bbox: (x, y, w, h) bounding box.
        required_border: Colour name ("blue", "red", "green") or None.

    Returns:
        True if no border required, or if ≥25% of border pixels match.
    """
    if required_border is None:
        return True

    x, y, w, h = bbox
    roi_mask = mask[y:y + h, x:x + w]
    if roi_mask.sum() == 0:
        return False

    # Erode to get interior, subtract to get border ring.
    # borderValue=0 ensures edge pixels are excluded from interior.
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    interior = cv2.erode(
        roi_mask, kern, iterations=2,
        borderType=cv2.BORDER_CONSTANT, borderValue=0,
    )
    border_ring = cv2.subtract(roi_mask, interior)
    if cv2.countNonZero(border_ring) < 10:
        return False

    roi_rgb = rgb_image[y:y + h, x:x + w]
    border_pixels = roi_rgb[border_ring > 0]

    ranges = _BORDER_HSV_RANGES.get(required_border.lower(), [])
    if not ranges:
        return False

    hsv_pixels = cv2.cvtColor(
        border_pixels.reshape(-1, 1, 3), cv2.COLOR_RGB2HSV
    ).reshape(-1, 3)
    match_count = 0
    for rng in ranges:
        lo = np.array(rng[:3], dtype=np.uint8)
        hi = np.array(rng[3:], dtype=np.uint8)
        match_count += int(np.count_nonzero(
            cv2.inRange(hsv_pixels.reshape(-1, 1, 3), lo, hi)
        ))
    ratio = match_count / len(hsv_pixels)
    return ratio >= 0.25


class DetectionVisualizer(Node):
    """RGB + aligned-depth object detection and visualization node.

    Subscribes to:
        /camera/color/image_raw (Image)
        /camera/aligned_depth_to_color/image_raw (Image, 16UC1)
        /camera/color/camera_info (CameraInfo)

    Publishes:
        /perception/detections (Detection3DArray)
        /perception/annotated_image (Image with bounding boxes + labels)
    """

    def __init__(self) -> None:
        super().__init__("detection_visualizer")

        # Load config.
        try:
            from pathlib import Path
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory("gp4_perception")) / "config"
        except Exception:
            from pathlib import Path
            share = Path(__file__).resolve().parents[1] / "config"

        import yaml
        with open(share / "perception.yaml") as f:
            cfg = yaml.safe_load(f) or {}
        pcfg = cfg.get("perception", {})
        self._color_classes: list[dict] = pcfg.get("color_classes", [])
        rgb_cfg = pcfg.get("rgb_detector", {})
        viz_cfg = pcfg.get("visualization", {})

        self._depth_scale = float(rgb_cfg.get("depth_scale_m", 0.001))
        self._base_frame = str(rgb_cfg.get("base_frame", "base_link"))
        self._camera_frame = str(rgb_cfg.get("camera_optical_frame",
                                              "camera_color_optical_frame"))
        self._bbox_z = float(rgb_cfg.get("bbox_thickness_z_m", 0.03))
        self._sync_slop = float(rgb_cfg.get("sync_slop_s", 0.05))
        self._sync_queue = int(rgb_cfg.get("sync_queue", 10))
        self._show_depth_panel = bool(viz_cfg.get("show_depth_panel", True))
        self._output_width = int(viz_cfg.get("output_width_px", 960))

        color_topic = str(rgb_cfg.get("color_topic", "/camera/color/image_raw"))
        depth_topic = str(rgb_cfg.get("depth_topic",
                                       "/camera/aligned_depth_to_color/image_raw"))
        info_topic = str(rgb_cfg.get("camera_info_topic",
                                      "/camera/color/camera_info"))

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_tf = None

        # Camera intrinsics (populated from CameraInfo).
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None

        # Low-latency QoS.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # CameraInfo subscriber — cached intrinsics, not time-synced.
        self.create_subscription(CameraInfo, info_topic,
                                 self._on_camera_info, qos)

        # Synchronized color + depth.
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        self._color_sub = Subscriber(self, Image, color_topic, qos_profile=qos)
        self._depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)
        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=self._sync_queue,
            slop=self._sync_slop,
        )
        self._sync.registerCallback(self._on_synced_rgbd)

        # Publishers.
        self._det_pub = self.create_publisher(
            Detection3DArray, "/perception/detections", 10
        )
        self._annotated_pub = self.create_publisher(
            Image, "/perception/annotated_image", 10
        )

        self.get_logger().info(
            "DetectionVisualizer node started (RGB+depth path, %d color classes).",
            len(self._color_classes),
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Cache camera intrinsics from CameraInfo."""
        k = msg.k  # 9-element row-major
        self._fx = float(k[0])
        self._fy = float(k[4])
        self._cx = float(k[2])
        self._cy = float(k[5])

    def _on_synced_rgbd(self, color_msg: Image, depth_msg: Image) -> None:
        """Process synchronized color + aligned-depth frame."""
        if self._fx is None:
            return  # no intrinsics yet

        # Decode images.
        try:
            rgb = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge color conversion failed: {exc}")
            return
        try:
            depth_raw = self._bridge.imgmsg_to_cv2(depth_msg,
                                                    desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge depth conversion failed: {exc}")
            return

        # 2D detection.
        objects = detect_color_objects(rgb, self._color_classes)

        # TF: camera_color_optical_frame → base_link (non-blocking).
        tf_cam_to_base = None
        try:
            tf_cam_to_base = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._camera_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            self._last_tf = tf_cam_to_base
        except Exception:
            if self._last_tf is not None:
                tf_cam_to_base = self._last_tf

        # Build Detection3DArray.
        det_arr = Detection3DArray()
        overlay_rgb = rgb.copy()  # annotate on a copy
        n_published = 0

        for obj in objects:
            class_id = obj["class_id"]
            x, y, w, h = obj["bbox"]
            mask = obj["mask"]
            cx_px, cy_px = obj["center_uv"]
            confidence = obj["confidence"]

            # Median depth over mask within bbox.
            depth_roi = depth_raw[y:y + h, x:x + w]
            mask_roi = mask[y:y + h, x:x + w]
            z_m = _median_depth_m(depth_roi, mask_roi,
                                   depth_scale=self._depth_scale)

            # Overlay colour.
            color_bgr = _color_for_class(class_id)

            # Draw 2D bounding box on overlay (BGR).
            cv2.rectangle(overlay_rgb, (x, y), (x + w, y + h), color_bgr, 2)

            if z_m is None or z_m <= 0.0:
                # Depth invalid — draw bbox but don't publish detection.
                label = f"{class_id} XYZ_INVALID"
                cv2.putText(overlay_rgb, label, (x, max(y - 8, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)
                continue

            # Deproject center to camera XYZ.
            cam_x, cam_y, cam_z = _deproject_pixel(
                float(cx_px), float(cy_px), z_m,
                self._fx, self._fy, self._cx, self._cy,
            )
            distance_m = z_m

            # Transform to base_link.
            base_xyz_str = "TF_UNAVAILABLE"
            frame_id = self._camera_frame
            det_x, det_y, det_z = cam_x, cam_y, cam_z

            if tf_cam_to_base is not None:
                try:
                    from geometry_msgs.msg import PointStamped
                    pt = PointStamped()
                    pt.header.frame_id = self._camera_frame
                    pt.header.stamp = color_msg.header.stamp
                    pt.point.x = cam_x
                    pt.point.y = cam_y
                    pt.point.z = cam_z
                    # Manual transform using the cached TF.
                    from scipy.spatial.transform import Rotation
                    t = tf_cam_to_base.transform.translation
                    r = tf_cam_to_base.transform.rotation
                    rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
                    cam_pt = np.array([cam_x, cam_y, cam_z])
                    base_pt = rot.apply(cam_pt) + np.array([t.x, t.y, t.z])
                    det_x, det_y, det_z = float(base_pt[0]), float(base_pt[1]), float(base_pt[2])
                    frame_id = self._base_frame
                    base_xyz_str = f"({det_x:.3f},{det_y:.3f},{det_z:.3f})"
                except Exception as exc:
                    self.get_logger().debug(f"TF transform failed: {exc}")

            # Build Detection3D.
            from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
            from geometry_msgs.msg import PoseWithCovariance
            from vision_msgs.msg import (
                Detection3D, ObjectHypothesis, ObjectHypothesisWithPose,
            )
            from std_msgs.msg import Header

            det = Detection3D()
            det.header = Header(
                stamp=color_msg.header.stamp,
                frame_id=frame_id,
            )
            pose = Pose()
            pose.position = Point(x=det_x, y=det_y, z=det_z)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            hyp = ObjectHypothesis()
            hyp.class_id = class_id
            hyp.score = float(confidence)
            det.results.append(ObjectHypothesisWithPose(
                hypothesis=hyp,
                pose=PoseWithCovariance(pose=pose),
            ))
            # Bbox size from pixel extent at depth.
            sx, sy = _bbox_size_m(float(w), float(h), z_m,
                                   self._fx, self._fy)
            det.bbox.size = Vector3(x=sx, y=sy, z=self._bbox_z)
            det_arr.detections.append(det)
            n_published += 1

            # Overlay label.
            cam_xyz_str = f"({cam_x:.3f},{cam_y:.3f},{cam_z:.3f})"
            lines = [
                f"{class_id} conf={confidence:.2f}",
                f"d={distance_m:.3f}m cam={cam_xyz_str}",
                f"base={base_xyz_str}",
            ]
            for li, line in enumerate(lines):
                label_y = max(y - 8 - (len(lines) - 1 - li) * 14, 15)
                cv2.putText(overlay_rgb, line, (x, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color_bgr, 1)

        # Status bar.
        cv2.putText(overlay_rgb, f"Detections: {n_published}",
                     (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish Detection3DArray.
        det_arr.header = Header(
            stamp=color_msg.header.stamp,
            frame_id=self._base_frame if tf_cam_to_base else self._camera_frame,
        )
        self._det_pub.publish(det_arr)

        # Build annotated image (optionally with depth panel).
        if self._show_depth_panel:
            # Colourmap the aligned depth.
            depth_vis = cv2.normalize(depth_raw, None, 0, 255,
                                       cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            # depth_color is BGR; overlay_rgb is RGB — convert for consistency.
            depth_color_rgb = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)
            # Resize depth to match colour image height.
            if depth_color_rgb.shape[0] != overlay_rgb.shape[0]:
                scale = overlay_rgb.shape[0] / depth_color_rgb.shape[0]
                new_w = int(depth_color_rgb.shape[1] * scale)
                depth_color_rgb = cv2.resize(depth_color_rgb, (new_w, overlay_rgb.shape[0]))
            combined = np.hstack([overlay_rgb, depth_color_rgb])
        else:
            combined = overlay_rgb

        # Scale to output width.
        if combined.shape[1] != self._output_width:
            scale = self._output_width / combined.shape[1]
            new_h = int(combined.shape[0] * scale)
            combined = cv2.resize(combined, (self._output_width, new_h))

        # Publish annotated image (convert RGB → BGR for standard encoding).
        try:
            bgr = cv2.cvtColor(combined, cv2.COLOR_RGB2BGR)
            annotated_msg = self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            annotated_msg.header = color_msg.header
            self._annotated_pub.publish(annotated_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish annotated image: {exc}")


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = DetectionVisualizer()
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

