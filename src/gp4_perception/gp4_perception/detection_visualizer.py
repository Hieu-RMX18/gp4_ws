"""Detection visualizer — overlay 3D detections on the 2D camera image.

Subscribes to:
    /perception/detections  (Detection3DArray in base_link frame)
    /camera/color/image_raw (Image)
    /camera/color/camera_info (CameraInfo)

Publishes:
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
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

from scipy.spatial.transform import Rotation

_WINDOW_NAME = "GP4 Perception"

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


def _build_3d_bbox_corners(pose, dims) -> np.ndarray:
    """Build 8 corners of a 3D oriented bounding box.

    Args:
        pose: geometry_msgs/Pose (centroid + orientation)
        dims: geometry_msgs/Vector3 (size x, y, z)

    Returns:
        (8, 3) float64 array of corner positions in the detection frame.
    """
    dx, dy, dz = dims.x / 2.0, dims.y / 2.0, dims.z / 2.0

    # 8 corners of the box in local frame (centered at origin).
    corners_local = np.array(
        [
            [-dx, -dy, -dz],
            [+dx, -dy, -dz],
            [+dx, +dy, -dz],
            [-dx, +dy, -dz],
            [-dx, -dy, +dz],
            [+dx, -dy, +dz],
            [+dx, +dy, +dz],
            [-dx, +dy, +dz],
        ],
        dtype=np.float64,
    )

    # Rotate corners by the bounding box orientation.
    q = pose.orientation
    rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
    corners_world = rot.apply(corners_local)

    # Translate to centroid.
    p = pose.position
    corners_world += np.array([p.x, p.y, p.z], dtype=np.float64)

    return corners_world


def _project_points_to_image(
    points_3d: np.ndarray,
    tf_base_to_optical: object,
    camera_matrix: np.ndarray,
) -> np.ndarray | None:
    """Project Nx3 points (in base_link) onto the image plane.

    Args:
        points_3d: (N, 3) in base_link frame.
        tf_base_to_optical: TF TransformStamped from base_link to camera_color_optical_frame.
        camera_matrix: 3x3 intrinsic matrix.

    Returns:
        (N, 2) pixel coordinates or None if transform fails.
    """
    t = tf_base_to_optical.transform.translation
    r = tf_base_to_optical.transform.rotation
    rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
    offset = np.array([t.x, t.y, t.z], dtype=np.float64)

    # lookupTransform("camera_color_optical_frame", "base_link") returns
    # R, T such that P_optical = R * P_base + T.
    pts_optical = rot.apply(points_3d) + offset

    # Filter points behind the camera (z <= 0).
    if np.any(pts_optical[:, 2] <= 0):
        # Some corners behind camera — still try to project what we can.
        pass

    # Project: pixel = K @ [x/z, y/z, 1]
    z = pts_optical[:, 2].copy()
    z[z <= 0.01] = 0.01  # avoid division by zero
    x_norm = pts_optical[:, 0] / z
    y_norm = pts_optical[:, 1] / z

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    pixels = np.stack(
        [
            fx * x_norm + cx,
            fy * y_norm + cy,
        ],
        axis=1,
    )

    return pixels


class DetectionVisualizer(Node):
    """Overlay 3D detection bounding boxes on the 2D camera image."""

    def __init__(self) -> None:
        super().__init__("detection_visualizer")

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Camera intrinsic matrix (populated from CameraInfo).
        self._camera_matrix: np.ndarray | None = None
        self._img_w = 640
        self._img_h = 480

        # Latest detections cache.
        self._latest_detections: Detection3DArray | None = None

        # --- OpenCV live window with bbox-filter toggle ---
        cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar(
            "BBox Filter", _WINDOW_NAME, 1, 1, self._on_bbox_trackbar
        )
        self._param_client = self.create_client(
            SetParameters, "/scene_processor/set_parameters"
        )

        # Low-latency QoS profile with depth=1 to discard old messages and prevent queue buildup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers.
        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self._on_camera_info,
            qos_profile,
        )
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self._on_image,
            qos_profile,
        )
        self.create_subscription(
            Detection3DArray,
            "/perception/detections",
            self._on_detections,
            10,
        )

        # Publisher.
        self._annotated_pub = self.create_publisher(
            Image, "/perception/annotated_image", 10
        )

        self._last_tf = None

        self.get_logger().info("DetectionVisualizer node started.")

    def _on_bbox_trackbar(self, val: int) -> None:
        """Toggle workspace bbox filter on scene_processor via ROS2 param."""
        if not self._param_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("scene_processor param service not available")
            return
        req = SetParameters.Request()
        param = ParameterMsg()
        param.name = "enable_bbox_filter"
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=bool(val)
        )
        req.parameters = [param]
        self._param_client.call_async(req)
        self.get_logger().info(
            "BBox filter %s", "ON" if val else "OFF"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Cache camera intrinsic matrix from CameraInfo."""
        k = msg.k  # 9-element row-major
        self._camera_matrix = np.array(k, dtype=np.float64).reshape(3, 3)
        self._img_w = msg.width
        self._img_h = msg.height

    def _on_detections(self, msg: Detection3DArray) -> None:
        """Cache latest detections."""
        self._latest_detections = msg

    def _on_image(self, msg: Image) -> None:
        """Receive camera image, overlay detections, publish annotated image."""
        if self._camera_matrix is None:
            return

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge conversion failed: {exc}")
            return

        # Look up TF: base_link → camera_color_optical_frame without blocking.
        tf_base_to_optical = None
        try:
            tf_base_to_optical = self._tf_buffer.lookup_transform(
                "camera_color_optical_frame",
                "base_link",
                Time(),
                timeout=Duration(seconds=0.0),
            )
            self._last_tf = tf_base_to_optical
        except Exception:
            last_tf = getattr(self, "_last_tf", None)
            if last_tf is not None:
                tf_base_to_optical = last_tf

        if tf_base_to_optical is None:
            # No TF yet — just publish the raw image with a status label.
            cv2.putText(
                cv_image,
                "Waiting for TF: base_link -> camera_color_optical_frame",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )
            self._publish_annotated(cv_image, msg.header)
            # Show in OpenCV window + process GUI events.
            cv2.imshow(_WINDOW_NAME, cv_image)
            cv2.waitKey(1)
            return

        detections = self._latest_detections
        n_detections = 0

        if detections and detections.detections:
            for det in detections.detections:
                if not det.results:
                    continue

                hyp = det.results[0]
                class_id = str(hyp.hypothesis.class_id)
                pose = hyp.pose.pose
                dims = det.bbox.size

                # Skip tiny bounding boxes (likely noise).
                max_dim = max(dims.x, dims.y, dims.z)
                if max_dim < 0.005:
                    continue

                color = _color_for_class(class_id)

                # Build 3D bbox corners and project to image.
                corners_3d = _build_3d_bbox_corners(pose, dims)
                pixels = _project_points_to_image(
                    corners_3d, tf_base_to_optical, self._camera_matrix
                )
                if pixels is None:
                    continue

                # Get 2D bounding rect from projected corners.
                px_int = pixels.astype(int)
                x_min = max(0, int(px_int[:, 0].min()))
                y_min = max(0, int(px_int[:, 1].min()))
                x_max = min(self._img_w - 1, int(px_int[:, 0].max()))
                y_max = min(self._img_h - 1, int(px_int[:, 1].max()))

                # Skip if projected box is outside image.
                if x_min >= x_max or y_min >= y_max:
                    continue

                # Draw 2D bounding box.
                cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)

                # Draw 3D wireframe edges (bottom face, top face, verticals).
                edges = [
                    # Bottom face.
                    (0, 1), (1, 2), (2, 3), (3, 0),
                    # Top face.
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    # Vertical edges.
                    (0, 4), (1, 5), (2, 6), (3, 7),
                ]
                for i0, i1 in edges:
                    pt1 = (int(pixels[i0, 0]), int(pixels[i0, 1]))
                    pt2 = (int(pixels[i1, 0]), int(pixels[i1, 1]))
                    # Clip to image bounds roughly.
                    if (
                        -500 < pt1[0] < self._img_w + 500
                        and -500 < pt1[1] < self._img_h + 500
                        and -500 < pt2[0] < self._img_w + 500
                        and -500 < pt2[1] < self._img_h + 500
                    ):
                        cv2.line(cv_image, pt1, pt2, color, 1)

                # Label with class_id and position.
                pos = pose.position
                label = f"{class_id} ({pos.x:.3f},{pos.y:.3f},{pos.z:.3f})"
                label_y = max(y_min - 8, 15)
                cv2.putText(
                    cv_image,
                    label,
                    (x_min, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                )

                n_detections += 1

        # Status bar at top.
        status = f"Detections: {n_detections}"
        cv2.putText(
            cv_image,
            status,
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        self._publish_annotated(cv_image, msg.header)

        # Show in OpenCV window + process GUI events.
        cv2.imshow(_WINDOW_NAME, cv_image)
        cv2.waitKey(1)

    def _publish_annotated(self, cv_image, header) -> None:
        """Publish annotated image."""
        try:
            annotated_msg = self._bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            annotated_msg.header = header
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
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
