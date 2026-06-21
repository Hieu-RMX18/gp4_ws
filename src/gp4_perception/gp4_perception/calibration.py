"""Hand-eye calibration service + TF publisher — merged module.

CalibrationService: implements /perception/calibrate_hand_eye (interfaces/srv/CalibrateHandEye).
TFPublisher: reads extrinsics.yaml and broadcasts base_link -> camera_link.

Solves AX = XB with cv2.calibrateHandEye (PARK method).
Expected usage:
  1. Launch calibration collection (fiducial on gripper, eye-to-hand setup).
  2. Jog robot through varied poses; node buffers (T_base->gripper, T_cam->target).
  3. Call /perception/calibrate_hand_eye with min_samples.
  4. Node solves, writes extrinsics.yaml with runtime-filled calibration_date.
  5. Launch tf_publisher node to broadcast the solved static transform.

Frame conventions:
  - OpenCV image-coordinate optical frames (Z forward, X right, Y down).
  - ROS uses right-hand FRD (X forward, Y left, Z up).
  - OpenCV target poses are estimated in camera_color_optical_frame.
  - The solver first produces base_link -> camera_color_optical_frame.
  - Before writing YAML, the node composes that solve with the RealSense
    internal TF camera_link <- camera_color_optical_frame.
  - The extrinsics in the YAML are always base_link -> camera_link, expressed
    as a ROS TransformStamped (translation + quaternion).
  - RealSense publishes camera_link -> camera_color_optical_frame separately.
  - When converting OpenCV Rodrigues + tvec to ROS quaternion, we use scipy.spatial
    to construct the quaternion from the rotation matrix.

Entry points:
  ros2 run gp4_perception calibration_service
  ros2 run gp4_perception tf_publisher
"""

from __future__ import annotations

import logging
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from interfaces.srv import CalibrateHandEye
from .safety_guards import check_reprojection_error

_LOGGER = logging.getLogger(__name__)
_CAMERA_ROOT_FRAME = "camera_link"
_COLOR_OPTICAL_FRAME = "camera_color_optical_frame"
_REPROJECTION_ERROR_MAX_MM = 5.0
_DUPLICATE_TRANSLATION_MAX_M = 0.010
_DUPLICATE_ROTATION_MAX_RAD = np.deg2rad(2.0)

# All OpenCV hand-eye solver methods, ordered by typical robustness.
# Two solvers only: PARK is the stable SE(3) primary; DANIILIDIS (dual
# quaternion) is an independent cross-check. Running all five added noise when
# a weak solver produced a falsely low residual.
_HAND_EYE_METHODS = [
    ("PARK", cv2.CALIB_HAND_EYE_PARK),
    ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
]

# Cross-check tolerance: warn when PARK and DANIILIDIS residuals diverge.
_SOLVER_DISAGREEMENT_MAX_MM = 2.0


@dataclass(frozen=True)
class FiducialConfig:
    target_type: str
    dictionary_name: str
    rows: int
    cols: int
    square_length_m: float
    marker_length_m: float

# Try to locate ArUco API across OpenCV versions.
try:
    _ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    _ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
except AttributeError:
    # OpenCV >= 4.7
    _ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    _ARUCO_PARAMS = cv2.aruco.DetectorParameters()


def _positive_float(raw: object, name: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"fiducials.{name} must be a positive number") from exc
    if value <= 0.0:
        raise ValueError(f"fiducials.{name} must be greater than zero")
    return value


def _positive_int(raw: object, name: str) -> int:
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"fiducials.{name} must be a positive integer") from exc
    if value <= 0:
        raise ValueError(f"fiducials.{name} must be greater than zero")
    return value


def _load_fiducial_config(fiducials_path: Path) -> FiducialConfig | None:
    """Load the physical calibration target geometry from fiducials.yaml."""
    if not fiducials_path.exists():
        _LOGGER.error("fiducials.yaml not found: %s", fiducials_path)
        return None

    try:
        with open(fiducials_path) as f:
            config = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as exc:
        _LOGGER.error("Failed to read fiducials.yaml at %s: %s", fiducials_path, exc)
        return None

    try:
        fiducials = config.get("fiducials") or {}
        target_type = str(fiducials.get("target_type", "aruco")).lower()
        if target_type not in {"aruco", "charuco"}:
            raise ValueError("fiducials.target_type must be 'aruco' or 'charuco'")

        dictionary_name = str(fiducials.get("marker_dictionary", "DICT_5X5_100"))
        if dictionary_name != "DICT_5X5_100":
            raise ValueError("fiducials.marker_dictionary must be DICT_5X5_100")

        marker_length_m = _positive_float(
            fiducials.get("marker_length_m"), "marker_length_m"
        )
        square_length_m = _positive_float(
            fiducials.get("square_length_m", marker_length_m),
            "square_length_m",
        )
        rows = _positive_int(fiducials.get("board_rows", 1), "board_rows")
        cols = _positive_int(fiducials.get("board_columns", 1), "board_columns")
    except ValueError as exc:
        _LOGGER.error("%s in %s", exc, fiducials_path)
        return None

    return FiducialConfig(
        target_type=target_type,
        dictionary_name=dictionary_name,
        rows=rows,
        cols=cols,
        square_length_m=square_length_m,
        marker_length_m=marker_length_m,
    )


def _load_fiducial_marker_length_m(fiducials_path: Path) -> float | None:
    """Load marker length for compatibility with existing tests/callers."""
    fiducials = _load_fiducial_config(fiducials_path)
    if fiducials is None:
        return None
    return fiducials.marker_length_m


def _create_charuco_board(fiducials: FiducialConfig, aruco_dict):
    size = (fiducials.cols, fiducials.rows)
    try:
        return cv2.aruco.CharucoBoard_create(
            fiducials.cols,
            fiducials.rows,
            fiducials.square_length_m,
            fiducials.marker_length_m,
            aruco_dict,
        )
    except AttributeError:
        return cv2.aruco.CharucoBoard(
            size,
            fiducials.square_length_m,
            fiducials.marker_length_m,
            aruco_dict,
        )


def _make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(rotation)
    transform[:3, 3] = np.asarray(translation).reshape(3)
    return transform


def _rotation_angle_rad(rotation: np.ndarray) -> float:
    cos_angle = (float(np.trace(rotation)) - 1.0) / 2.0
    return float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))


def _robot_pose_delta(
    rotation_a: np.ndarray,
    translation_a: np.ndarray,
    rotation_b: np.ndarray,
    translation_b: np.ndarray,
) -> tuple[float, float]:
    base_from_gripper_a = np.linalg.inv(_make_transform(rotation_a, translation_a))
    base_from_gripper_b = np.linalg.inv(_make_transform(rotation_b, translation_b))
    translation_delta_m = float(
        np.linalg.norm(base_from_gripper_a[:3, 3] - base_from_gripper_b[:3, 3])
    )
    rotation_delta_rad = _rotation_angle_rad(
        base_from_gripper_a[:3, :3].T @ base_from_gripper_b[:3, :3]
    )
    return translation_delta_m, rotation_delta_rad


def _is_duplicate_robot_pose(
    samples: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    rotation: np.ndarray,
    translation: np.ndarray,
    translation_threshold_m: float = _DUPLICATE_TRANSLATION_MAX_M,
    rotation_threshold_rad: float = _DUPLICATE_ROTATION_MAX_RAD,
) -> tuple[bool, float, float]:
    min_translation_m = float("inf")
    min_rotation_rad = float("inf")
    for sample_rotation, sample_translation, _, _ in samples:
        translation_delta_m, rotation_delta_rad = _robot_pose_delta(
            sample_rotation,
            sample_translation,
            rotation,
            translation,
        )
        min_translation_m = min(min_translation_m, translation_delta_m)
        min_rotation_rad = min(min_rotation_rad, rotation_delta_rad)
        if (
            translation_delta_m < translation_threshold_m
            and rotation_delta_rad < rotation_threshold_rad
        ):
            return True, translation_delta_m, rotation_delta_rad
    return False, min_translation_m, min_rotation_rad


def _pairwise_translation_residual_mm(
    samples: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    rotation_base_from_color: np.ndarray,
    translation_base_from_color: np.ndarray,
) -> float:
    base_from_color = _make_transform(
        rotation_base_from_color,
        translation_base_from_color,
    )
    color_from_base = np.linalg.inv(base_from_color)
    errors = []
    n_pairs = min(len(samples), 50)
    for i in range(n_pairs):
        rotation_gripper_from_base_i, translation_gripper_from_base_i, rotation_color_from_target_i, translation_color_from_target_i = samples[i]
        gripper_from_base_i = _make_transform(
            rotation_gripper_from_base_i,
            translation_gripper_from_base_i,
        )
        color_from_target_i = _make_transform(
            rotation_color_from_target_i,
            translation_color_from_target_i,
        )
        for j in range(i + 1, n_pairs):
            rotation_gripper_from_base_j, translation_gripper_from_base_j, rotation_color_from_target_j, translation_color_from_target_j = samples[j]
            gripper_from_base_j = _make_transform(
                rotation_gripper_from_base_j,
                translation_gripper_from_base_j,
            )
            color_from_target_j = _make_transform(
                rotation_color_from_target_j,
                translation_color_from_target_j,
            )
            robot_relative = np.linalg.inv(gripper_from_base_i) @ gripper_from_base_j
            camera_relative = (
                base_from_color
                @ color_from_target_i
                @ np.linalg.inv(color_from_target_j)
                @ color_from_base
            )
            errors.append(
                float(
                    np.linalg.norm(
                        robot_relative[:3, 3] - camera_relative[:3, 3]
                    )
                )
            )
    return float(np.median(errors)) * 1000.0 if errors else 0.0


def _run_solver(
    name: str,
    method: int,
    R_gripper: np.ndarray,
    t_gripper: np.ndarray,
    R_target: np.ndarray,
    t_target: np.ndarray,
    samples: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
) -> tuple[str, np.ndarray, np.ndarray, float] | None:
    """Run a single hand-eye solver; return (name, R, t, residual_mm) or None."""
    try:
        R_est, t_est = cv2.calibrateHandEye(
            R_gripper, t_gripper, R_target, t_target, method=method,
        )
        R_est = np.asarray(R_est)
        t_est = np.asarray(t_est).reshape(3)
        residual = _pairwise_translation_residual_mm(samples, R_est, t_est)
        _LOGGER.info("Solver %s: residual=%.2f mm", name, residual)
        return name, R_est, t_est, residual
    except Exception as exc:
        _LOGGER.warning("Solver %s failed: %s", name, exc)
        return None


def _solve_park_with_crosscheck(
    R_gripper: np.ndarray,
    t_gripper: np.ndarray,
    R_target: np.ndarray,
    t_target: np.ndarray,
    samples: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
) -> tuple[str, np.ndarray, np.ndarray, float] | None:
    """Solve with PARK (primary) and cross-check against DANIILIDIS.

    Always returns the PARK result, or None if PARK fails. Logs a warning when
    the two solvers' residuals diverge by more than
    ``_SOLVER_DISAGREEMENT_MAX_MM`` (a sign of noisy sample data).

    Returns (solver_name, R_cam2base, t_cam2base, residual_mm).
    """
    by_name = dict(_HAND_EYE_METHODS)
    park = _run_solver(
        "PARK", by_name["PARK"], R_gripper, t_gripper, R_target, t_target, samples
    )
    if park is None:
        _LOGGER.error("Primary solver PARK failed; calibration aborted.")
        return None

    crosscheck = _run_solver(
        "DANIILIDIS", by_name["DANIILIDIS"],
        R_gripper, t_gripper, R_target, t_target, samples,
    )
    if crosscheck is None:
        _LOGGER.warning(
            "Cross-check solver DANIILIDIS failed; using PARK without validation."
        )
    else:
        diff = abs(park[3] - crosscheck[3])
        if diff > _SOLVER_DISAGREEMENT_MAX_MM:
            _LOGGER.warning(
                "Hand-eye solvers disagree (PARK=%.2f mm, DANIILIDIS=%.2f mm, "
                "diff=%.2f mm > %.2f mm) — data may be noisy.",
                park[3], crosscheck[3], diff, _SOLVER_DISAGREEMENT_MAX_MM,
            )

    return park


def _reject_outlier_samples(
    samples: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    sigma_threshold: float = 2.0,
    min_samples: int = 8,
) -> tuple[list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]], int]:
    """Remove outlier samples using a leave-one-out residual check.

    1. Solve with PARK (fast) using all samples.
    2. Compute per-sample residual contribution.
    3. Remove samples whose contribution > median + sigma_threshold * MAD.
    4. Never remove below min_samples.

    Returns (cleaned_samples, n_rejected).
    """
    if len(samples) < min_samples + 2:
        return samples, 0

    # Quick solve with PARK to get initial extrinsic estimate
    R_arr = np.array([s[0] for s in samples])
    t_arr = np.array([s[1] for s in samples]).reshape(-1, 3, 1)
    R_t2c = np.array([s[2] for s in samples])
    t_t2c = np.array([s[3] for s in samples]).reshape(-1, 3, 1)
    try:
        R_est, t_est = cv2.calibrateHandEye(
            R_arr, t_arr, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_PARK,
        )
    except Exception:
        return samples, 0

    R_est = np.asarray(R_est)
    t_est = np.asarray(t_est).reshape(3)
    base_from_color = _make_transform(R_est, t_est)
    color_from_base = np.linalg.inv(base_from_color)

    # Compute per-sample average pairwise error
    n = len(samples)
    sample_errors = np.zeros(n)
    for i in range(n):
        gi = _make_transform(samples[i][0], samples[i][1])
        ci = _make_transform(samples[i][2], samples[i][3])
        pair_errors = []
        for j in range(n):
            if i == j:
                continue
            gj = _make_transform(samples[j][0], samples[j][1])
            cj = _make_transform(samples[j][2], samples[j][3])
            robot_rel = np.linalg.inv(gi) @ gj
            camera_rel = base_from_color @ ci @ np.linalg.inv(cj) @ color_from_base
            pair_errors.append(
                float(np.linalg.norm(robot_rel[:3, 3] - camera_rel[:3, 3]))
            )
        sample_errors[i] = float(np.median(pair_errors)) if pair_errors else 0.0

    # Robust threshold: median + sigma_threshold * MAD
    median_err = float(np.median(sample_errors))
    mad = float(np.median(np.abs(sample_errors - median_err)))
    threshold = median_err + sigma_threshold * max(mad, 1e-6)

    keep_mask = sample_errors <= threshold
    # Ensure we keep at least min_samples
    if np.sum(keep_mask) < min_samples:
        # Keep the min_samples with lowest error
        sorted_indices = np.argsort(sample_errors)
        keep_mask[:] = False
        keep_mask[sorted_indices[:min_samples]] = True

    cleaned = [s for s, keep in zip(samples, keep_mask) if keep]
    n_rejected = n - len(cleaned)
    return cleaned, n_rejected


def _transform_to_matrix(transform_stamped) -> np.ndarray:
    transform = getattr(transform_stamped, "transform", transform_stamped)
    translation = transform.translation
    rotation = transform.rotation
    return _make_transform(
        Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix(),
        np.array([[translation.x], [translation.y], [translation.z]]),
    )


def _base_from_camera_root(
    base_from_color: np.ndarray,
    root_from_color: np.ndarray,
) -> np.ndarray:
    return base_from_color @ np.linalg.inv(root_from_color)


class CalibrationService(Node):
    """Service node for hand-eye calibration."""

    def __init__(self, extrinsics_path: Path | None = None) -> None:
        super().__init__("calibration_service")
        self._bridge = CvBridge()
        self._lock = Lock()
        self._samples: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
        # (R_base2gripper, t_base2gripper, R_target2cam, t_target2cam)

        self._last_robot_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._last_robot_pose_time = 0.0

        if extrinsics_path is None:
            from ament_index_python.packages import get_package_share_directory

            share = Path(get_package_share_directory("gp4_perception"))
            extrinsics_path = share / "config" / "extrinsics.yaml"
        self._extrinsics_path = extrinsics_path
        self._fiducials_path = self._extrinsics_path.parent / "fiducials.yaml"
        self._fiducials = _load_fiducial_config(self._fiducials_path)
        self._marker_length_m = (
            self._fiducials.marker_length_m if self._fiducials is not None else None
        )
        self._charuco_board = (
            _create_charuco_board(self._fiducials, _ARUCO_DICT)
            if self._fiducials is not None
            and self._fiducials.target_type == "charuco"
            else None
        )
        self._marker_length_error_reported = False
        self._collecting = True  # Can be toggled via service
        self._max_samples = 200  # Auto-pause when reached

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._camera_info = None
        # RealSense v4.57.7 publishes with RELIABLE QoS but different durability
        # per topic: image_raw uses TRANSIENT_LOCAL, camera_info uses VOLATILE.
        _image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        _info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self._on_image,
            _image_qos,
        )
        from sensor_msgs.msg import CameraInfo

        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self._on_camera_info,
            _info_qos,
        )
        self._srv = self.create_service(
            CalibrateHandEye, "/perception/calibrate_hand_eye", self._handle
        )

        # --- Interactive control services for camera_preview ---
        from std_srvs.srv import SetBool, Trigger
        from std_msgs.msg import Int32

        self._sample_count_pub = self.create_publisher(Int32, "/perception/sample_count", 10)
        self._sample_count_timer = self.create_timer(0.5, self._publish_sample_count)

        self.create_service(
            SetBool, "/perception/toggle_collection", self._toggle_collection
        )
        self.create_service(
            Trigger, "/perception/clear_samples", self._clear_samples
        )

    def _publish_sample_count(self) -> None:
        from std_msgs.msg import Int32
        msg = Int32()
        with self._lock:
            msg.data = len(self._samples)
        self._sample_count_pub.publish(msg)

    def _toggle_collection(self, request, response):
        self._collecting = request.data
        state = "COLLECTING" if self._collecting else "PAUSED"
        self.get_logger().info(f"Collection {state}")
        response.success = True
        response.message = state
        return response

    def _clear_samples(self, request, response):
        with self._lock:
            count = len(self._samples)
            self._samples.clear()
        self.get_logger().info(f"Cleared {count} samples.")
        response.success = True
        response.message = f"Cleared {count} samples"
        return response

    def _on_camera_info(self, msg) -> None:
        self._camera_info = msg

    def _on_image(self, msg: Image) -> None:
        if not self._collecting:
            return

        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"cv_bridge conversion failed: {exc}")
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS
            )
        except Exception as exc:
            self.get_logger().warning(f"ArUco detectMarkers exception: {exc}")
            return
        if ids is None or len(ids) == 0:
            return

        self.get_logger().info(f"Detected {len(ids)} ArUco markers in frame.")

        # Look up robot pose – use latest available TF instead of image stamp
        # because MotoROS2 TF timestamps can lag behind camera timestamps
        # (different clock domains). Robot is stationary during sample capture,
        # so the latest TF is accurate enough.
        #
        # Eye-to-hand convention: cv2.calibrateHandEye needs T_base2gripper
        # (base TO gripper). lookup_transform("tool0", "base_link") returns
        # the transform that takes points FROM base_link INTO tool0, i.e.
        # T_base→gripper — exactly what the solver requires.
        try:
            t = self._tf_buffer.lookup_transform(
                "tool0", "base_link", rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as exc:
            self.get_logger().warning(f"[DIAG] TF lookup failed: {exc}")
            return

        # Estimate target pose from CameraInfo intrinsics and the configured board.
        if not hasattr(self, "_camera_info") or self._camera_info is None:
            self.get_logger().warning("[DIAG] CameraInfo is None")
            return

        # ROS2 Humble CameraInfo uses lowercase field names (k, d);
        # older versions used uppercase (K, D). Support both.
        K_raw = getattr(self._camera_info, 'K', None) or getattr(self._camera_info, 'k')
        D_raw = getattr(self._camera_info, 'D', None) or getattr(self._camera_info, 'd')
        K = np.array(K_raw).reshape(3, 3)
        D = np.array(D_raw)
        if self._marker_length_m is None:
            if not self._marker_length_error_reported:
                self.get_logger().error(
                    f"[DIAG] marker_length_m is None from {self._fiducials_path}"
                )
                self._marker_length_error_reported = True
            return
        pose = self._estimate_target_pose(corners, ids, gray, K, D)
        if pose is None:
            self.get_logger().info("[DIAG] Pose estimation returned None")
            return
        rvec, tvec = pose

        # Convert to matrices
        R_target2cam, _ = cv2.Rodrigues(rvec)
        t_target2cam = tvec.reshape(3, 1)

        # Robot pose: base_link -> tool0
        q = t.transform.rotation
        p = t.transform.translation
        R_base2gripper = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t_base2gripper = np.array([[p.x], [p.y], [p.z]])

        self.get_logger().info(
            f"[DIAG] TF OK, pose OK, robot=({p.x:.3f},{p.y:.3f},{p.z:.3f}), "
            f"n_samples={len(self._samples)}"
        )

        import time as _time
        now = _time.monotonic()

        with self._lock:
            # --- STATIONARY CHECK ---
            if self._last_robot_pose is not None:
                last_R, last_t = self._last_robot_pose
                trans_diff, rot_diff = _robot_pose_delta(
                    last_R, last_t, R_base2gripper, t_base2gripper
                )
                # If moved > 2mm or > 0.5 degrees, reset stationary timer
                if trans_diff > 0.002 or rot_diff > 0.0087:
                    self._last_robot_pose = (R_base2gripper, t_base2gripper)
                    self._last_robot_pose_time = now
                    return
                # Stationary, but not for 1 full second yet
                elif now - self._last_robot_pose_time < 1.0:
                    return
            else:
                self._last_robot_pose = (R_base2gripper, t_base2gripper)
                self._last_robot_pose_time = now
                return
            
            # If we reach here, the robot has been stationary for >= 1.0 seconds.
            duplicate, translation_delta_m, rotation_delta_rad = (
                _is_duplicate_robot_pose(
                    self._samples,
                    R_base2gripper,
                    t_base2gripper,
                )
            )
            if duplicate:
                import time as _time
                _now = _time.monotonic()
                _last = getattr(self, "_last_dup_log_t", 0.0)
                if _now - _last > 3.0:
                    self._last_dup_log_t = _now
                    self.get_logger().info(
                        f"[DIAG] DUPLICATE — delta: {translation_delta_m * 1000.0:.1f} mm / "
                        f"{np.rad2deg(rotation_delta_rad):.1f} deg, "
                        f"need >{_DUPLICATE_TRANSLATION_MAX_M * 1000:.0f} mm or "
                        f">{np.rad2deg(_DUPLICATE_ROTATION_MAX_RAD):.0f} deg"
                    )
                return
            if len(self._samples) < self._max_samples:
                self._samples.append(
                    (R_base2gripper, t_base2gripper, R_target2cam, t_target2cam)
                )
                n = len(self._samples)
                self.get_logger().info(f"Calibration sample {n} collected.")
                if n >= self._max_samples:
                    self._collecting = False
                    self.get_logger().info(
                        f"Reached {self._max_samples} samples — AUTO PAUSED. "
                        f"Press S to solve or R to reset."
                    )

    def _estimate_target_pose(
        self,
        corners,
        ids,
        gray: np.ndarray,
        K: np.ndarray,
        D: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        if self._fiducials is None or self._marker_length_m is None:
            if not self._marker_length_error_reported:
                _LOGGER.error(
                    "Skipping calibration samples because fiducial geometry "
                    "is unavailable from %s",
                    self._fiducials_path,
                )
                self._marker_length_error_reported = True
            return None

        if self._fiducials.target_type == "charuco":
            if self._charuco_board is None:
                _LOGGER.error("Charuco board is not configured; skipping sample.")
                return None
            retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners,
                ids,
                gray,
                self._charuco_board,
                cameraMatrix=K,
                distCoeffs=D,
            )
            if charuco_ids is None or retval < 6:
                _LOGGER.debug(
                    "Only %s Charuco corners detected; skipping sample.", retval
                )
                return None
            try:
                ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners,
                    charuco_ids,
                    self._charuco_board,
                    K,
                    D,
                    None,
                    None,
                )
            except TypeError:
                ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners,
                    charuco_ids,
                    self._charuco_board,
                    K,
                    D,
                )
            if not ok:
                return None
            return np.asarray(rvec).reshape(3), np.asarray(tvec).reshape(3)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self._marker_length_m, K, D
        )
        if rvecs is None or len(rvecs) == 0:
            return None
        return np.asarray(rvecs[0]).reshape(3), np.asarray(tvecs[0]).reshape(3)

    def _handle(
        self, request: CalibrateHandEye.Request, response: CalibrateHandEye.Response
    ) -> CalibrateHandEye.Response:
        with self._lock:
            n = len(self._samples)
        if n < request.min_samples:
            response.success = False
            response.failure_reason = (
                f"only {n} samples collected (min {request.min_samples})"
            )
            response.n_samples_collected = n
            return response

        # Unpack
        with self._lock:
            samples = self._samples[:]

        # Eye-to-hand: pass R_base2gripper directly (NOT inverted).
        # OpenCV calibrateHandEye expects (gripper2base, target2cam) for
        # eye-in-hand. For eye-to-hand, passing base2gripper instead
        # makes the solver return camera-to-base (not camera-to-gripper).
        R_base2gripper_list = []
        t_base2gripper_list = []
        R_target2cam = []
        t_target2cam = []
        for R_b2g, t_b2g, R_t2c, t_t2c in samples:
            R_base2gripper_list.append(R_b2g)
            t_base2gripper_list.append(t_b2g)
            R_target2cam.append(R_t2c)
            t_target2cam.append(t_t2c)

        R_base2gripper_arr = np.array(R_base2gripper_list)
        t_base2gripper_arr = np.array(t_base2gripper_list).reshape(-1, 3, 1)
        R_target2cam = np.array(R_target2cam)
        t_target2cam = np.array(t_target2cam).reshape(-1, 3, 1)

        # --- RANSAC-style outlier rejection ---
        # Solve once with all samples, then remove outliers whose per-sample
        # residual exceeds 2σ from the median, and re-solve with the clean set.
        samples, n_rejected = _reject_outlier_samples(samples)
        if n_rejected > 0:
            self.get_logger().info(
                f"Outlier rejection removed {n_rejected} sample(s); {len(samples)} remain."
            )
        n = len(samples)  # update count after rejection

        # Re-unpack after outlier rejection
        R_base2gripper_list = [s[0] for s in samples]
        t_base2gripper_list = [s[1] for s in samples]
        R_target2cam_list = [s[2] for s in samples]
        t_target2cam_list = [s[3] for s in samples]
        R_base2gripper_arr = np.array(R_base2gripper_list)
        t_base2gripper_arr = np.array(t_base2gripper_list).reshape(-1, 3, 1)
        R_target2cam = np.array(R_target2cam_list)
        t_target2cam = np.array(t_target2cam_list).reshape(-1, 3, 1)

        # --- PARK primary + DANIILIDIS cross-check ---
        best_result = _solve_park_with_crosscheck(
            R_base2gripper_arr, t_base2gripper_arr,
            R_target2cam, t_target2cam,
            samples,
        )
        if best_result is None:
            response.success = False
            response.failure_reason = "Primary hand-eye solver (PARK) failed."
            response.n_samples_collected = n
            return response

        solver_name, R_cam2base, t_cam2base, reproj_mm = best_result
        self.get_logger().info(
            f"Solver: {solver_name} (residual={reproj_mm:.2f} mm)"
        )

        if not np.isfinite(reproj_mm) or reproj_mm > _REPROJECTION_ERROR_MAX_MM:
            response.success = False
            response.failure_reason = (
                f"reprojection_error_mm = {reproj_mm:.2f} > max "
                f"{_REPROJECTION_ERROR_MAX_MM:.2f}; calibration not saved"
            )
            response.reprojection_error_mm = float(reproj_mm)
            response.n_samples_collected = n
            return response

        try:
            root_from_color_tf = self._tf_buffer.lookup_transform(
                _CAMERA_ROOT_FRAME,
                _COLOR_OPTICAL_FRAME,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5),
            )
            root_from_color = _transform_to_matrix(root_from_color_tf)
        except Exception as exc:
            response.success = False
            response.failure_reason = (
                f"camera internal TF {_CAMERA_ROOT_FRAME} <- "
                f"{_COLOR_OPTICAL_FRAME} unavailable: {exc}"
            )
            response.reprojection_error_mm = float(reproj_mm)
            response.n_samples_collected = n
            return response

        base_from_color = _make_transform(R_cam2base, t_cam2base)
        base_from_root = _base_from_camera_root(base_from_color, root_from_color)
        R_base_from_root = base_from_root[:3, :3]
        t_base_from_root = base_from_root[:3, 3]
        quat = Rotation.from_matrix(R_base_from_root).as_quat()  # [x, y, z, w]

        # Compute mean workspace distance
        distances = []
        for _, _, _, t_t2c in samples:
            distances.append(float(np.linalg.norm(t_t2c)))
        workspace_dist = float(np.mean(distances)) if distances else 0.0

        cal_date = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        extrinsics = {
            "hand_eye_extrinsics": {
                "parent_frame": "base_link",
                "child_frame": _CAMERA_ROOT_FRAME,
                "translation": {
                    "x": float(t_base_from_root[0]),
                    "y": float(t_base_from_root[1]),
                    "z": float(t_base_from_root[2]),
                },
                "rotation_quat": {
                    "x": float(quat[0]),
                    "y": float(quat[1]),
                    "z": float(quat[2]),
                    "w": float(quat[3]),
                },
                "calibration_date": cal_date,
                "reprojection_error_mm": round(float(reproj_mm), 3),
                "n_samples": n,
                "solver": solver_name,
                "workspace_distance_m": round(workspace_dist, 3),
            }
        }

        self._extrinsics_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._extrinsics_path, "w") as f:
            yaml.dump(extrinsics, f, default_flow_style=False, sort_keys=False)

        response.success = True
        response.failure_reason = ""
        response.extrinsics_yaml_path = str(self._extrinsics_path)
        response.reprojection_error_mm = float(reproj_mm)
        response.n_samples_collected = n
        response.calibration_date_iso = cal_date

        # Don't clear samples — user can re-solve or verify.
        # Use R (clear_samples service) to manually reset.
        return response


class TFPublisher(Node):
    """Broadcasts the hand-eye static transform from extrinsics.yaml on startup."""

    def __init__(self, extrinsics_path: Path | None = None) -> None:
        super().__init__("gp4_perception_tf_publisher")
        if extrinsics_path is None:
            from ament_index_python.packages import get_package_share_directory

            share = Path(get_package_share_directory("gp4_perception"))
            extrinsics_path = share / "config" / "extrinsics.yaml"
        self._path = extrinsics_path
        self._broadcaster = StaticTransformBroadcaster(self)
        self._load_and_publish()

    def _load_and_publish(self) -> None:
        if not self._path.exists():
            _LOGGER.error("extrinsics.yaml not found: %s", self._path)
            return
        with open(self._path) as f:
            data = yaml.safe_load(f) or {}
        ok, reason = check_reprojection_error(data, _REPROJECTION_ERROR_MAX_MM)
        if not ok:
            _LOGGER.error("Calibration not valid: %s", reason)
            return
        extrinsics = data.get("hand_eye_extrinsics", {})
        cal_date = extrinsics.get("calibration_date", "")
        if not cal_date or cal_date == "<NOT_CALIBRATED>":
            _LOGGER.error(
                "Calibration not valid (calibration_date='%s'). "
                "Run /perception/calibrate_hand_eye before launching tf_publisher.",
                cal_date,
            )
            return
        parent = extrinsics.get("parent_frame", "base_link")
        child = extrinsics.get("child_frame", _CAMERA_ROOT_FRAME)
        if parent != "base_link" or child != _CAMERA_ROOT_FRAME:
            _LOGGER.error(
                "Calibration not valid: expected parent_frame='base_link' and "
                "child_frame='%s', got parent_frame='%s', child_frame='%s'",
                _CAMERA_ROOT_FRAME,
                parent,
                child,
            )
            return
        t = extrinsics.get("translation", {})
        q = extrinsics.get("rotation_quat", {})
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = float(t.get("x", 0.0))
        tf_msg.transform.translation.y = float(t.get("y", 0.0))
        tf_msg.transform.translation.z = float(t.get("z", 0.0))
        tf_msg.transform.rotation.x = float(q.get("x", 0.0))
        tf_msg.transform.rotation.y = float(q.get("y", 0.0))
        tf_msg.transform.rotation.z = float(q.get("z", 0.0))
        tf_msg.transform.rotation.w = float(q.get("w", 1.0))
        self._broadcaster.sendTransform(tf_msg)
        _LOGGER.info("Published static transform %s -> %s", parent, child)


def main_calibration_service(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = CalibrationService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main_tf_publisher(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = TFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main_calibration_service())
