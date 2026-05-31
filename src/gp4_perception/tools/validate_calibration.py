#!/usr/bin/env python3
"""Calibration validation tool.

Detects the Charuco board, transforms its center to robot base_link frame
using the calibrated base_link -> camera_link extrinsics plus the RealSense
camera_link -> camera_color_optical_frame TF, and compares with the current
tool0 (TCP) position. Jog the TCP to touch the board center to verify the
calibration.

Keyboard:
    V  — Capture validation snapshot (freeze current error reading)
    Q  — Quit

Usage:
    python3 tools/validate_calibration.py
"""

import yaml

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from pathlib import Path


# ArUco / Charuco config (match fiducials.yaml)
_ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
_BASE_FRAME = "base_link"
_CAMERA_ROOT_FRAME = "camera_link"
_COLOR_OPTICAL_FRAME = "camera_color_optical_frame"


def _load_board_config() -> tuple[int, int, float, float]:
    try:
        from ament_index_python.packages import get_package_share_directory

        config_dir = Path(get_package_share_directory("gp4_perception")) / "config"
    except Exception:
        config_dir = Path(__file__).resolve().parents[1] / "config"
    with open(config_dir / "fiducials.yaml") as f:
        data = yaml.safe_load(f) or {}
    fiducials = data.get("fiducials") or {}
    return (
        int(fiducials["board_rows"]),
        int(fiducials["board_columns"]),
        float(fiducials["square_length_m"]),
        float(fiducials["marker_length_m"]),
    )


_BOARD_ROWS, _BOARD_COLS, _SQUARE_LEN, _MARKER_LEN = _load_board_config()
_CHARUCO_BOARD = cv2.aruco.CharucoBoard_create(
    _BOARD_COLS, _BOARD_ROWS, _SQUARE_LEN, _MARKER_LEN, _ARUCO_DICT
)

# UI Colors
_GREEN = (0, 255, 0)
_RED = (0, 0, 255)
_YELLOW = (0, 255, 255)
_CYAN = (255, 255, 0)
_WHITE = (255, 255, 255)
_GRAY = (180, 180, 180)
_BG = (30, 30, 30)
_MAGENTA = (255, 0, 255)


def _transform_to_matrix(transform_stamped) -> np.ndarray:
    transform = getattr(transform_stamped, "transform", transform_stamped)
    translation = transform.translation
    rotation = transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def _board_position_in_base(
    board_pos_color_optical: np.ndarray,
    base_from_camera_link: np.ndarray,
    camera_link_from_color_optical: np.ndarray,
) -> np.ndarray:
    board_pos_color_optical_h = np.append(np.asarray(board_pos_color_optical), 1.0)
    board_pos_base_h = (
        base_from_camera_link
        @ camera_link_from_color_optical
        @ board_pos_color_optical_h
    )
    return board_pos_base_h[:3]


def _load_extrinsics(path: str) -> tuple[np.ndarray, float] | None:
    """Load calibrated base_link -> camera_link transform from extrinsics.yaml."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
        ext = data["hand_eye_extrinsics"]
        parent = ext.get("parent_frame", _BASE_FRAME)
        child = ext.get("child_frame", _CAMERA_ROOT_FRAME)
        if parent != _BASE_FRAME or child != _CAMERA_ROOT_FRAME:
            raise ValueError(
                "expected parent_frame='base_link' and child_frame='camera_link', "
                f"got parent_frame={parent!r}, child_frame={child!r}"
            )
        t = ext["translation"]
        q = ext["rotation_quat"]
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([q["x"], q["y"], q["z"], q["w"]]).as_matrix()
        T[:3, 3] = [t["x"], t["y"], t["z"]]
        return T, ext.get("reprojection_error_mm", 0.0)
    except Exception as e:
        print(f"Failed to load extrinsics: {e}")
        return None


class CalibrationValidator(Node):
    def __init__(self):
        super().__init__("calibration_validator")

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None
        self._snapshots = []  # list of validation readings

        # Load extrinsics
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("gp4_perception"))
        ext_path = share / "config" / "extrinsics.yaml"
        result = _load_extrinsics(str(ext_path))
        if result is None:
            self.get_logger().error("Cannot load extrinsics.yaml!")
            self._T_base_from_camera_link = None
            self._cal_error = -1
        else:
            self._T_base_from_camera_link, self._cal_error = result
            self.get_logger().info(
                f"Loaded extrinsics (reproj error: {self._cal_error:.2f}mm)"
            )

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._T_camera_link_from_color_optical = None

        # QoS
        _image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        _info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, "/camera/color/image_raw", self._on_image, _image_qos
        )
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._on_info, _info_qos
        )

        self.get_logger().info(
            "Validation tool ready. Jog TCP to board center, press V to snapshot."
        )

    def _on_info(self, msg):
        K = getattr(msg, "K", None) or getattr(msg, "k")
        D = getattr(msg, "D", None) or getattr(msg, "d")
        self._camera_matrix = np.array(K).reshape(3, 3)
        self._dist_coeffs = np.array(D)

    def _get_tcp_position(self) -> np.ndarray | None:
        """Get current tool0 position in base_link frame."""
        try:
            t = self._tf_buffer.lookup_transform(
                _BASE_FRAME, "tool0", rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5),
            )
            p = t.transform.translation
            return np.array([p.x, p.y, p.z])
        except Exception:
            return None

    def _camera_link_from_color_optical(self) -> np.ndarray | None:
        if self._T_camera_link_from_color_optical is not None:
            return self._T_camera_link_from_color_optical
        try:
            t = self._tf_buffer.lookup_transform(
                _CAMERA_ROOT_FRAME,
                _COLOR_OPTICAL_FRAME,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().warning(
                f"Cannot validate calibration: missing {_CAMERA_ROOT_FRAME} <- "
                f"{_COLOR_OPTICAL_FRAME} TF ({exc})"
            )
            return None
        self._T_camera_link_from_color_optical = _transform_to_matrix(t)
        return self._T_camera_link_from_color_optical

    def _on_image(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = display.shape[:2]

        # Detect board
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS
        )

        board_pos_base = None  # board center in robot base frame
        tcp_pos = self._get_tcp_position()
        error_mm = None

        if ids is not None and len(ids) > 0 and self._camera_matrix is not None:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)

            retval, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, _CHARUCO_BOARD,
                cameraMatrix=self._camera_matrix,
                distCoeffs=self._dist_coeffs,
            )

            if ch_corners is not None and retval >= 6:
                cv2.aruco.drawDetectedCornersCharuco(
                    display, ch_corners, ch_ids, _YELLOW
                )
                try:
                    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        ch_corners, ch_ids, _CHARUCO_BOARD,
                        self._camera_matrix, self._dist_coeffs, None, None,
                    )
                except TypeError:
                    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        ch_corners, ch_ids, _CHARUCO_BOARD,
                        self._camera_matrix, self._dist_coeffs,
                    )

                if ok:
                    cv2.drawFrameAxes(
                        display, self._camera_matrix, self._dist_coeffs,
                        rvec, tvec, 0.05,
                    )

                    board_pos_color_optical = np.array(tvec).flatten()
                    camera_link_from_color_optical = (
                        self._camera_link_from_color_optical()
                    )
                    if (
                        self._T_base_from_camera_link is not None
                        and camera_link_from_color_optical is not None
                    ):
                        board_pos_base = _board_position_in_base(
                            board_pos_color_optical,
                            self._T_base_from_camera_link,
                            camera_link_from_color_optical,
                        )

                    # Compute error vs TCP
                    if board_pos_base is not None and tcp_pos is not None:
                        error_mm = np.linalg.norm(board_pos_base - tcp_pos) * 1000

        # ──── HUD ────
        panel_h = 220
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, panel_h), _BG, -1)
        cv2.addWeighted(overlay, 0.75, display, 0.25, 0, display)

        y = 25
        cv2.putText(display, "GP4 Calibration Validation", (10, y),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, _CYAN, 2)
        cv2.putText(display, f"Cal Error: {self._cal_error:.2f}mm", (420, y),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, _GRAY, 1)

        # Board position in robot frame
        y += 30
        if board_pos_base is not None:
            cv2.putText(
                display,
                f"Board (robot): X={board_pos_base[0]*1000:.1f} "
                f"Y={board_pos_base[1]*1000:.1f} "
                f"Z={board_pos_base[2]*1000:.1f} mm",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _GREEN, 1,
            )
        else:
            cv2.putText(display, "Board (robot): NOT DETECTED", (10, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, _RED, 1)

        # TCP position
        y += 25
        if tcp_pos is not None:
            cv2.putText(
                display,
                f"TCP   (robot): X={tcp_pos[0]*1000:.1f} "
                f"Y={tcp_pos[1]*1000:.1f} "
                f"Z={tcp_pos[2]*1000:.1f} mm",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _YELLOW, 1,
            )
        else:
            cv2.putText(display, "TCP   (robot): NO TF", (10, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, _RED, 1)

        # Error
        y += 30
        if error_mm is not None:
            if error_mm < 5:
                err_color = _GREEN
                verdict = "EXCELLENT"
            elif error_mm < 10:
                err_color = _YELLOW
                verdict = "GOOD"
            elif error_mm < 20:
                err_color = (0, 165, 255)
                verdict = "ACCEPTABLE"
            else:
                err_color = _RED
                verdict = "POOR"

            cv2.putText(
                display,
                f"DISTANCE ERROR: {error_mm:.1f}mm  [{verdict}]",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, err_color, 2,
            )

            # Delta breakdown
            y += 25
            delta = (board_pos_base - tcp_pos) * 1000
            cv2.putText(
                display,
                f"dX={delta[0]:.1f}  dY={delta[1]:.1f}  dZ={delta[2]:.1f} mm",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _GRAY, 1,
            )
        else:
            cv2.putText(display, "Jog TCP to board center to measure error",
                         (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, _GRAY, 1)

        # Snapshots
        y += 30
        if self._snapshots:
            avg_err = np.mean(self._snapshots)
            cv2.putText(
                display,
                f"Snapshots: {len(self._snapshots)}  Avg: {avg_err:.1f}mm  "
                f"Last: {self._snapshots[-1]:.1f}mm",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _MAGENTA, 1,
            )

        # Bottom bar
        cv2.rectangle(display, (0, h - 30), (w, h), _BG, -1)
        cv2.putText(display, "V: Save Snapshot  |  Q: Quit", (10, h - 10),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.4, _GRAY, 1)

        cv2.imshow("GP4 Calibration Validation", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("v") or key == ord("V"):
            if error_mm is not None:
                self._snapshots.append(error_mm)
                self.get_logger().info(
                    f"Snapshot #{len(self._snapshots)}: {error_mm:.1f}mm"
                )
            else:
                self.get_logger().warn("Cannot snapshot — no board or TCP detected")
        elif key == ord("q") or key == ord("Q"):
            if self._snapshots:
                avg = np.mean(self._snapshots)
                self.get_logger().info(
                    f"\n=== VALIDATION SUMMARY ===\n"
                    f"  Snapshots: {len(self._snapshots)}\n"
                    f"  Mean error: {avg:.1f}mm\n"
                    f"  Min: {min(self._snapshots):.1f}mm\n"
                    f"  Max: {max(self._snapshots):.1f}mm\n"
                    f"  Cal reproj: {self._cal_error:.2f}mm\n"
                    f"=========================="
                )
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CalibrationValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == "__main__":
    main()
