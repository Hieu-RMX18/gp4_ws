#!/usr/bin/env python3
"""Live camera preview with ArUco/Charuco detection overlay and calibration controls.

Keyboard shortcuts:
    SPACE  — Pause / Resume sample collection
    S      — Solve calibration and save extrinsics.yaml
    R      — Reset (clear all collected samples)
    Q      — Quit

Usage:
    python3 tools/camera_preview.py
"""

import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, Trigger
from interfaces.srv import CalibrateHandEye


# Match fiducials.yaml: DICT_5X5_100
_ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()


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


# Charuco board from fiducials.yaml
_BOARD_ROWS, _BOARD_COLS, _SQUARE_LEN, _MARKER_LEN = _load_board_config()
_CHARUCO_BOARD = cv2.aruco.CharucoBoard_create(
    _BOARD_COLS, _BOARD_ROWS, _SQUARE_LEN, _MARKER_LEN, _ARUCO_DICT
)

# UI Colors
_GREEN = (0, 255, 0)
_RED = (0, 0, 255)
_YELLOW = (0, 255, 255)
_ORANGE = (0, 165, 255)
_CYAN = (255, 255, 0)
_WHITE = (255, 255, 255)
_GRAY = (180, 180, 180)
_BG = (30, 30, 30)


class CameraPreview(Node):
    def __init__(self):
        super().__init__("camera_preview")

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None
        self._sample_count = 0
        self._collecting = True
        self._solve_status = ""  # status message after solve
        self._solve_busy = False

        # QoS matching RealSense v4.57.7
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
        self.create_subscription(
            Int32, "/perception/sample_count", self._on_sample_count, 10
        )

        # Service clients
        self._toggle_client = self.create_client(
            SetBool, "/perception/toggle_collection"
        )
        self._clear_client = self.create_client(
            Trigger, "/perception/clear_samples"
        )
        self._solve_client = self.create_client(
            CalibrateHandEye, "/perception/calibrate_hand_eye"
        )

        self.get_logger().info(
            "Camera Preview started. Keys: SPACE=pause/resume  S=solve  R=reset  Q=quit"
        )

    def _on_info(self, msg):
        K = getattr(msg, "K", None) or getattr(msg, "k")
        D = getattr(msg, "D", None) or getattr(msg, "d")
        self._camera_matrix = np.array(K).reshape(3, 3)
        self._dist_coeffs = np.array(D)

    def _on_sample_count(self, msg):
        self._sample_count = msg.data

    def _toggle_collection(self):
        """Toggle pause/resume."""
        self._collecting = not self._collecting
        req = SetBool.Request()
        req.data = self._collecting
        if self._toggle_client.wait_for_service(timeout_sec=1.0):
            future = self._toggle_client.call_async(req)
            future.add_done_callback(self._toggle_done)
        else:
            self.get_logger().warn("toggle_collection service not available")

    def _toggle_done(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f"Collection: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Toggle failed: {e}")

    def _clear_samples(self):
        """Clear all samples."""
        req = Trigger.Request()
        if self._clear_client.wait_for_service(timeout_sec=1.0):
            future = self._clear_client.call_async(req)
            future.add_done_callback(self._clear_done)
        else:
            self.get_logger().warn("clear_samples service not available")

    def _clear_done(self, future):
        try:
            resp = future.result()
            self._solve_status = f"CLEARED: {resp.message}"
            self.get_logger().info(resp.message)
        except Exception as e:
            self.get_logger().error(f"Clear failed: {e}")

    def _solve_calibration(self):
        """Call solve in background thread."""
        if self._solve_busy:
            return
        self._solve_busy = True
        self._solve_status = "SOLVING..."

        def _call():
            req = CalibrateHandEye.Request()
            req.fiducial_id = (
                f"charuco_{_BOARD_ROWS}x{_BOARD_COLS}_"
                f"{int(_SQUARE_LEN * 1000)}mm_{int(_MARKER_LEN * 1000)}mm"
            )
            req.min_samples = 12
            if self._solve_client.wait_for_service(timeout_sec=2.0):
                future = self._solve_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                try:
                    resp = future.result()
                    if resp.success:
                        self._solve_status = (
                            f"SAVED! Error: {resp.reprojection_error_mm:.2f}mm "
                            f"| {resp.n_samples_collected} samples "
                            f"| {resp.extrinsics_yaml_path}"
                        )
                        self.get_logger().info(self._solve_status)
                    else:
                        self._solve_status = f"FAILED: {resp.failure_reason}"
                        self.get_logger().error(self._solve_status)
                except Exception as e:
                    self._solve_status = f"ERROR: {e}"
                    self.get_logger().error(str(e))
            else:
                self._solve_status = "SERVICE NOT AVAILABLE"
            self._solve_busy = False

        threading.Thread(target=_call, daemon=True).start()

    def _on_image(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = display.shape[:2]

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS
        )

        n_markers = 0 if ids is None else len(ids)
        n_charuco = 0
        board_pose_ok = False
        dist_cm = 0.0

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            n_markers = len(ids)

            if self._camera_matrix is not None:
                retval, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, gray, _CHARUCO_BOARD,
                    cameraMatrix=self._camera_matrix,
                    distCoeffs=self._dist_coeffs,
                )
                n_charuco = retval if retval else 0

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
                        board_pose_ok = True
                        cv2.drawFrameAxes(
                            display, self._camera_matrix, self._dist_coeffs,
                            rvec, tvec, 0.05,
                        )
                        dist_cm = np.linalg.norm(tvec) * 100

        # ──── HUD OVERLAY ────
        panel_h = 200
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, panel_h), _BG, -1)
        cv2.addWeighted(overlay, 0.75, display, 0.25, 0, display)

        y = 25
        # Title
        cv2.putText(display, "GP4 Hand-Eye Calibration", (10, y),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.75, _CYAN, 2)

        # Collection status
        y += 30
        max_samples = 200
        if self._collecting:
            cv2.circle(display, (18, y - 5), 6, _GREEN, -1)
            cv2.putText(display, "COLLECTING", (30, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.55, _GREEN, 2)
        elif self._sample_count >= max_samples:
            cv2.circle(display, (18, y - 5), 6, _YELLOW, -1)
            cv2.putText(display, "AUTO PAUSED (MAX)", (30, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.55, _YELLOW, 2)
        else:
            cv2.circle(display, (18, y - 5), 6, _ORANGE, -1)
            cv2.putText(display, "PAUSED", (30, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.55, _ORANGE, 2)

        # Sample count (big)
        cv2.putText(display, f"Samples: {self._sample_count}/{max_samples}", (200, y),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, _WHITE, 2)
        min_needed = 12
        if self._sample_count >= max_samples:
            cv2.putText(display, "PRESS S TO SOLVE", (430, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.55, _YELLOW, 2)
        elif self._sample_count >= min_needed:
            cv2.putText(display, "READY TO SOLVE", (430, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.55, _GREEN, 2)

        # Detection info
        y += 28
        m_color = _GREEN if n_markers > 0 else _RED
        cv2.putText(display, f"ArUco: {n_markers}", (10, y),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, m_color, 1)

        c_color = _GREEN if n_charuco >= 6 else _ORANGE
        cv2.putText(display, f"Charuco: {n_charuco}/6+", (150, y),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, c_color, 1)

        if board_pose_ok:
            cv2.putText(display, f"Pose: OK  Dist: {dist_cm:.0f}cm", (310, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, _GREEN, 1)
        else:
            cv2.putText(display, "Pose: --", (310, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, _GRAY, 1)

        # Solve status
        y += 28
        if self._solve_status:
            s_color = _GREEN if "SAVED" in self._solve_status else _YELLOW
            if "FAILED" in self._solve_status or "ERROR" in self._solve_status:
                s_color = _RED
            cv2.putText(display, self._solve_status, (10, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, s_color, 1)

        # Keyboard shortcuts bar at bottom
        bar_y = h - 10
        shortcuts = "SPACE: Pause/Resume  |  S: Solve & Save  |  R: Reset  |  Q: Quit"
        cv2.rectangle(display, (0, h - 30), (w, h), _BG, -1)
        cv2.putText(display, shortcuts, (10, bar_y),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.4, _GRAY, 1)

        # ──── DISPLAY ────
        cv2.imshow("GP4 Calibration Preview", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(" "):
            self._toggle_collection()
        elif key == ord("s") or key == ord("S"):
            self._solve_calibration()
        elif key == ord("r") or key == ord("R"):
            self._clear_samples()
        elif key == ord("q") or key == ord("Q"):
            self.get_logger().info("Quit requested.")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == "__main__":
    main()
