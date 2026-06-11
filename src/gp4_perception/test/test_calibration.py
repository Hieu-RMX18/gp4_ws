"""Unit tests for calibration solver logic (gp4_perception.calibration)."""

import tempfile
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml
from rclpy.node import Node


def _solve_synthetic_hand_eye(num_samples: int = 20):
    """Generate synthetic pose pairs and solve with cv2.calibrateHandEye.

    Convention: eye-to-hand, AX = XB where:
      A = R_gripper2base, t_gripper2base  (robot FK)
      B = R_target2cam, t_target2cam      (camera observation)
      X = R_cam2gripper, t_cam2gripper    (unknown extrinsic, what the solver returns)

    For eye-to-hand: target is fixed in base frame.
      T_base2target = known constant
      T_target2cam = T_cam2base^{-1} * T_base2target
      T_cam2base   = T_gripper2base * T_cam2gripper = A * X
      So T_target2cam = (A * X)^{-1} * T_base2target
    """
    # Ground-truth extrinsic X: cam2gripper
    R_x = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    t_x = np.array([[0.05], [-0.10], [0.15]], dtype=np.float64)

    # Target fixed in base frame
    R_base2target = np.eye(3, dtype=np.float64)
    t_base2target = np.array([[0.4], [0.0], [0.1]], dtype=np.float64)

    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    rng = np.random.default_rng(seed=42)
    for _ in range(num_samples):
        # Random gripper pose in base frame (A)
        angle = rng.random(3) * 0.8
        R_a = cv2.Rodrigues(angle)[0]
        t_a = rng.random((3, 1)) * 0.3

        # T_cam2base = A * X
        R_cam2base = R_a @ R_x
        t_cam2base = R_a @ t_x + t_a

        # T_target2cam = T_cam2base^{-1} * T_base2target
        R_cam2base_inv = R_cam2base.T

        R_b = R_cam2base_inv @ R_base2target
        t_b = R_cam2base_inv @ (t_base2target - t_cam2base)

        R_gripper2base_list.append(R_a)
        t_gripper2base_list.append(t_a)
        R_target2cam_list.append(R_b)
        t_target2cam_list.append(t_b)

    R_gripper2base = np.array(R_gripper2base_list)
    t_gripper2base = np.array(t_gripper2base_list).reshape(-1, 3, 1)
    R_target2cam = np.array(R_target2cam_list)
    t_target2cam = np.array(t_target2cam_list).reshape(-1, 3, 1)

    R_est, t_est = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    return R_x, t_x, np.asarray(R_est), np.asarray(t_est).reshape(3)


class TestSyntheticCalibration:
    def test_translation_error_below_5mm(self):
        R_true, t_true, R_est, t_est = _solve_synthetic_hand_eye(num_samples=24)
        t_err = np.linalg.norm(t_true.ravel() - t_est)
        assert t_err < 0.005, f"Translation error {t_err*1000:.2f} mm exceeds 5 mm"

    def test_rotation_error_small(self):
        R_true, _, R_est, _ = _solve_synthetic_hand_eye(num_samples=24)
        trace = np.trace(R_est.T @ R_true)
        angle_err = np.arccos(np.clip((trace - 1) / 2, -1.0, 1.0))
        assert angle_err < np.deg2rad(
            5.0
        ), f"Rotation error {np.rad2deg(angle_err):.2f} deg > 5 deg"

    def test_yaml_written_with_iso_date(self):
        R_true, t_true, R_est, t_est = _solve_synthetic_hand_eye(num_samples=12)
        from scipy.spatial.transform import Rotation

        quat = Rotation.from_matrix(R_est).as_quat()
        from datetime import datetime, timezone

        cal_date = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        extrinsics = {
            "hand_eye_extrinsics": {
                "translation": {
                    "x": float(t_est[0]),
                    "y": float(t_est[1]),
                    "z": float(t_est[2]),
                },
                "rotation_quat": {
                    "x": float(quat[0]),
                    "y": float(quat[1]),
                    "z": float(quat[2]),
                    "w": float(quat[3]),
                },
                "calibration_date": cal_date,
            }
        }
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump(extrinsics, f)
            path = Path(f.name)
        loaded = yaml.safe_load(path.read_text())
        iso_str = loaded["hand_eye_extrinsics"]["calibration_date"]
        dt = datetime.fromisoformat(iso_str.replace("Z", "+00:00"))
        assert dt.tzinfo is not None
        assert (datetime.now(timezone.utc) - dt).total_seconds() < 60
        path.unlink()


def _make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation).reshape(3)
    return transform


def test_pairwise_residual_uses_consistent_hand_eye_motion_order():
    """Residual must compare robot and camera motions in the same frame."""
    from gp4_perception import calibration
    from scipy.spatial.transform import Rotation

    rng = np.random.default_rng(seed=7)

    def random_transform(translation_scale: float) -> np.ndarray:
        rotation = Rotation.from_rotvec(rng.normal(size=3) * 0.5).as_matrix()
        translation = rng.normal(size=3) * translation_scale
        return _make_transform(rotation, translation)

    base_from_color = random_transform(0.5)
    gripper_from_target = random_transform(0.2)
    samples = []
    for _ in range(12):
        gripper_from_base = random_transform(0.8)
        color_from_target = (
            np.linalg.inv(base_from_color)
            @ np.linalg.inv(gripper_from_base)
            @ gripper_from_target
        )
        samples.append(
            (
                gripper_from_base[:3, :3],
                gripper_from_base[:3, 3].reshape(3, 1),
                color_from_target[:3, :3],
                color_from_target[:3, 3].reshape(3, 1),
            )
        )

    residual_mm = calibration._pairwise_translation_residual_mm(
        samples,
        base_from_color[:3, :3],
        base_from_color[:3, 3],
    )

    assert residual_mm < 1e-6


def test_robot_pose_duplicate_gate_rejects_stationary_samples():
    """Stationary duplicate samples must not satisfy hand-eye pose diversity."""
    from gp4_perception import calibration

    rotation = np.eye(3)
    translation = np.array([[0.1], [0.2], [0.3]])
    samples = [(rotation, translation, np.eye(3), np.zeros((3, 1)))]

    duplicate, translation_delta_m, rotation_delta_rad = (
        calibration._is_duplicate_robot_pose(samples, rotation, translation)
    )

    assert duplicate
    assert translation_delta_m == 0.0
    assert rotation_delta_rad == 0.0


def test_base_to_camera_root_uses_realsense_internal_transform():
    """Published extrinsic should attach base_link to camera_link, not optical frame."""
    from gp4_perception import calibration

    base_from_color = np.eye(4)
    base_from_color[:3, 3] = [0.5, 0.0, 1.0]
    root_from_color = np.eye(4)
    root_from_color[:3, 3] = [0.02, 0.0, 0.0]

    base_from_root = calibration._base_from_camera_root(
        base_from_color,
        root_from_color,
    )

    np.testing.assert_allclose(base_from_root[:3, 3], [0.48, 0.0, 1.0])


def test_calibration_service_uses_reliable_qos_for_realsense_topics(
    monkeypatch, tmp_path
):
    """RealSense image and camera-info subscriptions must use RELIABLE QoS
    to match RealSense v4.57.7 publisher defaults."""
    from gp4_perception import calibration
    from rclpy.qos import ReliabilityPolicy

    captured_qos = []

    monkeypatch.setattr(Node, "__init__", lambda self, node_name: None)
    monkeypatch.setattr(
        Node,
        "create_subscription",
        lambda self, msg_type, topic, callback, qos: captured_qos.append(qos),
    )
    monkeypatch.setattr(Node, "create_service", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "create_publisher", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "create_timer", lambda *args, **kwargs: object())
    monkeypatch.setattr(calibration, "Buffer", lambda: object())
    monkeypatch.setattr(calibration, "TransformListener", lambda *args, **kwargs: None)

    calibration.CalibrationService(extrinsics_path=tmp_path / "extrinsics.yaml")

    assert len(captured_qos) == 2
    for qos in captured_qos:
        assert qos.reliability == ReliabilityPolicy.RELIABLE


def test_calibration_service_uses_marker_length_from_fiducials_yaml(
    monkeypatch, tmp_path
):
    """Pose estimation must use the configured physical marker size."""
    from gp4_perception import calibration

    fiducials_path = tmp_path / "fiducials.yaml"
    fiducials_path.write_text("fiducials:\n  marker_length_m: 0.052\n")
    captured_marker_lengths = []

    class FakeBridge:
        def imgmsg_to_cv2(self, msg, desired_encoding):
            return np.zeros((4, 4, 3), dtype=np.uint8)

    class FakeTfBuffer:
        def lookup_transform(self, target_frame, source_frame, stamp, timeout):
            return SimpleNamespace(
                transform=SimpleNamespace(
                    translation=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )

    def estimate_pose_single_markers(
        corners, marker_length, camera_matrix, dist_coeffs
    ):
        captured_marker_lengths.append(marker_length)
        return (
            np.zeros((1, 1, 3), dtype=np.float64),
            np.zeros((1, 1, 3), dtype=np.float64),
            None,
        )

    monkeypatch.setattr(Node, "__init__", lambda self, node_name: None)
    monkeypatch.setattr(Node, "create_subscription", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "create_service", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "create_publisher", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "create_timer", lambda *args, **kwargs: object())
    import logging as _logging
    _test_logger = _logging.getLogger("test_calibration")
    monkeypatch.setattr(Node, "get_logger", lambda self: _test_logger)
    monkeypatch.setattr(calibration, "CvBridge", lambda: FakeBridge())
    monkeypatch.setattr(calibration, "Buffer", lambda: FakeTfBuffer())
    monkeypatch.setattr(calibration, "TransformListener", lambda *args, **kwargs: None)
    monkeypatch.setattr(calibration.cv2, "cvtColor", lambda image, code: image)
    monkeypatch.setattr(
        calibration.cv2.aruco,
        "detectMarkers",
        lambda image, dictionary, parameters: (
            [np.zeros((4, 1, 2), dtype=np.float32)],
            np.array([[1]], dtype=np.int32),
            None,
        ),
    )
    monkeypatch.setattr(
        calibration.cv2.aruco,
        "estimatePoseSingleMarkers",
        estimate_pose_single_markers,
    )

    service = calibration.CalibrationService(
        extrinsics_path=tmp_path / "extrinsics.yaml"
    )
    service._on_camera_info(
        SimpleNamespace(
            K=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            D=[0.0, 0.0, 0.0, 0.0, 0.0],
        )
    )
    service._on_image(SimpleNamespace(header=SimpleNamespace(stamp=object())))

    assert captured_marker_lengths == [0.052]


def test_duplicate_sample_warning_does_not_crash_with_rclpy_logger(
    monkeypatch, tmp_path
):
    """Duplicate sample rejection must not crash the calibration node."""
    from gp4_perception import calibration

    fiducials_path = tmp_path / "fiducials.yaml"
    fiducials_path.write_text("fiducials:\n  marker_length_m: 0.052\n")
    warnings = []

    class FakeLogger:
        def info(self, message):
            warnings.append(message)

        def warning(self, message):
            warnings.append(message)

        def error(self, message):
            pass

        def debug(self, message):
            warnings.append(message)

    class FakeBridge:
        def imgmsg_to_cv2(self, msg, desired_encoding):
            return np.zeros((4, 4, 3), dtype=np.uint8)

    class FakeTfBuffer:
        def lookup_transform(self, target_frame, source_frame, stamp, timeout):
            return SimpleNamespace(
                transform=SimpleNamespace(
                    translation=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )

    monkeypatch.setattr(Node, "__init__", lambda self, node_name: None)
    monkeypatch.setattr(Node, "create_subscription", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "create_service", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "create_publisher", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "create_timer", lambda *args, **kwargs: object())
    monkeypatch.setattr(Node, "get_logger", lambda self: FakeLogger())
    monkeypatch.setattr(calibration, "CvBridge", lambda: FakeBridge())
    monkeypatch.setattr(calibration, "Buffer", lambda: FakeTfBuffer())
    monkeypatch.setattr(calibration, "TransformListener", lambda *args, **kwargs: None)
    monkeypatch.setattr(calibration.cv2, "cvtColor", lambda image, code: image)
    monkeypatch.setattr(
        calibration.cv2.aruco,
        "detectMarkers",
        lambda image, dictionary, parameters: (
            [np.zeros((4, 1, 2), dtype=np.float32)],
            np.array([[1]], dtype=np.int32),
            None,
        ),
    )
    monkeypatch.setattr(
        calibration.cv2.aruco,
        "estimatePoseSingleMarkers",
        lambda corners, marker_length, camera_matrix, dist_coeffs: (
            np.zeros((1, 1, 3), dtype=np.float64),
            np.zeros((1, 1, 3), dtype=np.float64),
            None,
        ),
    )

    service = calibration.CalibrationService(
        extrinsics_path=tmp_path / "extrinsics.yaml"
    )
    service._on_camera_info(
        SimpleNamespace(
            K=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            D=[0.0, 0.0, 0.0, 0.0, 0.0],
        )
    )
    service._samples.append(
        (np.eye(3), np.array([[0.1], [0.2], [0.3]]), np.eye(3), np.zeros((3, 1)))
    )

    # Seed the stationary-pose state so the robot appears to have been
    # stationary for >1 s already.  FakeTfBuffer returns (0.1, 0.2, 0.3)
    # identity-rotation, matching the existing sample — duplicate check fires.
    import time as _t
    service._last_robot_pose = (np.eye(3), np.array([[0.1], [0.2], [0.3]]))
    service._last_robot_pose_time = _t.monotonic() - 2.0

    service._on_image(SimpleNamespace(header=SimpleNamespace(stamp=object())))

    assert len(service._samples) == 1
    assert any("Rejected calibration sample" in message or "DUPLICATE" in message for message in warnings)


def test_fiducial_config_loads_charuco_board_dimensions(tmp_path):
    """Charuco pose estimation must use printed square and marker dimensions."""
    from gp4_perception import calibration

    fiducials_path = tmp_path / "fiducials.yaml"
    fiducials_path.write_text(
        "\n".join(
            [
                "fiducials:",
                "  target_type: charuco",
                "  marker_dictionary: DICT_5X5_100",
                "  board_rows: 10",
                "  board_columns: 11",
                "  square_length_m: 0.020",
                "  marker_length_m: 0.015",
            ]
        )
    )

    fiducials = calibration._load_fiducial_config(fiducials_path)

    assert fiducials is not None
    assert fiducials.target_type == "charuco"
    assert fiducials.rows == 10
    assert fiducials.cols == 11
    assert fiducials.square_length_m == 0.020
    assert fiducials.marker_length_m == 0.015


def test_tf_publisher_rejects_invalid_reprojection_error(monkeypatch, tmp_path):
    """Invalid calibration must not be broadcast into TF."""
    from gp4_perception import calibration

    extrinsics_path = tmp_path / "extrinsics.yaml"
    extrinsics_path.write_text(
        yaml.dump(
            {
                "hand_eye_extrinsics": {
                    "parent_frame": "base_link",
                    "child_frame": "camera_link",
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation_quat": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "calibration_date": "2026-05-23T00:00:00Z",
                    "reprojection_error_mm": 111.0,
                }
            }
        )
    )
    sent = []

    class FakeBroadcaster:
        def __init__(self, _node):
            pass

        def sendTransform(self, transform):
            sent.append(transform)

    class FakeClock:
        def now(self):
            from builtin_interfaces.msg import Time

            return SimpleNamespace(to_msg=lambda: Time())

    monkeypatch.setattr(Node, "__init__", lambda self, node_name: None)
    monkeypatch.setattr(Node, "get_clock", lambda self: FakeClock())
    monkeypatch.setattr(calibration, "StaticTransformBroadcaster", FakeBroadcaster)
    logged_errors = []
    monkeypatch.setattr(
        calibration._LOGGER,
        "error",
        lambda message, *args: logged_errors.append(message % args),
    )

    calibration.TFPublisher(extrinsics_path=extrinsics_path)

    assert sent == []
    assert any("reprojection_error_mm" in message for message in logged_errors)

def test_tf_publisher_rejects_legacy_optical_child_frame(monkeypatch, tmp_path):
    """Calibration YAML must publish the ROS camera root, not the optical frame."""
    from gp4_perception import calibration

    extrinsics_path = tmp_path / "extrinsics.yaml"
    extrinsics_path.write_text(
        yaml.dump(
            {
                "hand_eye_extrinsics": {
                    "parent_frame": "base_link",
                    "child_frame": "camera_color_optical_frame",
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation_quat": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "calibration_date": "2026-05-23T00:00:00Z",
                    "reprojection_error_mm": 1.0,
                }
            }
        )
    )
    sent = []

    class FakeBroadcaster:
        def __init__(self, _node):
            pass

        def sendTransform(self, transform):
            sent.append(transform)

    class FakeClock:
        def now(self):
            from builtin_interfaces.msg import Time

            return SimpleNamespace(to_msg=lambda: Time())

    monkeypatch.setattr(Node, "__init__", lambda self, node_name: None)
    monkeypatch.setattr(Node, "get_clock", lambda self: FakeClock())
    monkeypatch.setattr(calibration, "StaticTransformBroadcaster", FakeBroadcaster)
    logged_errors = []
    monkeypatch.setattr(
        calibration._LOGGER,
        "error",
        lambda message, *args: logged_errors.append(message % args),
    )

    calibration.TFPublisher(extrinsics_path=extrinsics_path)

    assert sent == []
    assert any("child_frame" in message for message in logged_errors)
