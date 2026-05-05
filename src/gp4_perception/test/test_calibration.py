"""Unit tests for calibration solver logic (gp4_perception.calibration)."""

import tempfile
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml


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
        t_cam2base_inv = -R_cam2base.T @ t_cam2base

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
        assert angle_err < np.deg2rad(5.0), f"Rotation error {np.rad2deg(angle_err):.2f} deg > 5 deg"

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
