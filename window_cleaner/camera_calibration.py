from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraCalibrationParameters:
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    calibration_width: int = 640
    calibration_height: int = 480

    def camera_matrix_for_image(self, width: int, height: int) -> np.ndarray:
        if width <= 0 or height <= 0:
            raise ValueError("image width and height must be positive")

        scale_x = width / float(self.calibration_width)
        scale_y = height / float(self.calibration_height)
        scaled = self.camera_matrix.astype(np.float64).copy()
        scaled[0, 0] *= scale_x
        scaled[0, 2] *= scale_x
        scaled[1, 1] *= scale_y
        scaled[1, 2] *= scale_y
        return scaled

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        if frame is None or frame.size == 0:
            raise ValueError("frame must be a non-empty image")
        height, width = frame.shape[:2]
        matrix = self.camera_matrix_for_image(width, height)
        return cv2.undistort(frame, matrix, self.distortion_coefficients)


def load_camera_calibration(
    path: str | Path,
    calibration_width: int = 640,
    calibration_height: int = 480,
) -> CameraCalibrationParameters:
    calibration_path = Path(path)
    if not calibration_path.exists():
        raise FileNotFoundError(f"Camera calibration file not found: {calibration_path}")

    with np.load(calibration_path) as data:
        if "mtx" not in data or "dist" not in data:
            raise ValueError("Camera calibration NPZ must contain 'mtx' and 'dist'")
        camera_matrix = np.asarray(data["mtx"], dtype=np.float64)
        distortion_coefficients = np.asarray(data["dist"], dtype=np.float64)

    if camera_matrix.shape != (3, 3):
        raise ValueError(f"Expected camera matrix shape (3, 3), got {camera_matrix.shape}")
    if distortion_coefficients.size < 4:
        raise ValueError("Distortion coefficients must contain at least four values")
    if calibration_width <= 0 or calibration_height <= 0:
        raise ValueError("calibration image width and height must be positive")

    return CameraCalibrationParameters(
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
        calibration_width=calibration_width,
        calibration_height=calibration_height,
    )
