from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import cv2
import numpy as np

from window_cleaner.window_geometry import WindowCornersPx


@dataclass(frozen=True)
class Point2D:
    x_cm: float
    y_cm: float


@dataclass(frozen=True)
class WindowBIM2D:
    coordinate_frame: str
    unit: str
    bottom_left: Point2D
    bottom_right: Point2D
    top_right: Point2D
    top_left: Point2D
    width_cm: float
    height_cm: float


@dataclass(frozen=True)
class WindowPose:
    rotation_vector: tuple[float, float, float]
    translation_vector_cm: tuple[float, float, float]
    rotation_matrix: list[list[float]]
    camera_position_in_window_cm: tuple[float, float, float]
    reprojection_rmse_px: float


def build_rectangular_bim(
    width_cm: float,
    height_cm: float,
    map_bottom_left: tuple[float, float] = (0.0, 0.0),
) -> WindowBIM2D:
    if width_cm <= 0 or height_cm <= 0:
        raise ValueError("window width and height must be positive")

    x0, y0 = float(map_bottom_left[0]), float(map_bottom_left[1])
    width, height = float(width_cm), float(height_cm)
    return WindowBIM2D(
        coordinate_frame="map",
        unit="cm",
        bottom_left=Point2D(x0, y0),
        bottom_right=Point2D(x0 + width, y0),
        top_right=Point2D(x0 + width, y0 + height),
        top_left=Point2D(x0, y0 + height),
        width_cm=width,
        height_cm=height,
    )


def window_object_points(width_cm: float, height_cm: float) -> np.ndarray:
    if width_cm <= 0 or height_cm <= 0:
        raise ValueError("window width and height must be positive")
    return np.array(
        [
            [0.0, height_cm, 0.0],
            [width_cm, height_cm, 0.0],
            [width_cm, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )


def estimate_window_pose(
    image_corners: WindowCornersPx,
    width_cm: float,
    height_cm: float,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray | None = None,
) -> WindowPose:
    object_points = window_object_points(width_cm, height_cm)
    image_points = np.asarray(image_corners.ordered(), dtype=np.float64)
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    distortion = (
        np.zeros((1, 5), dtype=np.float64)
        if distortion_coefficients is None
        else np.asarray(distortion_coefficients, dtype=np.float64)
    )

    success, rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        image_points,
        matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not success:
        raise RuntimeError("PnP pose estimation failed")

    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        matrix,
        distortion,
    )
    residuals = projected.reshape(-1, 2) - image_points
    reprojection_rmse = float(np.sqrt(np.mean(np.sum(residuals**2, axis=1))))

    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    camera_position = -rotation_matrix.T @ translation_vector
    return WindowPose(
        rotation_vector=tuple(float(value) for value in rotation_vector.reshape(3)),
        translation_vector_cm=tuple(float(value) for value in translation_vector.reshape(3)),
        rotation_matrix=rotation_matrix.astype(float).tolist(),
        camera_position_in_window_cm=tuple(float(value) for value in camera_position.reshape(3)),
        reprojection_rmse_px=reprojection_rmse,
    )


def save_bim_result(
    path: str | Path,
    bim: WindowBIM2D,
    pose: WindowPose,
    detected_top_left_px: tuple[float, float],
    detected_top_right_px: tuple[float, float],
    camera_map_position_cm: tuple[float, float] = (15.0, 30.0),
) -> None:
    result = {
        "bim_2d": asdict(bim),
        "pnp_pose": asdict(pose),
        "source_observations": {
            "robot_initial_position": "map_bottom_left",
            "map_origin_cm": [0.0, 0.0],
            "camera_map_position_cm": list(map(float, camera_map_position_cm)),
            "detected_top_left_px": list(map(float, detected_top_left_px)),
            "detected_top_right_px": list(map(float, detected_top_right_px)),
            "pnp_correspondence_note": (
                "PnP requires four 2D-3D correspondences; all four detected image corners "
                "were used. The 2D BIM rectangle is anchored at the robot's initial map bottom-left."
            ),
        },
    }
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
